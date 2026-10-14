import rclpy
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

# ── IK & TF2 ──
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_transformations as tft

# ─── TARGET TAG SELECTION ─────────────────────────────────────────────────────
# Default target; overwritten at runtime by an /ssvep_command message from MATLAB.
# Must be one of the IDs declared in tags_36h11.yaml.
TARGET_TAG_ID = 5

# ─── SSVEP CLASS INDEX → APRILTAG ID ──────────────────────────────────────────
# MATLAB publishes a std_msgs/Int32 class index on /ssvep_command. This is the
# single source of truth mapping that index to the AprilTag the arm fetches.
#   class 0 (8 Hz, HOME) intentionally has no tag — it just (re)homes.
FREQ_TO_TAG = {
    1: 5,   # 12 Hz -> Lotion   -> tag 0
    2: 1,   # 15 Hz -> Lip Balm -> tag 1
    3: 2,   # 18 Hz -> Red Cup  -> tag 2
}
HOME_CLASS_INDEX = 0   # class index that means "go/stay home", no fetch

# ─── SCAN WAYPOINTS ───────────────────────────────────────────────────────────
SCAN_WAYPOINTS = [
    [0.34,  0.03,  2.76, -1.88,  0.01, -1.19,  1.53],
    [-0.32, 0.00,  2.17, -1.81,  0.03, -1.32,  0.26],
    [0.85, -0.05,  3.11, -1.91, -0.03, -1.25,  2.41],
    [0.79,  0.29,  3.14, -1.45, -0.00, -1.33,  2.37],
    [0.00,  0.33,  3.14, -1.51,  0.00, -1.20,  1.57],
    [-0.47, 0.22,  2.62, -1.49,  0.20, -1.39,  0.56],
    [-0.06, 0.66,  2.40, -0.97,  0.45, -1.61,  0.92],
    [0.34,  0.72,  2.40, -0.87,  0.46, -1.65,  1.38],
    [0.65,  0.93,  2.46, -0.28,  0.56, -1.95,  2.03],
    [0.00,  0.26,  3.14, -2.27,  0.00,  0.96,  1.57],   # home (index 9)
]

# Home pose to return to after delivery (the 'home' waypoint above).
HOME_WAYPOINT = [0.00, 0.26, 3.14, -2.27, 0.00, 0.96, 1.57]
RETURN_HOME_DURATION_SEC = 4.0

# ─── SCAN TIMING ──────────────────────────────────────────────────────────────
MOVE_DURATION_SEC = 2.7   # was 4 -> ~1.5x faster joint moves
SCAN_INTERVAL_SEC = 4
INIT_DELAY_SEC    = 0.5

# ─── SERVOING PARAMETERS ─────────────────────────────────────────────────────
IMG_W, IMG_H  = 640, 360
TARGET_CX     = IMG_W / 2
TARGET_CY     = IMG_H / 2
TARGET_SIZE   = 100
BORDER_MARGIN = 75

KP_XY       = 0.0006
KP_Z        = 0.0009
DEAD_XY     = 10
DEAD_Z      = 15
MAX_LIN_VEL = 0.12   # was 0.08 -> ~1.5x faster servoing approach
MIN_LIN_VEL = 0.01

# ─── CONVERGENCE HOLD ────────────────────────────────────────────────────────
CONVERGE_HOLD_SEC = 1.0

# ─── VISUAL FREEZE WATCHDOG (servoing only) ──────────────────────────────────
# A frozen arm = total error magnitude doesn't improve at all over a window.
# Slow steady approach (error shrinking a little each sample) is NOT frozen, so
# we compare against a sample from FREEZE_SEC ago, not the previous sample.
VISUAL_FREEZE_SEC      = 3.0    # window length to judge progress over
VISUAL_FREEZE_MIN_PROG = 2.0    # px; min total error-magnitude improvement over the window

# ─── GRASP SEQUENCE (Cartesian Deltas via IK) ────────────────────────────────
# Step format: (dx_m, dy_m, dz_m, gripper_pos, duration_sec, label)
GRIPPER_OPEN  = 0.0
GRIPPER_CLOSE = 0.5
# If the gripper jaws close across the wrong axis of the tag, set this to
# +/- math.pi/2 to rotate the preserved yaw by 90deg. Test on a tilted tag.
GRASP_YAW_OFFSET = 0.0
# When True, the grasp wrist yaw follows the detected AprilTag's in-plane
# image rotation (computed from corner pixels at convergence). Image-space
# only — no hand-eye / camera-to-robot calibration is used, consistent with
# the IBVS approach. Set False to fall back to plain yaw preservation.
ALIGN_WRIST_TO_TAG = True
# Collapse the square tag's 4-fold corner-ordering symmetry into a +/-45deg
# band so the wrist never turns more than 45deg to align. Disable if you want
# the raw measured angle (e.g. for a non-square / asymmetric target).
FOLD_TAG_SYMMETRY = True
GRASP_STEPS = [
    (0.0,  0.0,  0.0,   GRIPPER_OPEN,  1.5, 'Open gripper'),
    (0.04, 0.0,  0.0,   None,          1.5, 'Align horizontally (-1cm X)'),
    (0.0,  0.01, 0.0,  None,           1.5, 'Align horizontally (-1cm Y)'),
    (0.0,  0.0, -0.06,  None,          2.0, 'Descend to box (2cm)'),
    (0.0,  0.0,  0.0,   GRIPPER_CLOSE, 1.5, 'Close gripper (grasp)'),
    (0.0,  0.0,  0.15,  None,          2.0, 'Lift object (15cm)'),
]
GRIPPER_TIMEOUT = 5.0

# ─── DELIVERY (absolute pose, top-down, works from any grasp location) ───────
# base_link has buffer_length=0 (no TF data) — use 'world'. world->base_link
# is a static identity transform, so captured numbers stay valid.
DELIVERY_FRAME = 'world'

# Captured drop pose (tf2_echo world -> end_effector_link). This is where the
# end-effector itself should sit so the user can take the tag from the gripper.
DROP_X = 0.456
DROP_Y = 0.395
DROP_Z_TABLE = 0.314

# Captured orientation at that pose (xyzw) — already top-down, ~90deg yaw.
TOPDOWN_QUAT = (0.709, 0.704, -0.031, 0.040)

DROP_APPROACH_HEIGHT = 0.15   # hover this far above the drop point before lowering

GRIPPER_RELEASE_WAIT_SEC = 1.0   # pause before opening so user can take the tag

# Step format: (abs_x, abs_y, abs_z, gripper_pos, duration_sec, label)
# abs_* are absolute positions in DELIVERY_FRAME; orientation is always TOPDOWN_QUAT.
# gripper_pos 'WAIT_THEN_OPEN' = hold position, wait, then open (user grabs tag).
DELIVERY_STEPS = [
    (DROP_X, DROP_Y, DROP_Z_TABLE + DROP_APPROACH_HEIGHT, None,             4.0, 'Move above drop position'),
    (DROP_X, DROP_Y, DROP_Z_TABLE,                        None,             2.5, 'Lower to drop height'),
    (DROP_X, DROP_Y, DROP_Z_TABLE,                        'WAIT_THEN_OPEN', GRIPPER_RELEASE_WAIT_SEC, 'Wait 3s then release'),
    (DROP_X, DROP_Y, DROP_Z_TABLE + DROP_APPROACH_HEIGHT, None,             2.5, 'Retreat upward'),
]


class ScanAndReach(Node):

    def __init__(self):
        super().__init__('scan_and_reach')

        # ── Publishers / subscribers / service clients ──
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.twist_pub = self.create_publisher(
            Twist, '/twist_controller/commands', 10)
        self.switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_cb, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)

        # SSVEP command from MATLAB: class index -> AprilTag id -> start a run.
        self.create_subscription(
            Int32, '/ssvep_command', self.ssvep_cmd_cb, 10)

        # Gripper action client
        self.gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

        # ── TF2 and IK Setup ──
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── State machine ──
        self.state            = 'waiting'   # idle until an /ssvep_command arrives
        self.target_tag_id    = TARGET_TAG_ID
        self.current_waypoint = 0
        self.servoing_done    = False
        self.latest_joints    = [0.0] * 7
        self.init_done        = False
        self.last_detection_waypoint = 0
        self.grasp_yaw_abs    = None   # absolute world-Z wrist yaw fixed at convergence

        # ── Smooth 50 Hz twist publishing ──
        self.active_twist  = Twist()
        self.last_det_time = self.get_clock().now()
        self.create_timer(0.02, self.publish_twist_loop)

        # ── Convergence hold timer ──
        self.converge_start_time = None

        # ── Visual freeze watchdog state ──
        self.freeze_best_mag = None
        self.freeze_progress_time = self.get_clock().now()

        # ── Grasp / delivery sequence state ──
        self.grasp_step_index    = 0
        self.delivery_step_index = 0

        # ── Start sequence ──
        # Reset controllers now, but DO NOT begin scanning. The node sits idle
        # until ssvep_cmd_cb receives a target on /ssvep_command.
        self.reset_controllers()
        self.init_timer = None
        self.scan_timer = None
        self.get_logger().info(
            'Scan and Reach ready — idle, waiting for /ssvep_command.')

    # ═══════════════════════════════════════════════════════════════════════════
    # SSVEP COMMAND → START A RUN
    # ═══════════════════════════════════════════════════════════════════════════

    def ssvep_cmd_cb(self, msg):
        cls = int(msg.data)

        # Once a run is committed (scanning / servoing / grasping / delivering),
        # ignore ALL incoming commands — including home. This lets the user stop
        # looking at the LED the moment a run begins; the arm completes the full
        # scan -> grasp -> deliver -> home cycle autonomously.
        if self.state not in ('waiting', 'done'):
            self.get_logger().warn(
                f'SSVEP command {cls} ignored — run in progress (state={self.state}).')
            return

        # Home command (only honoured while idle): just (re)home, no fetch.
        if cls == HOME_CLASS_INDEX:
            self.get_logger().info('SSVEP home command received — returning home.')
            self.set_state('done')
            self.send_joint_goal(HOME_WAYPOINT, duration_sec=RETURN_HOME_DURATION_SEC)
            return

        if cls not in FREQ_TO_TAG:
            self.get_logger().warn(f'SSVEP command {cls} has no tag mapping — ignored.')
            return

        self.target_tag_id = FREQ_TO_TAG[cls]
        self.get_logger().info(
            f'SSVEP command {cls} -> AprilTag {self.target_tag_id}. Starting scan.')
        self.start_scan()

    def start_scan(self):
        # Fresh run from the first waypoint.
        self.set_state('scanning')
        self.current_waypoint    = 0
        self.servoing_done       = False
        self.init_done           = False
        self.converge_start_time = None
        self.freeze_best_mag     = None
        self.freeze_progress_time = self.get_clock().now()
        self.grasp_step_index    = 0
        self.delivery_step_index = 0
        self.grasp_yaw_abs       = None

        if self.init_timer is not None:
            try:
                self.init_timer.cancel()
            except Exception:
                pass
        if self.scan_timer is not None:
            try:
                self.scan_timer.cancel()
            except Exception:
                pass
        self.init_timer = self.create_timer(INIT_DELAY_SEC, self.init_scan)
        self.scan_timer = self.create_timer(SCAN_INTERVAL_SEC, self.scan_step)

    def set_state(self, new_state):
        self.state = new_state

    # ═══════════════════════════════════════════════════════════════════════════
    # PHASE 1 — RESET CONTROLLERS
    # ═══════════════════════════════════════════════════════════════════════════

    def reset_controllers(self):
        self.get_logger().info('Waiting for controller manager...')
        self.switch_client.wait_for_service(timeout_sec=5.0)
        req = SwitchController.Request()
        req.activate_controllers   = ['joint_trajectory_controller']
        req.deactivate_controllers = ['twist_controller']
        req.strictness = 2
        future = self.switch_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                'Controllers reset.' if f.result().ok
                else 'Reset skipped (already in correct state).'))

    # ═══════════════════════════════════════════════════════════════════════════
    # PHASE 2 — SCANNING
    # ═══════════════════════════════════════════════════════════════════════════

    def init_scan(self):
        start_wp = self.current_waypoint
        self.get_logger().info(f'Init: sending waypoint {start_wp}')
        self.send_joint_goal(SCAN_WAYPOINTS[start_wp])
        self.current_waypoint = start_wp + 1
        self.init_done = True
        self.init_timer.cancel()

    def scan_step(self):
        if not self.init_done:
            return
        if self.state != 'scanning':
            return
        if self.current_waypoint >= len(SCAN_WAYPOINTS):
            self.get_logger().warn(
                'Full scan complete — tag not found. Returning to idle.')
            self.scan_timer.cancel()
            self.set_state('waiting')
            return
        self.get_logger().info(f'Scan waypoint {self.current_waypoint}')
        self.send_joint_goal(SCAN_WAYPOINTS[self.current_waypoint])
        self.current_waypoint += 1

    def _shortest_path_targets(self, current, target):
        """Adjust each target joint by +/-2*pi so the controller interpolates
        the SHORT way from current to target. Prevents the 'elbow twist' where
        a joint unwinds nearly a full turn (e.g. joint_1 driving 3.0 -> 0.0 the
        long way, or joint_4's +360 deg offset) instead of taking the near path.
        Only adjusts by full turns, so the physical target pose is unchanged."""
        adjusted = []
        for c, t in zip(current, target):
            diff = t - c
            while diff > math.pi:
                t -= 2.0 * math.pi
                diff = t - c
            while diff < -math.pi:
                t += 2.0 * math.pi
                diff = t - c
            adjusted.append(t)
        return adjusted

    def send_joint_goal(self, positions, duration_sec=MOVE_DURATION_SEC):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1','joint_2','joint_3',
                        'joint_4','joint_5','joint_6','joint_7']
        start = list(self.latest_joints[:7])
        # Take the shortest angular path to the target to avoid long-way twists.
        target = self._shortest_path_targets(start, list(positions))
        pt0 = JointTrajectoryPoint()
        pt0.positions = start
        pt0.time_from_start = Duration(sec=0, nanosec=0)
        pt1 = JointTrajectoryPoint()
        pt1.positions = target
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt1.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points = [pt0, pt1]
        self.traj_pub.publish(msg)

    def joint_cb(self, msg):
        # Map by joint NAME, not position index — robust to publish order and
        # to the gripper joint appearing in the same message.
        if not msg.position:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.latest_joints = [
                float(name_to_pos[f'joint_{i}']) for i in range(1, 8)]
        except KeyError:
            # Fallback: arm joints not all present yet — keep previous values
            pass

    # ═══════════════════════════════════════════════════════════════════════════
    # PHASE 3 — TAG DETECTED → SWITCH TO SERVOING
    # ═══════════════════════════════════════════════════════════════════════════

    def detection_cb(self, msg):
        if self.state == 'scanning':
            if self.current_waypoint < 3:
                return
            valid = [d for d in msg.detections
                     if d.id == self.target_tag_id
                     and self.get_size(d) > 25
                     and BORDER_MARGIN < d.centre.x < (IMG_W - BORDER_MARGIN)
                     and BORDER_MARGIN < d.centre.y < (IMG_H - BORDER_MARGIN)]
            if valid:
                self.last_detection_waypoint = max(0, self.current_waypoint - 1)
                self.get_logger().info(
                    f'Tag {valid[0].id} found at waypoint '
                    f'{self.last_detection_waypoint}! Starting servoing...')
                self.scan_timer.cancel()
                self.set_state('switching')
                self.switch_to_twist()

        elif self.state == 'switching':
            return

        elif self.state == 'servoing':
            valid = [d for d in msg.detections
                     if d.id == self.target_tag_id and self.get_size(d) > 25]
            if not valid:
                self.stop_twist()
                return
            self.run_servoing(valid[0])

    def switch_to_twist(self):
        req = SwitchController.Request()
        req.activate_controllers   = ['twist_controller']
        req.deactivate_controllers = ['joint_trajectory_controller']
        req.strictness = 2
        future = self.switch_client.call_async(req)
        future.add_done_callback(self.switch_done_cb)
        self.get_logger().info('Switching to twist controller...')

    def switch_done_cb(self, future):
        if self.state != 'switching':
            return
        if future.result().ok:
            self.get_logger().info('Switched! Settling for 1.5s...')
            self.settle_timer = self.create_timer(1.5, self.start_servoing)
        else:
            self.get_logger().error('Controller switch failed!')
            self.set_state('scanning')

    def start_servoing(self):
        self.settle_timer.cancel()
        self.set_state('servoing')
        self.stop_twist()
        self.freeze_best_mag = None
        self.freeze_progress_time = self.get_clock().now()
        self.get_logger().info('Servoing active.')

    # ═══════════════════════════════════════════════════════════════════════════
    # PHASE 4 — 3-DOF IBVS
    # ═══════════════════════════════════════════════════════════════════════════

    def get_size(self, det):
        cx, cy = det.centre.x, det.centre.y
        return sum(math.sqrt((c.x - cx)**2 + (c.y - cy)**2)
                   for c in det.corners) / 4

    def _tag_image_yaw(self, det):
        """In-plane rotation of the tag in the IMAGE, in radians, derived from
        its corner pixels. apriltag orders the corners consistently, so the
        vector corner[0]->corner[1] is one tag edge and its angle tracks the
        tag's rotation. Purely image-space — no camera/robot calibration."""
        c0 = det.corners[0]
        c1 = det.corners[1]
        # Leading minus: image y-axis points down, so negate to get a yaw whose
        # sign matches the wrist's world-Z rotation direction (confirmed in lab).
        yaw = -math.atan2(c1.y - c0.y, c1.x - c0.x)
        if FOLD_TAG_SYMMETRY:
            # collapse 4-fold square symmetry into a +/-45deg band
            yaw = ((yaw + math.pi / 4) % (math.pi / 2)) - math.pi / 4
        return yaw

    def calc_velocity(self, error, kp, max_vel, min_vel, deadzone):
        if abs(error) < deadzone:
            return 0.0
        cmd = error * kp
        cmd = math.copysign(max(min_vel, abs(cmd)), cmd)
        return math.copysign(min(max_vel, abs(cmd)), cmd)

    def run_servoing(self, det):
        if self.servoing_done:
            self.stop_twist()
            return

        size  = self.get_size(det)
        err_x = det.centre.x - TARGET_CX
        err_y = det.centre.y - TARGET_CY
        err_z = size - TARGET_SIZE

        # ── Windowed freeze detection ──
        # Track the best (smallest) total error magnitude and when we last saw a
        # meaningful improvement. If no improvement for the whole window, the arm
        # is genuinely stuck (not just approaching slowly).
        err_mag = math.sqrt(err_x**2 + err_y**2 + err_z**2)
        now = self.get_clock().now()
        if self.freeze_best_mag is None or err_mag < self.freeze_best_mag - VISUAL_FREEZE_MIN_PROG:
            self.freeze_best_mag = err_mag
            self.freeze_progress_time = now
        else:
            stalled = (now - self.freeze_progress_time).nanoseconds / 1e9
            if stalled > VISUAL_FREEZE_SEC:
                self.get_logger().error(
                    f'Visual freeze {stalled:.1f}s '
                    f'(no progress: X:{err_x:.0f} Y:{err_y:.0f} '
                    f'Z:{err_z:.0f} mag:{err_mag:.0f}) — restarting...')
                self.trigger_restart()
                return

        self.get_logger().info(
            f'Servo  X:{err_x:+.0f}px  Y:{err_y:+.0f}px  '
            f'Z:{err_z:+.0f}px  size:{size:.0f}')

        if (abs(err_x) < DEAD_XY and abs(err_y) < DEAD_XY
                and abs(err_z) < DEAD_Z):
            if self.converge_start_time is None:
                self.converge_start_time = self.get_clock().now()
            else:
                held = (self.get_clock().now()
                        - self.converge_start_time).nanoseconds / 1e9
                if held >= CONVERGE_HOLD_SEC:
                    tag_img_yaw = self._tag_image_yaw(det)
                    # Resolve the ABSOLUTE world-Z wrist yaw ONCE here. Reading
                    # EE yaw per grasp step and re-adding tag_yaw accumulated
                    # (wrist kept rotating through descend/lift). Fixing it now
                    # makes every grasp step idempotent.
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            'world', 'end_effector_link', rclpy.time.Time())
                        q = tf.transform.rotation
                        _, _, ee_yaw = tft.euler_from_quaternion(
                            [q.x, q.y, q.z, q.w])
                        self.grasp_yaw_abs = (
                            ee_yaw + tag_img_yaw + GRASP_YAW_OFFSET)
                    except Exception as e:
                        self.get_logger().error(
                            f'TF yaw capture failed: {e} — preserving EE yaw.')
                        self.grasp_yaw_abs = None
                    yaw_str = (f'{math.degrees(self.grasp_yaw_abs):+.1f}'
                               if self.grasp_yaw_abs is not None else 'EE-preserve')
                    self.get_logger().info(
                        f'Converged and held for {held:.1f}s '
                        f'(tag yaw {math.degrees(tag_img_yaw):+.1f} deg, '
                        f'target wrist yaw {yaw_str} deg) — starting grasp.')
                    self.stop_twist()
                    self.servoing_done = True
                    self.set_state('grasping')
                    self.switch_back_to_joint(callback=self.grasp_sequence_start)
                    return
        else:
            self.converge_start_time = None

        twist = Twist()
        twist.linear.x  = self.calc_velocity(-err_x, KP_XY, MAX_LIN_VEL, MIN_LIN_VEL, DEAD_XY)
        twist.linear.y  = self.calc_velocity(-err_y, KP_XY, MAX_LIN_VEL, MIN_LIN_VEL, DEAD_XY)
        twist.linear.z  = self.calc_velocity(-err_z, KP_Z,  MAX_LIN_VEL, MIN_LIN_VEL, DEAD_Z)
        twist.angular.z = 0.0

        self.last_det_time = self.get_clock().now()
        self.active_twist  = twist

    def publish_twist_loop(self):
        if self.state != 'servoing':
            return
        dt = (self.get_clock().now() - self.last_det_time).nanoseconds / 1e9
        self.twist_pub.publish(Twist() if dt > 1.0 else self.active_twist)

    def stop_twist(self):
        self.active_twist = Twist()
        self.twist_pub.publish(Twist())

    # ═══════════════════════════════════════════════════════════════════════════
    # PHASE 5 — SWITCH BACK
    # ═══════════════════════════════════════════════════════════════════════════

    def switch_back_to_joint(self, callback=None):
        req = SwitchController.Request()
        req.activate_controllers   = ['joint_trajectory_controller']
        req.deactivate_controllers = ['twist_controller']
        req.strictness = 2
        future = self.switch_client.call_async(req)
        if callback:
            future.add_done_callback(
                lambda f: callback() if f.result().ok else
                self.get_logger().error('Switch-back failed — grasp aborted.'))
        else:
            future.add_done_callback(
                lambda f: self.get_logger().info(
                    'Switched back to joint trajectory controller.'
                    if f.result().ok else 'Switch-back failed!'))

    # ═══════════════════════════════════════════════════════════════════════════
    # IK HELPERS
    # ═══════════════════════════════════════════════════════════════════════════

    def _topdown_quat_preserving_yaw(self):
        """Top-down grasp orientation (tool Z pointing down), commanding the
        ABSOLUTE world-Z wrist yaw fixed once at convergence (grasp_yaw_abs).
        Re-reading EE yaw and re-adding the tag yaw per step caused cumulative
        drift (wrist kept rotating through descend/lift), so the target is fixed
        and re-applied idempotently. Falls back to preserving current EE yaw if
        no absolute yaw was captured. Returns (x,y,z,w) or None."""
        if self.grasp_yaw_abs is not None:
            yaw = self.grasp_yaw_abs
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'world', 'end_effector_link', rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f'TF Error (yaw read): {e}')
                return None
            q = tf.transform.rotation
            _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # roll=pi -> tool Z down; command the fixed yaw about world Z.
        return tft.quaternion_from_euler(math.pi, 0.0, yaw)

    def calculate_ik_delta_async(self, dx, dy, dz, callback):
        """Relative Cartesian move off the current end-effector POSITION, but
        with a clean top-down (yaw-preserving) orientation rather than the
        drifted IBVS orientation. The clean orientation makes a straight-down
        descend well-conditioned for IK (fixes NO_IK_SOLUTION / error -31)."""
        try:
            # base_link has buffer_length=0 — use 'world' (static identity to base_link)
            transform = self.tf_buffer.lookup_transform(
                'world', 'end_effector_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'TF Error: {e}')
            callback(None)
            return

        quat = self._topdown_quat_preserving_yaw()
        if quat is None:
            callback(None)
            return

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'world'
        target_pose.pose.position.x = transform.transform.translation.x + dx
        target_pose.pose.position.y = transform.transform.translation.y + dy
        target_pose.pose.position.z = transform.transform.translation.z + dz
        target_pose.pose.orientation.x = float(quat[0])
        target_pose.pose.orientation.y = float(quat[1])
        target_pose.pose.orientation.z = float(quat[2])
        target_pose.pose.orientation.w = float(quat[3])

        self._call_ik(target_pose, callback)

    def calculate_ik_absolute_async(self, x, y, z, quat, callback):
        """Absolute Cartesian goal in DELIVERY_FRAME with a fixed orientation.
        Used for delivery — independent of where the grasp happened."""
        target_pose = PoseStamped()
        target_pose.header.frame_id = DELIVERY_FRAME
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.x = float(quat[0])
        target_pose.pose.orientation.y = float(quat[1])
        target_pose.pose.orientation.z = float(quat[2])
        target_pose.pose.orientation.w = float(quat[3])

        self._call_ik(target_pose, callback)

    def _call_ik(self, target_pose, callback):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'manipulator'
        req.ik_request.pose_stamped = target_pose
        req.ik_request.avoid_collisions = True

        # Seed the solver with the CURRENT joints so it returns the nearest
        # solution branch instead of an arbitrary one — prevents elbow/wrist
        # flips (the "twisting move" toward far targets).
        seed = JointState()
        seed.name = ['joint_1','joint_2','joint_3',
                     'joint_4','joint_5','joint_6','joint_7']
        seed.position = list(self.latest_joints[:7])
        req.ik_request.robot_state.joint_state = seed

        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('IK service not available.')
            callback(None)
            return

        future = self.ik_client.call_async(req)

        def done_cb(fut):
            response = fut.result()
            if response.error_code.val == 1:
                callback(list(response.solution.joint_state.position[:7]))
            else:
                self.get_logger().error(
                    f'IK Failed with error code: {response.error_code.val}')
                callback(None)

        future.add_done_callback(done_cb)

    # ═══════════════════════════════════════════════════════════════════════════
    # PHASE 6 — GRASP SEQUENCE (relative Cartesian IK)
    # ═══════════════════════════════════════════════════════════════════════════

    def grasp_sequence_start(self):
        self.get_logger().info('Grasp sequence: settling for 1s for controller activation...')
        self.grasp_step_index = 0
        self._grasp_settle_timer = self.create_timer(1.0, self._grasp_after_settle)

    def _grasp_after_settle(self):
        self._grasp_settle_timer.cancel()
        self.execute_grasp_step()

    def execute_grasp_step(self):
        if self.grasp_step_index >= len(GRASP_STEPS):
            self.grasp_done()
            return

        dx, dy, dz, gripper_pos, duration, label = GRASP_STEPS[self.grasp_step_index]

        self.get_logger().info(
            f'Grasp step {self.grasp_step_index + 1}/{len(GRASP_STEPS)}: {label}')

        if gripper_pos is not None and (dx, dy, dz) == (0.0, 0.0, 0.0):
            self.send_gripper(gripper_pos, done_callback=None)
            self._grasp_timer = self.create_timer(
                duration, self._advance_grasp_step_timer_cb)
            return

        def on_ik_done(ik_solution):
            if not ik_solution:
                self.get_logger().error('Aborting grasp - IK could not find a path.')
                return

            target = ik_solution
            self.get_logger().info(
                f'  current joints: {[f"{j:.3f}" for j in self.latest_joints]}')
            self.get_logger().info(
                f'  target joints:  {[f"{j:.3f}" for j in target]}')

            self.send_joint_goal(target, duration_sec=duration)

            if gripper_pos is not None:
                self.send_gripper(gripper_pos, done_callback=None)

            self._grasp_timer = self.create_timer(
                duration + 0.5, self._advance_grasp_step_timer_cb)

        self.calculate_ik_delta_async(dx, dy, dz, on_ik_done)

    def _advance_grasp_step_timer_cb(self):
        try:
            self._grasp_timer.cancel()
        except Exception:
            pass
        self._advance_grasp_step()

    def _advance_grasp_step(self):
        self.grasp_step_index += 1
        self.execute_grasp_step()

    def grasp_done(self):
        self.get_logger().info('Grasp sequence complete — object lifted. Starting delivery.')
        self.set_state('delivering')
        self.delivery_sequence_start()

    # ═══════════════════════════════════════════════════════════════════════════
    # PHASE 7 — DELIVERY SEQUENCE (absolute top-down pose, any start location)
    # ═══════════════════════════════════════════════════════════════════════════

    def delivery_sequence_start(self):
        self.get_logger().info(
            f'Delivery sequence: target drop ({DROP_X:.3f}, {DROP_Y:.3f}, '
            f'{DROP_Z_TABLE:.3f}) in {DELIVERY_FRAME}, top-down.')
        self.delivery_step_index = 0
        self.execute_delivery_step()

    def execute_delivery_step(self):
        if self.delivery_step_index >= len(DELIVERY_STEPS):
            self.delivery_done()
            return

        x, y, z, gripper_pos, duration, label = DELIVERY_STEPS[self.delivery_step_index]

        self.get_logger().info(
            f'Delivery step {self.delivery_step_index + 1}/{len(DELIVERY_STEPS)}: {label}')

        # Pure gripper action (no motion): fire and wait the step duration.
        if gripper_pos is not None and self._delivery_step_is_stationary():
            if gripper_pos == 'WAIT_THEN_OPEN':
                self.get_logger().info(
                    f'Holding for {GRIPPER_RELEASE_WAIT_SEC:.0f}s before release '
                    '— user can take the tag now.')
                self._delivery_timer = self.create_timer(
                    GRIPPER_RELEASE_WAIT_SEC, self._delivery_release_cb)
            else:
                self.send_gripper(gripper_pos, done_callback=None)
                self._delivery_timer = self.create_timer(
                    duration, self._advance_delivery_step_timer_cb)
            return

        def on_ik_done(ik_solution):
            if not ik_solution:
                self.get_logger().error(
                    'Delivery aborted - IK could not reach drop pose. '
                    'Check DROP_X/Y/Z are inside the arm workspace.')
                return

            target = ik_solution
            self.get_logger().info(
                f'  target joints: {[f"{j:.3f}" for j in target]}')

            self.send_joint_goal(target, duration_sec=duration)

            if gripper_pos is not None and gripper_pos != 'WAIT_THEN_OPEN':
                self.send_gripper(gripper_pos, done_callback=None)

            self._delivery_timer = self.create_timer(
                duration + 0.5, self._advance_delivery_step_timer_cb)

        self.calculate_ik_absolute_async(x, y, z, TOPDOWN_QUAT, on_ik_done)

    def _delivery_release_cb(self):
        try:
            self._delivery_timer.cancel()
        except Exception:
            pass
        self.get_logger().info('Opening gripper — releasing tag.')
        self.send_gripper(GRIPPER_OPEN, done_callback=None)
        # Give the gripper time to physically open before retreating
        self._delivery_timer = self.create_timer(
            1.5, self._advance_delivery_step_timer_cb)

    def _delivery_step_is_stationary(self):
        """True if this delivery step targets the same absolute pose as the
        previous one (i.e. it's a release-in-place, no arm motion needed)."""
        idx = self.delivery_step_index
        if idx == 0:
            return False
        cur = DELIVERY_STEPS[idx][:3]
        prev = DELIVERY_STEPS[idx - 1][:3]
        return cur == prev

    def _advance_delivery_step_timer_cb(self):
        try:
            self._delivery_timer.cancel()
        except Exception:
            pass
        self.delivery_step_index += 1
        self.execute_delivery_step()

    def delivery_done(self):
        self.get_logger().info(
            'Delivery complete — object released. Returning to home position...')
        self.send_joint_goal(HOME_WAYPOINT, duration_sec=RETURN_HOME_DURATION_SEC)
        self._home_timer = self.create_timer(
            RETURN_HOME_DURATION_SEC + 0.5, self._return_home_done)

    def _return_home_done(self):
        try:
            self._home_timer.cancel()
        except Exception:
            pass
        self.get_logger().info('At home position. Done.')
        self.set_state('done')

    # ═══════════════════════════════════════════════════════════════════════════
    # GRIPPER
    # ═══════════════════════════════════════════════════════════════════════════

    def send_gripper(self, position, done_callback=None):
        if not self.gripper_client.wait_for_server(timeout_sec=GRIPPER_TIMEOUT):
            self.get_logger().error('Gripper action server not available!')
            if done_callback:
                done_callback()
            return
        goal = GripperCommand.Goal()
        goal.command.position   = float(position)
        goal.command.max_effort = 0.0
        future = self.gripper_client.send_goal_async(goal)

        def on_goal_response(send_future):
            handle = send_future.result()
            if not handle.accepted:
                self.get_logger().error('Gripper goal rejected.')
                if done_callback:
                    done_callback()
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda _: done_callback() if done_callback else None)

        future.add_done_callback(on_goal_response)

    # ═══════════════════════════════════════════════════════════════════════════
    # WATCHDOGS
    # ═══════════════════════════════════════════════════════════════════════════

    def trigger_restart(self):
        self.get_logger().warn(
            f'Triggering restart from waypoint {self.last_detection_waypoint}...')
        self.stop_twist()
        self.set_state('scanning')
        self.current_waypoint    = self.last_detection_waypoint
        self.servoing_done       = False
        self.init_done           = False
        self.converge_start_time = None
        self.freeze_best_mag    = None
        self.freeze_progress_time = self.get_clock().now()
        self.grasp_step_index    = 0
        self.delivery_step_index = 0
        self.grasp_yaw_abs       = None   # clear stale captured yaw on restart

        req = SwitchController.Request()
        req.activate_controllers   = ['joint_trajectory_controller']
        req.deactivate_controllers = ['twist_controller']
        req.strictness = 2
        future = self.switch_client.call_async(req)
        future.add_done_callback(self.restart_done_cb)

    def restart_done_cb(self, future):
        try:
            self.init_timer.cancel()
        except Exception:
            pass
        try:
            if self.scan_timer.is_canceled():
                self.scan_timer.reset()
        except Exception:
            self.scan_timer = self.create_timer(SCAN_INTERVAL_SEC, self.scan_step)
        self.init_timer = self.create_timer(INIT_DELAY_SEC, self.init_scan)


def main():
    rclpy.init()
    rclpy.spin(ScanAndReach())

if __name__ == '__main__':
    main()
