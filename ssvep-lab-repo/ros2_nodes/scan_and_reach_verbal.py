import os
import math
import time
import json
import datetime
import yaml
import serial
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy

# ── IK & TF2 ──
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_transformations as tft

# ─── CONFIG FILE ──────────────────────────────────────────────────────────────
# Scan profiles, delivery poses and the action registry now live in YAML so
# objects/destinations/trajectories can be added without editing this file.
# Override the path with the SCAN_CONFIG env var if needed.
CONFIG_PATH = os.environ.get(
    'SCAN_CONFIG',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scan_config.yaml'))

# ─── ARDUINO SERIAL ───────────────────────────────────────────────────────────
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 115200

# ─── EVENT LOGGING ────────────────────────────────────────────────────────────
# Every robot-side event is appended (one JSON object per line) to a .jsonl file
# under LOG_DIR, timestamped in Unix epoch seconds so it aligns with MATLAB's
# marker table (both machines assumed NTP-synced). Override dir with LOG_DIR env.
LOG_DIR = os.environ.get(
    'LOG_DIR',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs'))


class TrialLogger:
    """Append-only JSONL event log for one ROS session.

    One line per event: {experiment, trial, event, t_unix, ...extra}.
    `experiment` is the incrementing block ID forwarded from the Arduino
    (EXP:<n> on each 'g'); it stays None until the first block starts.
    `trial` counts robot runs within this process (each /ssvep_command).
    """
    def __init__(self, log_dir):
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
        self.path = os.path.join(log_dir, f'robot_events_{stamp}.jsonl')
        self.experiment = None
        self.trial = -1
        # Touch the file so its existence is obvious even before any event.
        open(self.path, 'a').close()

    def set_experiment(self, exp_id):
        self.experiment = exp_id

    def new_trial(self, **extra):
        self.trial += 1
        self.event('trial_start', **extra)

    def event(self, name, **extra):
        rec = {
            'experiment': self.experiment,
            'trial': self.trial,
            'event': name,
            't_unix': time.time(),
        }
        rec.update(extra)
        with open(self.path, 'a') as f:
            f.write(json.dumps(rec) + '\n')
        return rec

# ─── HOME ─────────────────────────────────────────────────────────────────────
HOME_WAYPOINT = [0.00, 0.26, 3.14, -2.27, 0.00, 0.96, 1.57]
RETURN_HOME_DURATION_SEC = 4.0

# ─── AUTO-END AFTER N SELECTIONS ──────────────────────────────────────────────
# A "run" is a fixed number of completed selections (each ending in a home
# return). After SELECTIONS_PER_RUN successful completions, the node sends 'e'
# to the Arduino, which emits 'x' -> experiment_end -> the recorder flushes.
# Set to 0 to disable (free-run until manual end).
SELECTIONS_PER_RUN = 4

# ─── SETTLING BLANK ───────────────────────────────────────────────────────────
# For BLANK_AFTER_FLICKER_SEC after the LEDs (re)start (each 'g' or 'r'), the
# EEG hasn't settled and the classifier is most likely to emit a fluke. Ignore
# /ssvep_command during this window so noise can't launch a run. 0 = disabled.
BLANK_AFTER_FLICKER_SEC = 0.5

# ─── SCAN TIMING ──────────────────────────────────────────────────────────────
MOVE_DURATION_SEC = 3
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
MAX_LIN_VEL = 0.12
MIN_LIN_VEL = 0.01

# ─── CONVERGENCE HOLD ────────────────────────────────────────────────────────
CONVERGE_HOLD_SEC = 1.0

# ─── VISUAL FREEZE WATCHDOG (servoing only) ──────────────────────────────────
VISUAL_FREEZE_SEC      = 2.0
VISUAL_FREEZE_MIN_PROG = 1.0

# ─── GRASP SEQUENCE (Cartesian Deltas via IK) ────────────────────────────────
GRIPPER_OPEN  = 0.0
GRIPPER_CLOSE = 0.5
GRASP_YAW_OFFSET = 0.0
ALIGN_WRIST_TO_TAG = True
FOLD_TAG_SYMMETRY = True
GRASP_STEPS = [
    (0.0,  0.0,  0.0,   GRIPPER_OPEN,  1.5, 'Open gripper'),
    (0.02, 0.0,  0.0,   None,          1.5, 'Align horizontally (-1cm X)'),
    (0.0,  0.0, 0.0,  None,           1.5, 'Align horizontally (-1cm Y)'),
    (0.0,  0.0, -0.06,  None,          2.0, 'Descend to box (2cm)'),
    (0.0,  0.0,  0.0,   GRIPPER_CLOSE, 1.5, 'Close gripper (grasp)'),
    (0.0,  0.0,  0.15,  None,          2.0, 'Lift object (15cm)'),
]
GRIPPER_TIMEOUT = 5.0

# ─── DELIVERY ────────────────────────────────────────────────────────────────
DELIVERY_FRAME = 'world'
DROP_APPROACH_HEIGHT = 0.15
GRIPPER_RELEASE_WAIT_SEC = 1.0


# ════════════════════════════════════════════════════════════════════════════
# CONFIG MODEL
# ════════════════════════════════════════════════════════════════════════════

class Action:
    """One resolved behaviour: which scan profile to sweep, which tag to grasp,
    and which delivery pose to drop at. Built from the YAML registry."""
    def __init__(self, cls_index, scan_name, scan_waypoints,
                 tag_id, deliver_name, drop_xyz, drop_quat):
        self.cls_index      = cls_index
        self.scan_name      = scan_name
        self.scan_waypoints = scan_waypoints      # list of 7-joint lists (+home)
        self.tag_id         = tag_id
        self.deliver_name   = deliver_name
        self.drop_xyz       = tuple(drop_xyz)     # (x, y, z) in world
        self.drop_quat      = tuple(drop_quat)    # (x, y, z, w) top-down

    def delivery_steps(self):
        """Build the 4-step delivery sequence for THIS action's drop pose.
        Identical shape to the old module-level DELIVERY_STEPS, but per-action.
        Step: (abs_x, abs_y, abs_z, gripper_pos, duration_sec, label)."""
        x, y, z = self.drop_xyz
        return [
            (x, y, z + DROP_APPROACH_HEIGHT, None,             4.0, 'Move above drop position'),
            (x, y, z,                        None,             2.5, 'Lower to drop height'),
            (x, y, z,                        'WAIT_THEN_OPEN', GRIPPER_RELEASE_WAIT_SEC, 'Wait then release'),
            (x, y, z + DROP_APPROACH_HEIGHT, None,             2.5, 'Retreat upward'),
        ]


def load_config(path):
    """Parse the YAML into {class_index: Action}. Each scan profile gets the
    shared HOME_WAYPOINT appended as its final waypoint, preserving the old
    behaviour where an unsuccessful sweep ends in a safe home pose."""
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)

    profiles  = cfg.get('scan_profiles', {})
    poses     = cfg.get('delivery_poses', {})
    actions   = cfg.get('actions', {})

    resolved = {}
    for cls, row in actions.items():
        cls = int(cls)
        scan_name = row['scan']
        if scan_name not in profiles:
            raise ValueError(
                f"action {cls}: scan profile '{scan_name}' not in scan_profiles")
        deliver_name = row['deliver']
        if deliver_name not in poses:
            raise ValueError(
                f"action {cls}: delivery pose '{deliver_name}' not in delivery_poses")

        waypoints = [list(map(float, wp)) for wp in profiles[scan_name]]
        waypoints.append(list(HOME_WAYPOINT))   # safe end-of-sweep pose

        pose = poses[deliver_name]
        resolved[cls] = Action(
            cls_index=cls,
            scan_name=scan_name,
            scan_waypoints=waypoints,
            tag_id=int(row['tag']),
            deliver_name=deliver_name,
            drop_xyz=pose['xyz'],
            drop_quat=pose['quat'],
        )
    return resolved


class ScanAndReach(Node):

    def __init__(self):
        super().__init__('scan_and_reach_verbal')

        # ── Load action registry ──
        self.actions = load_config(CONFIG_PATH)
        self.active  = None   # the Action currently being executed
        self.get_logger().info(
            f'Loaded {len(self.actions)} actions from {CONFIG_PATH}: '
            f'{sorted(self.actions.keys())}')

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
        self.create_subscription(
            Int32, '/ssvep_command', self.ssvep_cmd_cb, 10)
        # Debug-only: lets you fire the resume ('r') path without the robot.
        # Publish: ros2 topic pub --once /debug_resume std_msgs/msg/Empty "{}"
        self.create_subscription(
            Empty, '/debug_resume', self._debug_resume_cb, 10)

        # ── Trial-event bridge to MATLAB ──
        # MATLAB subscribes to /trial_event to stamp its continuous EEG marker
        # table. Payloads: 'experiment_start:<id>', 'cue_onset:<id>',
        # 'trial_end:<byte>' (byte = t/r/c), 'experiment_end:<id>'.
        # Publisher is transient_local so a late-joining MATLAB still catches
        # the most recent experiment_start.
        trial_qos = QoSProfile(depth=10)
        trial_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.trial_event_pub = self.create_publisher(
            String, '/trial_event', trial_qos)

        # Gripper action client
        self.gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

        # ── TF2 and IK Setup ──
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── State machine ──
        self.state            = 'waiting'
        self.target_tag_id    = None
        self.current_waypoint = 0
        self.servoing_done    = False
        self.latest_joints    = [0.0] * 7
        self.init_done        = False
        self.last_detection_waypoint = 0
        self.grasp_yaw_abs    = None

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
        self.delivery_steps      = []   # set per-run from self.active

        # ── Event logging ──
        self.trial_logger = TrialLogger(LOG_DIR)
        self.experiment_id = None       # latest block id forwarded from Arduino
        self._completed_selections = 0   # for SELECTIONS_PER_RUN auto-end
        self._flicker_start_t = None     # set on g/r; commands blanked just after
        self.get_logger().info(
            f'Event log: {self.trial_logger.path}')

        # ── Arduino serial (LED gating) ──
        # Always active. The Arduino reacts only to g/p/r bytes, so this whole
        # path can be tested without the robot: just run the node and publish
        # to /ssvep_command from another terminal.
        self._arduino = None
        try:
            self._arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=3)
            # Wait for the Arduino's "ready" line instead of a blind sleep, so
            # 'g' is never sent into a bootloader reset.
            deadline = time.time() + 5.0
            while time.time() < deadline:
                line = self._arduino.readline().decode(errors='ignore').strip()
                if 'ready' in line.lower():
                    break
            self._arduino_send(b'g')     # start LED sequence
            self.get_logger().info(
                f'Arduino connected on {ARDUINO_PORT} — LED sequence started.')
        except serial.SerialException as e:
            self.get_logger().warn(
                f'Arduino not found ({e}). LED gating disabled.')

        # Poll the Arduino for async notifications (e.g. 't' = classifier timeout).
        self._classifier_misses = 0
        if self._arduino:
            self.create_timer(0.1, self._poll_arduino)

        # ── Start sequence ──
        self.reset_controllers()
        self.init_timer = None
        self.scan_timer = None
        self.get_logger().info(
            'Scan and Reach ready — idle, waiting for /ssvep_command.')

    # ═══════════════════════════════════════════════════════════════════════════
    # ARDUINO HELPERS
    # ═══════════════════════════════════════════════════════════════════════════

    def _arduino_send(self, byte: bytes):
        """Send a single control byte to the Arduino. Warns if not connected."""
        if byte in (b'g', b'r'):
            self._flicker_start_t = self.get_clock().now()
        if self._arduino and self._arduino.is_open:
            try:
                self._arduino.write(byte)
                self.get_logger().info(f'Arduino <- {byte}')
            except serial.SerialException as e:
                self.get_logger().warn(f'Arduino write failed: {e}')
        else:
            self.get_logger().warn(f'Arduino not connected; dropped {byte}')

    def _publish_trial_event(self, payload: str):
        """Publish one marker string to MATLAB on /trial_event."""
        msg = String()
        msg.data = payload
        self.trial_event_pub.publish(msg)
        self.get_logger().info(f'/trial_event -> {payload}')

    def _debug_resume_cb(self, msg):
        """Test hook: fire the same resume byte the robot sends on home return."""
        self.get_logger().info('/debug_resume received — sending r to Arduino.')
        self._arduino_send(b'r')

    def _poll_arduino(self):
        """Read async notifications from the Arduino and bridge them to MATLAB.

        Recognised lines:
          EXP:<n>  block started ('g' handled) -> experiment id <n>
          CUE:<c>  cue onset for class <c>      -> cue_onset (fallback path)
          t        classifier timeout (10s miss)-> trial_end:t
          x        end-of-block sentinel        -> experiment_end
        Note: r/c are commands WE send to the Arduino, so their trial-end
        markers are emitted where we send them (return-home / scan-fail),
        not here. Only 't' comes back unsolicited from the Arduino.
        """
        if not (self._arduino and self._arduino.is_open):
            return
        try:
            while self._arduino.in_waiting:
                line = self._arduino.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('EXP:'):
                    exp = line[4:].strip()
                    self.experiment_id = exp
                    self.trial_logger.set_experiment(exp)
                    self.trial_logger.event('experiment_start', exp=exp)
                    self._publish_trial_event(f'experiment_start:{exp}')

                elif line == 'HOME':
                    # Verbal-cue paradigm: robot returned home; the Arduino just
                    # fired the pin-9 TTL. Mark it on the EEG timeline.
                    self.trial_logger.event('home_reached')
                    self._publish_trial_event('home_reached')

                elif line == 'x':
                    self.get_logger().info('Arduino end-of-block sentinel (x).')
                    self.trial_logger.event('experiment_end',
                                            exp=self.experiment_id)
                    self._publish_trial_event(
                        f'experiment_end:{self.experiment_id}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Arduino read failed: {e}')

    # ═══════════════════════════════════════════════════════════════════════════
    # SSVEP COMMAND → START A RUN
    # ═══════════════════════════════════════════════════════════════════════════

    def ssvep_cmd_cb(self, msg):
        cls = int(msg.data)
        # Reject commands during the post-flicker settling window.
        if BLANK_AFTER_FLICKER_SEC > 0 and self._flicker_start_t is not None:
            dt = (self.get_clock().now() - self._flicker_start_t).nanoseconds / 1e9
            if dt < BLANK_AFTER_FLICKER_SEC:
                self.get_logger().info(
                    f'Command ignored — within {BLANK_AFTER_FLICKER_SEC}s '
                    f'settling blank ({dt:.2f}s).')
                return

        # Once a run is committed, ignore ALL incoming commands so the user can
        # stop looking at the LED the moment a run begins.
        if self.state not in ('waiting', 'done'):
            self.get_logger().warn(
                f'SSVEP command {cls} ignored — run in progress (state={self.state}).')
            return

        if cls not in self.actions:
            self.get_logger().warn(
                f'SSVEP command {cls} has no action mapping — ignored.')
            return

        # Resolve and latch the active action for this whole run.
        self.active = self.actions[cls]
        self.target_tag_id = self.active.tag_id
        self.get_logger().info(
            f'SSVEP {cls} -> action [scan={self.active.scan_name}, '
            f'tag={self.active.tag_id}, deliver={self.active.deliver_name}]. '
            'Starting scan.')
        # A classifier decision arrived -> a new robot trial begins.
        self.trial_logger.new_trial(cls=cls, tag=self.active.tag_id)
        self.trial_logger.event('command_received', cls=cls)
        self._arduino_send(b'p')   # pause LED — robot about to move
        self.start_scan()

    def _scan_waypoints(self):
        """Waypoints for the currently active action (falls back safely)."""
        if self.active is not None:
            return self.active.scan_waypoints
        return [list(HOME_WAYPOINT)]

    def start_scan(self):
        self.trial_logger.event('robot_scan_start')
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
        waypoints = self._scan_waypoints()
        start_wp = self.current_waypoint
        self.get_logger().info(f'Init: sending waypoint {start_wp}')
        self.send_joint_goal(waypoints[start_wp])
        self.current_waypoint = start_wp + 1
        self.init_done = True
        self.init_timer.cancel()

    def scan_step(self):
        if not self.init_done:
            return
        if self.state != 'scanning':
            return
        waypoints = self._scan_waypoints()
        if self.current_waypoint >= len(waypoints):
            self.get_logger().warn(
                'Full scan complete — tag not found. Advancing to next trial.')
            self.scan_timer.cancel()
            self.set_state('waiting')
            # Advance to the NEXT colour on scan failure (was: 'c' = repeat same
            # colour). The failure mode is still recorded distinctly in the log
            # as outcome='scan_fail' vs 'classifier_timeout', so the two remain
            # separable for reporting even though both now advance the LED.
            self.trial_logger.event('trial_end', reason='r',
                                    outcome='scan_fail')
            self._publish_trial_event('trial_end:r')
            self._arduino_send(b'r')   # resume LED and ADVANCE to next colour
            return
        self.get_logger().info(f'Scan waypoint {self.current_waypoint}')
        self.send_joint_goal(waypoints[self.current_waypoint])
        self.current_waypoint += 1

    def _shortest_path_targets(self, current, target):
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
        if not msg.position:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.latest_joints = [
                float(name_to_pos[f'joint_{i}']) for i in range(1, 8)]
        except KeyError:
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
                self.trial_logger.event(
                    'tag_detected', tag_id=int(valid[0].id),
                    waypoint=self.last_detection_waypoint)
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
        self.trial_logger.event('twist_controller_start')
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
        c0 = det.corners[0]
        c1 = det.corners[1]
        yaw = -math.atan2(c1.y - c0.y, c1.x - c0.x)
        if FOLD_TAG_SYMMETRY:
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
        return tft.quaternion_from_euler(math.pi, 0.0, yaw)

    def calculate_ik_delta_async(self, dx, dy, dz, callback):
        try:
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

        # The close-gripper step is the moment of grasp; log it once.
        if gripper_pos == GRIPPER_CLOSE:
            self.trial_logger.event('grasp')

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
    # PHASE 7 — DELIVERY SEQUENCE (per-action absolute top-down pose)
    # ═══════════════════════════════════════════════════════════════════════════

    def delivery_sequence_start(self):
        # Build the step list for THIS action's drop pose.
        self.delivery_steps = self.active.delivery_steps()
        self.delivery_quat  = self.active.drop_quat
        dx, dy, dz = self.active.drop_xyz
        self.get_logger().info(
            f'Delivery sequence ({self.active.deliver_name}): drop '
            f'({dx:.3f}, {dy:.3f}, {dz:.3f}) in {DELIVERY_FRAME}, top-down.')
        self.delivery_step_index = 0
        self.execute_delivery_step()

    def execute_delivery_step(self):
        if self.delivery_step_index >= len(self.delivery_steps):
            self.delivery_done()
            return

        x, y, z, gripper_pos, duration, label = self.delivery_steps[self.delivery_step_index]

        self.get_logger().info(
            f'Delivery step {self.delivery_step_index + 1}/{len(self.delivery_steps)}: {label}')

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
                    'Check drop xyz is inside the arm workspace.')
                return

            target = ik_solution
            self.get_logger().info(
                f'  target joints: {[f"{j:.3f}" for j in target]}')

            self.send_joint_goal(target, duration_sec=duration)

            if gripper_pos is not None and gripper_pos != 'WAIT_THEN_OPEN':
                self.send_gripper(gripper_pos, done_callback=None)

            self._delivery_timer = self.create_timer(
                duration + 0.5, self._advance_delivery_step_timer_cb)

        self.calculate_ik_absolute_async(x, y, z, self.delivery_quat, on_ik_done)

    def _delivery_release_cb(self):
        try:
            self._delivery_timer.cancel()
        except Exception:
            pass
        self.get_logger().info('Opening gripper — releasing tag.')
        self.send_gripper(GRIPPER_OPEN, done_callback=None)
        self._delivery_timer = self.create_timer(
            1.5, self._advance_delivery_step_timer_cb)

    def _delivery_step_is_stationary(self):
        idx = self.delivery_step_index
        if idx == 0:
            return False
        cur = self.delivery_steps[idx][:3]
        prev = self.delivery_steps[idx - 1][:3]
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
        self.trial_logger.event('delivery_complete')
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
        # Successful run: the 'r' we send advances the Arduino to the next
        # colour, so this is also the trial boundary MATLAB should mark.
        self.trial_logger.event('trial_end', reason='r', outcome='success')
        self._publish_trial_event('trial_end:r')
        self._arduino_send(b'r')   # resume LED — robot idle

        # Count this completed selection; auto-end the block after N of them.
        self._completed_selections += 1
        if SELECTIONS_PER_RUN > 0 and \
                self._completed_selections >= SELECTIONS_PER_RUN:
            self.get_logger().info(
                f'{self._completed_selections} selections complete — '
                f'ending block (sending e to Arduino).')
            self._arduino_send(b'e')   # -> Arduino emits x -> experiment_end -> flush
            self._completed_selections = 0

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
        self.grasp_yaw_abs       = None

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
