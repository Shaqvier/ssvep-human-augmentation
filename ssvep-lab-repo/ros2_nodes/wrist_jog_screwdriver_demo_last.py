"""
wrist_jog_screwdriver_demo.py
─────────────────────────────────────────────────────────────────────────────
SSVEP screwdriver — DEMO node.

Stripped-down version of wrist_jog_screwdriver.py for live demos:
  • no trial structure, no /trial_event markers, no recorder handshake
  • no Arduino serial gate — the classifier stream is always live
  • no homing — the robot stays wherever you left it

Listens to:
  /ssvep_scores  (std_msgs/Float32MultiArray) — every classifier window,
                 including rests. Drives continuous-jog hysteresis.

Continuous jog (Schmitt trigger on the score stream):
  START  after VOTES_START consecutive same-class windows
  keep stepping joint_7 every same-class window while active
  FLIP   direction if the other class wins VOTES_START in a row
  STOP   after VOTES_STOP consecutive rest windows

  Class 0  →  joint_7 −  (unscrew / CCW)
  Class 1  →  joint_7 +  (screw   / CW)

Usage:
  python3 wrist_jog_screwdriver_demo.py
"""

import math

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from controller_manager_msgs.srv import SwitchController

# ─── JOG PARAMETERS ────────────────────────────────────────────────────────────
JOG_STEP_RAD     = math.radians(10)   # per accepted window while jogging
JOG_DURATION_SEC = 0.15                # trajectory duration per step

# ─── CONTINUOUS-JOG HYSTERESIS ─────────────────────────────────────────────────
VOTES_START = 4     # same-class windows to begin / to flip direction
VOTES_STOP  = 4     # consecutive rest windows to halt

# ─── STIMULUS FREQUENCY → CLASS ────────────────────────────────────────────────
# Whatever the LED groups in front of the participant are actually running.
FREQ_TO_CLASS = {
    16: 0,   # unscrew / CCW
    19: 1,   # screw   / CW
}

# Class → joint_7 direction
CLASS_TO_DIRECTION = {
    0: -1,
    1: +1,
}

# joint_7 soft limits
JOINT7_MIN_RAD = -2 * math.pi
JOINT7_MAX_RAD = +2 * math.pi


class WristJogDemo(Node):

    def __init__(self):
        super().__init__('wrist_jog_screwdriver_demo')

        # ── ROS publishers / subscribers / service clients ──
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory', 10)
        self.switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        # Payload: [s16 s19 s23 s29 winner_hz max_val margin passed published].
        self.create_subscription(
            Float32MultiArray, '/ssvep_scores', self.scores_cb, 10)

        # ── Robot state ──
        # latest_joints: measured, from /joint_states. Used ONLY to latch a
        # starting pose — never as the source of a target, otherwise gravity
        # sag on joint_2 gets fed back into the setpoint and ratchets downward.
        # cmd_joints: the latched commanded pose. Only joint_7 is ever mutated.
        self.latest_joints = None
        self.cmd_joints    = None

        # ── Continuous-jog hysteresis state ──
        self.is_jogging    = False
        self.jog_direction = 0
        self.run_class     = None
        self.run_count     = 0
        self.rest_count    = 0

        # ── Startup ──
        # Controller switch only (no homing) so trajectory goals are accepted.
        self.reset_controllers()

        # Diagnostic: report how far each held joint has sagged from its latched
        # setpoint. If this stays flat, the sag is bounded tracking error; if it
        # grows, something is still feeding measurements back into the target.
        self.create_timer(5.0, self._report_drift)

        self.get_logger().info(
            'Wrist jog DEMO ready — waiting for /ssvep_scores.\n'
            f'  {[f for f, c in FREQ_TO_CLASS.items() if c == 0][0]} Hz = '
            'unscrew (joint_7 -)\n'
            f'  {[f for f, c in FREQ_TO_CLASS.items() if c == 1][0]} Hz = '
            'screw   (joint_7 +)')

    # ═══════════════════════════════════════════════════════════════════════════
    # STARTUP
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
    # SSVEP → JOG
    # ═══════════════════════════════════════════════════════════════════════════

    def scores_cb(self, msg):
        """Continuous-jog hysteresis driven by every classifier window."""
        if self.latest_joints is None:
            return   # no joint state yet — nothing safe to step from

        d = list(msg.data)
        if len(d) < 9:
            return
        winner_hz = int(round(d[4]))
        passed    = int(round(d[7]))

        cls = FREQ_TO_CLASS.get(winner_hz) if passed else None

        if cls is None:
            # ── REST window ──
            self.run_class = None
            self.run_count = 0
            self.rest_count += 1
            if self.is_jogging and self.rest_count >= VOTES_STOP:
                self._stop_jog('rest')
            return

        # ── A class won this window ──
        self.rest_count = 0
        if cls == self.run_class:
            self.run_count += 1
        else:
            self.run_class = cls
            self.run_count = 1

        direction = CLASS_TO_DIRECTION[cls]

        if not self.is_jogging:
            if self.run_count >= VOTES_START:
                self._start_jog(direction, cls)
                self._step(direction, cls)
        else:
            if direction == self.jog_direction:
                # Same direction — keep stepping every window (continuous).
                self._step(direction, cls)
            elif self.run_count >= VOTES_START:
                # Opposite class held long enough — FLIP without stopping.
                self.get_logger().info(f'Direction flip -> class {cls}.')
                self._start_jog(direction, cls)
                self._step(direction, cls)
            # else: ambiguous window — hold current direction, don't step.

    def _start_jog(self, direction, cls):
        # Latch joints 1-6 ONCE, on the first jog of the session. Re-latching on
        # every JOG START would capture whatever sag accrued during the rest
        # period and bake it in, reintroducing the drift more slowly.
        if self.cmd_joints is None:
            self.cmd_joints = list(self.latest_joints[:7])
            self.get_logger().info(
                'Latched hold pose (joints 1-6): '
                + ', '.join(f'{math.degrees(v):.1f}' for v in self.cmd_joints[:6]))
        self.is_jogging    = True
        self.jog_direction = direction
        action = 'SCREW' if direction > 0 else 'UNSCREW'
        self.get_logger().info(f'JOG START — {action} (class {cls}).')

    def _report_drift(self):
        if self.cmd_joints is None or self.latest_joints is None:
            return
        dev = [math.degrees(self.latest_joints[i] - self.cmd_joints[i])
               for i in range(6)]
        worst = max(range(6), key=lambda i: abs(dev[i]))
        self.get_logger().info(
            f'Hold deviation: joint_{worst + 1} {dev[worst]:+.2f} deg '
            '(' + ', '.join(f'{d:+.2f}' for d in dev) + ')')

    def _stop_jog(self, reason):
        self.is_jogging    = False
        self.jog_direction = 0
        self.run_class     = None
        self.run_count     = 0
        self.get_logger().info(f'JOG STOP ({reason}).')

    def _step(self, direction, cls):
        """One incremental joint_7 move. Stepping every window blends into
        continuous motion."""
        if self.cmd_joints is None:
            return
        # Joints 1-6: latched commanded values, never re-read from /joint_states,
        # so gravity sag can't feed back into the setpoint.
        target = list(self.cmd_joints[:6])

        # Joint 7: stepped from the MEASURED value, exactly as the original
        # script did. This is self-limiting — the command can never run more
        # than one step ahead of the arm, so nothing integrates.
        new_j7 = self.latest_joints[6] + direction * JOG_STEP_RAD
        new_j7 = max(JOINT7_MIN_RAD, min(JOINT7_MAX_RAD, new_j7))
        target.append(new_j7)

        self.cmd_joints = target
        self.send_joint_goal(target, duration_sec=JOG_DURATION_SEC)

    # ═══════════════════════════════════════════════════════════════════════════
    # JOINT TRAJECTORY
    # ═══════════════════════════════════════════════════════════════════════════

    def send_joint_goal(self, positions, duration_sec=JOG_DURATION_SEC):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3',
                           'joint_4', 'joint_5', 'joint_6', 'joint_7']

        # pt0 is the MEASURED state — it must match where the arm actually is,
        # or the controller sees a step discontinuity at t=0 and faults. This is
        # safe: the sag ratchet came from the TARGET being derived from
        # measurements, not from the start point.
        pt0 = JointTrajectoryPoint()
        pt0.positions       = [float(v) for v in self.latest_joints[:7]]
        pt0.time_from_start = Duration(sec=0, nanosec=0)

        pt1 = JointTrajectoryPoint()
        pt1.positions = [float(v) for v in positions]
        sec     = int(duration_sec)
        nanosec = int(round((duration_sec - sec) * 1e9))
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


# ─── ENTRY POINT ──────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    rclpy.spin(WristJogDemo())


if __name__ == '__main__':
    main()
