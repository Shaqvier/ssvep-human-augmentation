"""
wrist_jog_screwdriver.py
─────────────────────────────────────────────────────────────────────────────
SSVEP screwdriver experiment — ROS 2 node.

Listens to:
  /ssvep_scores   (std_msgs/Float32MultiArray) — EVERY classifier window,
                    incl. rests. Drives continuous-jog hysteresis.
  Arduino serial  — 's' opens the active window, 'x' closes it, 'e' ends block

Continuous jog (Schmitt trigger on the score stream):
  START  after 4 consecutive same-class windows (0=unscrew, 1=screw)
  keep stepping joint_7 every same-class window while active
  FLIP   direction if the other class wins 4 in a row (no stop needed)
  STOP   after 4 consecutive rest windows, or on window close

  Class 0  →  Red  16 Hz  →  joint_7 −  (unscrew / left)
  Class 1  →  Blue 19 Hz  →  joint_7 +  (screw   / right)

Requires the classifier (read_ssvep_ros2_fbcca_banded.m) publishing /ssvep_scores
and, for logging, the recorder (record_ssvep_continuous.m) subscribing to
/trial_event. Gated by the Arduino window so classifier noise during cue/blank
never moves the robot.

Usage:
  python3 wrist_jog_screwdriver.py [--port /dev/ttyUSB0] [--baud 115200]
  python3 wrist_jog_screwdriver.py --no-gate   # testing without Arduino
"""

import argparse
import math
import threading
import serial          # pyserial

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from controller_manager_msgs.srv import SwitchController

# ─── HOME ─────────────────────────────────────────────────────────────────────
HOME_WAYPOINT     = [1.5, 0.57, 3.14, -2.27, -0.01, 0.88, 1.57]
HOME_DURATION_SEC = 4.0
# ─── JOG PARAMETERS ────────────────────────────────────────────────────────────
JOG_STEP_RAD     = math.radians(2.5)   # per accepted detection while jogging
JOG_DURATION_SEC = 0.25                 # trajectory duration per step

# ─── CONTINUOUS-JOG HYSTERESIS ─────────────────────────────────────────────────
# Movement is continuous: once started, the wrist keeps stepping on every
# same-class window until it stops. A Schmitt-trigger on the /ssvep_scores
# stream prevents chatter:
#   START  after VOTES_START consecutive windows of the SAME class (0 or 1)
#   STOP   after VOTES_STOP  consecutive REST windows
#   FLIP   direction if the OTHER class wins VOTES_START in a row while jogging
# Each window is ~decision_rate apart (~0.25 s), so stepping every window with a
# short trajectory duration blends into continuous motion.
VOTES_START = 4     # same-class windows to begin / to flip direction
VOTES_STOP  = 4     # consecutive rest windows to halt

# Class → direction
#   Class 0  (Red,  16 Hz)  →  − (unscrew / left)
#   Class 1  (Blue, 19 Hz)  →  + (screw   / right)
CLASS_TO_DIRECTION = {
    0: -1,
    1: +1,
}

# joint_7 soft limits
JOINT7_MIN_RAD = -2*math.pi
JOINT7_MAX_RAD =  2*math.pi

# ─── SERIAL ───────────────────────────────────────────────────────────────────
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200


class WristJogScrewdriver(Node):

    def __init__(self, serial_port: str, baud: int, no_gate: bool = False):
        super().__init__('wrist_jog_screwdriver')

        # ── ROS publishers / subscribers / service clients ──
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory', 10)
        self.switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        # Continuous jog reads EVERY decision window (incl. rests) from the
        # classifier's /ssvep_scores stream, not the debounced /ssvep_command.
        # Payload: [s16 s19 s23 s29 winner_hz max_val margin passed published].
        self.create_subscription(
            Float32MultiArray, '/ssvep_scores', self.scores_cb, 10)

        # ── Trial-event bridge to the recorder ──
        # Same topic the reach experiment uses: the recorder subscribes to
        # /trial_event and stamps each marker onto the continuous EEG timeline.
        # transient_local so a late-started recorder still catches experiment_start.
        te_qos = QoSProfile(depth=10)
        te_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.trial_event_pub = self.create_publisher(String, '/trial_event', te_qos)
        self._recording_open = False      # first 's' opens the recording
        self._experiment_id = 0           # increments each session (each first 's')
        self._home_after_block = False    # set by serial thread on 'e', acted on
                                          # by the executor via _home_poll timer

        # ── Robot state ──
        self.latest_joints = list(HOME_WAYPOINT)

        # ── Continuous-jog hysteresis state ──
        self.is_jogging    = False   # currently moving?
        self.jog_direction = 0       # +1 screw / -1 unscrew / 0 idle
        self.run_class     = None    # class of the current consecutive run
        self.run_count     = 0       # length of that run
        self.rest_count    = 0       # consecutive rest windows

        # ── Window gate ──
        # no_gate=True: always open (testing without Arduino)
        # no_gate=False: controlled by serial 's'/'x' from Arduino
        self.window_open = no_gate
        if no_gate:
            self.get_logger().warn(
                '--no-gate active: window always open. '
                'Remove this flag for the real experiment.')

        # ── Serial reader thread (skipped when no_gate) ──
        self._serial = None
        self._serial_thread = None
        if not no_gate:
            self._open_serial(serial_port, baud)

        # ── Startup ──
        self.reset_controllers()
        self.home_timer = self.create_timer(1.0, self.go_home_once)

        # Poll the home-after-block flag in the executor thread (the serial
        # reader thread must not call send_joint_goal directly — it races with
        # joint_cb and the trajectory publisher, both executor-owned).
        self._home_poll = self.create_timer(0.2, self._check_home_after_block)

        self.get_logger().info(
            'Wrist jog screwdriver ready.\n'
            '  Class 0 (Red,  16 Hz) = unscrew (joint_7 -1 deg/detection)\n'
            '  Class 1 (Blue, 19 Hz) = screw   (joint_7 +1 deg/detection)')

    # ═══════════════════════════════════════════════════════════════════════════
    # SERIAL — gate thread
    # ═══════════════════════════════════════════════════════════════════════════

    def _publish_trial_event(self, payload: str):
        msg = String()
        msg.data = payload
        self.trial_event_pub.publish(msg)

    def _open_serial(self, port: str, baud: int):
        try:
            self._serial = serial.Serial(port, baud, timeout=1)
            self._serial_thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self._serial_thread.start()
            self.get_logger().info(f'Serial opened on {port} @ {baud}.')
        except Exception as e:
            self.get_logger().error(
                f'Could not open serial port {port}: {e}\n'
                '  Tip: run with --no-gate to bypass serial for testing.')

    def _serial_reader(self):
        """Background thread: reads 's' / 'x' / 'e' bytes from the Arduino.

        s -> window opens: first 's' of a session also opens the recording
             (experiment_start); every 's' is a cue_onset.
        x -> window closes: trial_end. Also log the net wrist position reached
             during this window.
        e -> block complete: experiment_end (recorder flushes).
        """
        while rclpy.ok():
            try:
                byte = self._serial.read(1)
                if not byte:
                    continue
                ch = byte.decode('ascii', errors='ignore')
                if ch == 's':
                    self.window_open = True
                    if not self._recording_open:
                        self._recording_open = True
                        self._experiment_id += 1
                        self._publish_trial_event(
                            f'experiment_start:{self._experiment_id}')
                        self.get_logger().info(
                            f'  Recording OPEN (experiment {self._experiment_id}).')
                    # reset hysteresis for the new window
                    self.run_class = None
                    self.run_count = 0
                    self.rest_count = 0
                    self._window_start_j7 = self.latest_joints[6]
                    self._publish_trial_event('cue_onset:screwdriver')
                    self.get_logger().info('  Stimulus window OPEN — robot active.')
                elif ch == 'x':
                    self.window_open = False
                    if self.is_jogging:
                        self._stop_jog('window closed')
                    # net wrist movement over this window (degrees)
                    net_deg = math.degrees(
                        self.latest_joints[6] - getattr(self, '_window_start_j7',
                                                        self.latest_joints[6]))
                    self._publish_trial_event(f'wrist_net:{net_deg:.1f}')
                    self._publish_trial_event('trial_end:x')
                    self.get_logger().info(
                        f'  Stimulus window CLOSED — net wrist {net_deg:+.1f} deg.')
                elif ch == 'e':
                    if self._recording_open:
                        self._recording_open = False
                        self._publish_trial_event(
                            f'experiment_end:{self._experiment_id}')
                        self.get_logger().info('  Block complete — recording FLUSH.')
                    # Return to home after the full 10-trial run. Scheduled on a
                    # one-shot timer so it runs in the ROS executor thread, not
                    # this serial thread (send_joint_goal publishes a message and
                    # reads latest_joints, which the executor owns).
                    self.window_open = False
                    self._home_after_block = True
            except Exception:
                pass

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

    def go_home_once(self):
        self.home_timer.cancel()
        self.get_logger().info('Moving to home position...')
        self.send_joint_goal(HOME_WAYPOINT, duration_sec=HOME_DURATION_SEC)

    def _check_home_after_block(self):
        """Executor-thread poll: home once after the block-complete 'e' byte."""
        if self._home_after_block:
            self._home_after_block = False
            self.get_logger().info(
                'Block finished — returning to home position.')
            self.send_joint_goal(HOME_WAYPOINT, duration_sec=HOME_DURATION_SEC)

    # ═══════════════════════════════════════════════════════════════════════════
    # SSVEP → JOG
    # ═══════════════════════════════════════════════════════════════════════════

    def scores_cb(self, msg):
        """Continuous-jog hysteresis driven by every classifier window.

        Gated by the Arduino window (s/x): outside it, ignore everything and
        make sure we're stopped.
        """
        if not self.window_open:
            if self.is_jogging:
                self._stop_jog('window closed')
            return

        d = list(msg.data)
        if len(d) < 9:
            return
        winner_hz = int(round(d[4]))
        passed    = int(round(d[7]))

        # Map winner frequency -> screwdriver class. Rest = no/low detection.
        # 16 Hz -> class 0 (unscrew), 19 Hz -> class 1 (screw). Anything else,
        # or a window that didn't pass threshold, counts as REST.
        if passed and winner_hz == 23:
            cls = 0
        elif passed and winner_hz == 29:
            cls = 1
        else:
            cls = None   # rest

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
            # START once the run reaches the threshold.
            if self.run_count >= VOTES_START:
                self._start_jog(direction, cls)
                self._step(direction, cls)
        else:
            if direction == self.jog_direction:
                # Same direction — keep stepping every window (continuous).
                self._step(direction, cls)
            elif self.run_count >= VOTES_START:
                # Opposite class held long enough — FLIP without stopping.
                self.get_logger().info(
                    f'Direction flip -> class {cls}.')
                self._start_jog(direction, cls)   # re-announces direction
                self._step(direction, cls)
            # else: opposite class but not yet 4 in a row — keep current dir,
            # do not step this window (ambiguous), wait for the run to build.

    def _start_jog(self, direction, cls):
        self.is_jogging    = True
        self.jog_direction = direction
        action = 'SCREW' if direction > 0 else 'UNSCREW'
        self.get_logger().info(f'JOG START — {action} (class {cls}).')
        self._publish_trial_event(f'jog_start:{cls},{direction:+d}')

    def _stop_jog(self, reason):
        self.is_jogging    = False
        self.jog_direction = 0
        self.run_class     = None
        self.run_count     = 0
        self.get_logger().info(f'JOG STOP ({reason}).')
        self._publish_trial_event(f'jog_stop:{reason}')

    def _step(self, direction, cls):
        """One incremental joint-7 move. Stepping every window blends into
        continuous motion."""
        target = list(self.latest_joints[:7])
        new_j7 = target[6] + direction * JOG_STEP_RAD
        new_j7 = max(JOINT7_MIN_RAD, min(JOINT7_MAX_RAD, new_j7))
        target[6] = new_j7

        # log the step onto the EEG timeline
        self._publish_trial_event(
            f'wrist_jog:{cls},{direction:+d},{math.degrees(new_j7):.1f}')

        self.send_joint_goal(target, duration_sec=JOG_DURATION_SEC)

    # ═══════════════════════════════════════════════════════════════════════════
    # JOINT TRAJECTORY
    # ═══════════════════════════════════════════════════════════════════════════

    def send_joint_goal(self, positions, duration_sec=HOME_DURATION_SEC):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3',
                           'joint_4', 'joint_5', 'joint_6', 'joint_7']

        start  = [float(v) for v in self.latest_joints[:7]]
        target = [float(v) for v in positions]

        pt0 = JointTrajectoryPoint()
        pt0.positions       = start
        pt0.time_from_start = Duration(sec=0, nanosec=0)

        pt1 = JointTrajectoryPoint()
        pt1.positions = target
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
    parser = argparse.ArgumentParser(description='SSVEP screwdriver wrist jog node')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help='Arduino serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--no-gate', action='store_true',
                        help='Disable serial window gate — testing without Arduino')
    args, _ = parser.parse_known_args()

    rclpy.init()
    rclpy.spin(WristJogScrewdriver(args.port, args.baud, no_gate=args.no_gate))


if __name__ == '__main__':
    main()
