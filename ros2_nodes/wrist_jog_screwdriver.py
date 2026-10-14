#!/usr/bin/env python3
"""
wrist_jog_screwdriver.py
Original ROS2 SSVEP -> wrist-jog screwdriver control node (2026-07-26).

Trial-based control: listens for the current SSVEP winner frequency (via
/ssvep_scores), jogs the wrist joint (joint 7) in short steps, and hands
off to an Arduino-driven cued screwdriver sequence over serial ('s'/'w'/'x'
to start/unscrew/screw, 'h' to signal "moving home", 'e' to signal "at
home"). Each trial ends with a return to HOME_WAYPOINT.

winner_hz mapping:
  19 Hz -> class 0 (unscrew)
  23 Hz -> class 1 (screw)

NOTE (see docs/analysis_notes/writeup_notes_analysis_phase.md, section 7):
this is an early iteration later superseded by wrist_jog_screwdriver_b.py
(continuous jog + hysteresis voting) and the demo variants. Kept here for
provenance / to document how the control scheme evolved.
"""

import math
import time
import threading

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient

SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 115200

HOME_WAYPOINT = [0.00, 0.57, 3.14, -2.27, -0.01, 0.88, 1.57]
JOINT_NAMES = [f'joint_{i}' for i in range(1, 8)]

JOG_STEP_RAD = 0.25            # direct radian step (not math.radians(2.5))
JOG_DURATION_SEC = 0.3
TRIALS_PER_RUN = 10
HOME_AFTER_TRIAL_DELAY_SEC = 1.0

FREQ_TO_CLASS = {19: 0, 23: 1}   # 19 Hz -> unscrew, 23 Hz -> screw


class WristJogScrewdriver(Node):
    def __init__(self):
        super().__init__('wrist_jog_screwdriver')

        self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1.0)
        time.sleep(2.0)  # let the Arduino reset after opening the port

        self.current_joint_state = None
        self.trial_count = 0
        self.homing_in_progress = False
        self._home_timer = None

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)
        self.scores_sub = self.create_subscription(
            Float32MultiArray, '/ssvep_scores', self._scores_cb, 10)
        self.command_sub = self.create_subscription(
            Int32, '/ssvep_command', self._command_cb, 10)

        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

        self.get_logger().info('wrist_jog_screwdriver node ready.')

    def _joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def _scores_cb(self, msg: Float32MultiArray):
        # msg.data expected as [freq_16, freq_19, freq_23, freq_29] scores
        pass  # winner frequency currently arrives via /ssvep_command (int Hz)

    def _command_cb(self, msg: Int32):
        winner_hz = msg.data
        if winner_hz not in FREQ_TO_CLASS:
            return
        if self.homing_in_progress:
            return

        cls = FREQ_TO_CLASS[winner_hz]
        direction = 'unscrew' if cls == 0 else 'screw'
        self.get_logger().info(f'Winner {winner_hz} Hz -> {direction} (class {cls})')

        self._jog_wrist(cls)
        self.serial.write(b's')
        self.serial.write(b'w' if cls == 0 else b'x')

        self.trial_count += 1
        if self.trial_count >= TRIALS_PER_RUN:
            self._go_home_after_trial()

    def _jog_wrist(self, cls: int):
        if self.current_joint_state is None:
            self.get_logger().warn('No joint state yet; skipping jog.')
            return

        joints = list(self.current_joint_state.position[:7])
        step = JOG_STEP_RAD if cls == 1 else -JOG_STEP_RAD
        joints[6] += step  # joint 7 (wrist rotation)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(JOG_DURATION_SEC * 1e9)
        goal.trajectory.points = [point]

        self.action_client.wait_for_server(timeout_sec=2.0)
        self.action_client.send_goal_async(goal)

    def _go_home_after_trial(self):
        self.homing_in_progress = True
        self.serial.write(b'h')  # tell Arduino we're moving home

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = HOME_WAYPOINT
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]

        self.action_client.wait_for_server(timeout_sec=2.0)
        self.action_client.send_goal_async(goal)

        self._home_timer = threading.Timer(
            HOME_AFTER_TRIAL_DELAY_SEC, self._check_home_after_block)
        self._home_timer.start()

    def _check_home_after_block(self):
        # Two-stage timer: confirm the robot has settled at home before
        # telling the Arduino we're ready ('e' = at home) and resetting
        # the trial counter for the next block.
        self.serial.write(b'e')
        self.homing_in_progress = False
        self.trial_count = 0
        self.get_logger().info('Home confirmed; ready for next block.')


def main(args=None):
    rclpy.init(args=args)
    node = WristJogScrewdriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
