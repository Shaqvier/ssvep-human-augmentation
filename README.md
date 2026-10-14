# SSVEP-Controlled Supernumerary Robotic Arm

EEG-driven SSVEP brain-computer interface controlling a KINOVA Gen3 supernumerary robotic arm (MSc dissertation, Human and Biological Robotics, Imperial College London). A participant looks at a flickering LED target; the EEG response is classified and sent to the robot over ROS 2 as a `/ssvep_command` class index.

This is a reorganized copy of the working dev repo (`ssvep-workspace-augmentation`) for future lab members. **Before running anything, read "Which file do I actually use?" below** — several scripts exist in multiple iterations and picking the wrong one will give you a mismatched protocol.

## 1. One-time setup

Full details (gripper build-from-source, firmware version check, calibration) are in `docs/ros2_commands.txt` — read it fully before your first run. Summary:

1. **Check Kinova firmware is 2.5.x or newer.** On 2.4.0 the gripper write path silently no-ops. Check via the webapp (`http://192.168.1.10` → System) and update if needed.
2. **Build the Robotiq gripper driver from source** (not available via `apt`): the `wjwwood/serial` library (tylerjw fork, `ros2` branch), then `PickNikRobotics/ros2_robotiq_gripper`, then `colcon build --packages-select robotiq_driver --cmake-args -DBUILD_TESTING=OFF`. Required xacro/controller-type patches to `ros2_kortex` are documented in `docs/ros2_commands.txt` — apply them if you re-clone `ros2_kortex`.
3. **Calibrate the camera** (checkerboard intrinsics) and, if accuracy is poor, redo **hand-eye calibration** (`easy_handeye2`, `eye_in_hand`, `robot_base_frame:=world`). Commands and tips in `docs/ros2_commands.txt`.
4. **Confirm `/joint_states` and the wrist camera at 640×360** before relying on any IBVS servoing — a mismatched resolution silently throws off `TARGET_CX`/`TARGET_CY`.

## 2. Which file do I actually use?

Pick based on your task, then confirm the firmware/ROS pairing below before flashing or launching anything.

| Task | Current ROS 2 node | Matching firmware | Classifier |
|---|---|---|---|
| Scan-and-reach, cued trials | `ros2_nodes/scan_and_reach_online_experiment.py` | `firmware/sequence_experiment_cue.ino` | `matlab/classifiers/read_ssvep_ros2_fbcca_banded.m` |
| Scan-and-reach, verbal cue (no LED cue phase) | `ros2_nodes/scan_and_reach_verbal.py` | `firmware/sequence_experiment.ino` | `matlab/classifiers/read_ssvep_ros2_fbcca_banded.m` |
| Screwdriver jog | `ros2_nodes/wrist_jog_screwdriver_b.py` | `firmware/SSVEP_Screwdriver_Experiment.ino` | `matlab/classifiers/read_ssvep_ros2_fbcca_banded.m` |
| Offline-only SSVEP calibration block (no robot) | — | `firmware/SSVEP_Offline_Experiment.ino` | — |

Everything else in `ros2_nodes/` and `firmware/` (the `_table_final`, `_final`, `_demo`, `_demo_last`, `4colorLEDStrip*` files, `read_ssvep_ros2.m`, `read_ssvep_ros2_fbcca.m`) is an earlier iteration kept for reference/comparison, **not** meant to be run as-is. If you do open one, check its header comment for its exact serial protocol and frequency→class mapping — these are **not consistent across variants**.

## 3. Running a live session

1. **Start BrainVision Recorder** and confirm the LSL EEG stream is up (in MATLAB: `lsl_resolve_byprop(lib, 'type', 'EEG')` should return a non-empty result).
2. **Flash the matching Arduino sketch** (table above) and wire the trigger/serial lines per its header comment.
3. **Bring up the ROS 2 stack**, one terminal per step, in this order (full commands in `docs/ros2_commands.txt`):
   1. Arm driver (`kortex_bringup gen3.launch.py ... gripper:=robotiq_2f_85`)
   2. MoveIt `move_group`
   3. MoveIt RViz (optional, for visual confirmation)
   4. Wrist camera stream (`gscam2`, 640×360 — verify with `ros2 topic echo /image_raw --once --field width`)
   5. `sync_camera_info.py`
   6. `image_proc rectify_node`
   7. `apriltag_ros apriltag_node` with `config/tags_36h11.yaml`
   8. The ROS 2 node from the table above
4. **Run the classifier**: open `matlab/classifiers/read_ssvep_ros2_fbcca_banded.m`, confirm `channel_indices = [2 14 13 19 16 17 18]` matches your cap setup, and run it. It publishes `/ssvep_command` (and `/ssvep_scores` if the node listens for it).
5. **Verify before trusting any run**: check `ros2 topic echo /ssvep_command` fires when you expect, and watch the classifier's console output for a few seconds of rest/no-stimulus data — scores should sit near the noise floor, not lock onto one class (see troubleshooting below if they don't).
6. **Stop/reset**: use the kill-all-nodes block in `docs/ros2_commands.txt` rather than Ctrl-C-ing every terminal individually if something hangs.

## 4. Troubleshooting checklist

Work through these in order if something looks wrong before changing thresholds or code:

1. **Classifier locked onto one class with no stimulus present?** Check `channel_indices` — a floating channel (std ~1000+ vs ~60 for a real connected channel) will do this. Confirm against `[2 14 13 19 16 17 18]`, not against the full 32-ch label list.
2. **FBCCA scores look off / skewed to one frequency on rest data?** Check `fb_low_base`: use `14` (or the `m*8` scheme in `ssvep_combined_band.m`), not `16` (rolloff shoulder → systematic misdetection) or `8` (near-flat noise floor).
3. **Robot IK jumping to a weird pose / NO_IK_SOLUTION?** Run `ros2 run tf2_tools view_frames` and check the TF tree isn't split (e.g. `camera_frame` disconnected from the kinematic chain). Always look up poses in `world`, never `base_link` (`base_link` has no transform data — `buffer_length=0`).
4. **AprilTag detection sluggish during close-range servoing?** Check the control PC's CPU governor is in performance mode, not powersave.
5. **Gripper commands accepted but nothing moves?** Check firmware version (must be 2.5.x+) and that `robotiq_gripper_controller` is `position_controllers/GripperActionController` (plural `controllers` — the singular-named plugin doesn't exist and the default parallel-gripper controller silently drops the goal).
6. **LSL shows "connected" but no data flows?** That status only confirms UDP multicast discovery, not real TCP data. On multi-homed machines, force unicast to the correct IP via `KnownPeers` in `lsl_api.cfg`.
7. **Joint faults during continuous jog control?** Make sure joint 7 steps are computed from the *measured* current joint value, not an accumulated target — an accumulator drifts from reality.

## Reference

**Repo structure**
```
firmware/           Arduino sketches (LED stimulus + trigger + serial protocol)
ros2_nodes/         ROS 2 Python nodes (robot control, teach/calibration tools)
matlab/classifiers/ Live LSL -> ROS 2 classifier scripts (CCA / FBCCA variants)
matlab/utils/       Standalone test/support scripts (mock LSL sender, etc.)
config/             YAML configs (scan/delivery poses, AprilTag definitions)
docs/ros2_commands.txt   Terminal-by-terminal command reference for the full ROS 2 stack
```

**Hardware/software stack**: BrainVision actiCHamp + 32-ch CACS-32 cap, LSL streaming; MATLAB R2025b, FBCCA per Chen et al. 2015 (*J. Neural Eng.* 12(4):046008); KINOVA Gen3 7-DOF (`192.168.1.10`) + Robotiq 2F-85, ROS 2 Jazzy / Ubuntu 24.04, `kortex_bringup`, MoveIt 2; AprilTag `tag36h11` via `gscam2` + `apriltag_ros`; Arduino + WS2812B (FastLED).

**Home joint pose** (most scan-and-reach variants): `[0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57]`. The screwdriver-jog nodes use a nearby but distinct home waypoint — check the specific file rather than assuming.

**Dry-run without hardware**: `matlab/utils/mock_sender.m` streams a fake EEG LSL source at a chosen SSVEP frequency, so you can exercise the classifier → ROS 2 path without the amplifier.

## License

**No license has been chosen yet.** This is going into a private lab repo pending supervisor and Imperial IP/tech-transfer review before any decision on open-sourcing. Don't treat the missing `LICENSE` file as "no restrictions" — default copyright applies. Revisit once sign-off is obtained.
