# SSVEP-Controlled Supernumerary Robotic Arm

MSc dissertation project (Human and Biological Robotics, Imperial College London): controlling a supernumerary robotic limb — a KINOVA Gen3 arm acting as an extra "hand" — using a hybrid SSVEP brain-computer interface. A participant looks at a flickering LED target; the EEG response is classified in real time and translated into ROS 2 commands that drive the arm through scanning, visual servoing, grasping and delivery, or through simpler discrete/continuous jog control for the screwdriver task.

This repo is a reorganized copy of the working development repo (`ssvep-workspace-augmentation`), restructured for readability by future lab members rather than by commit history. See "Repo history and status" below before you dig in.

## System overview

1. **Stimulus**: an Arduino drives a WS2812B LED strip (FastLED) with 2–4 groups flickering at distinct frequencies (16/19/23/29 Hz, or a 4-target 8/12/15/18 Hz set for the scan-and-reach task). The participant fixates on the target they want to select.
2. **Acquisition**: a BrainVision actiCHamp amplifier + 32-channel CACS-32 cap stream EEG over LSL. Only a small occipital/parietal subset of electrodes is actually connected/gelled (see "Known gotchas" — this matters a lot).
3. **Classification**: a MATLAB script pulls the LSL stream, windows it, and runs Filter Bank CCA (Chen et al., 2015) against banded reference frequencies to vote on which target the participant is looking at. The winning class is published on ROS 2 topic `/ssvep_command` (`std_msgs/Int32`), with per-class scores optionally published on `/ssvep_scores`.
4. **Robot control**: a ROS 2 (Jazzy, Ubuntu 24.04) Python node subscribes to `/ssvep_command` and drives a KINOVA Gen3 7-DOF arm (+ Robotiq 2F-85 gripper) via `kortex_bringup` / MoveIt 2. Two task families exist:
   - **Scan-and-reach**: sweep a set of joint waypoints, detect an AprilTag (tag36h11) with a wrist camera, visually servo onto it, grasp, and deliver it to a fixed drop pose.
   - **Screwdriver jog**: interpret the winner frequency as "screw" / "unscrew" and jog wrist joint 7 accordingly, optionally handshaking with the Arduino over serial for cue/trial timing.
5. **Analysis**: MATLAB scripts turn recorded `.mat` sessions (or the live JSONL robot-event logs) into diagnostic plots, ITR/accuracy numbers, and figures for the write-up.

## Repository structure

```
firmware/           Arduino sketches (LED stimulus + trigger + serial protocol)
ros2_nodes/         ROS 2 Python nodes (robot control, teach/calibration tools)
matlab/
  classifiers/      Live LSL -> ROS 2 classifier scripts (CCA / FBCCA variants)
  analysis/         Offline analysis, plotting, and batch-processing scripts
  utils/            Standalone test/support scripts (mock LSL sender, etc.)
config/             YAML configs (scan/delivery poses, AprilTag definitions)
docs/
  analysis_notes/   Write-up notes from the analysis phase (methods, stats, backlog)
  figures/          Key figures (e.g. fig1_overview.png)
  ros2_commands.txt Terminal-by-terminal command reference for the full ROS 2 stack
```

## Hardware / software stack

- **EEG**: BrainVision actiCHamp, 32-ch CACS-32 cap, BrainVision Recorder, LSL streaming (`liblsl` MATLAB bindings).
- **Classification**: MATLAB (developed on R2025b) + EEGLAB. FBCCA per Chen et al. 2015, *J. Neural Eng.* 12(4):046008.
- **Robot**: KINOVA Gen3 7-DOF, IP `192.168.1.10`, Robotiq 2F-85 gripper. ROS 2 Jazzy on Ubuntu 24.04, `kortex_bringup`, MoveIt 2.
- **Vision**: AprilTag family `tag36h11`, `gscam2` RTSP wrist-camera stream, `apriltag_ros`.
- **Stimulus hardware**: Arduino + WS2812B LED strip via FastLED.
- **Connected EEG channels** (indices into the 32-ch actiCHamp layout): `[2 14 13 19 16 17 18]` → Fz (reference), P3, Pz, P4, O1, Oz, O2. **Do not assume all 32 channels are usable** — see gotchas below.
- **Home joint pose**: `[0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57]` (used by most scan-and-reach variants; the screwdriver-jog nodes use a nearby but distinct home waypoint — check the specific file).

Full terminal-by-terminal launch sequence, firmware requirements, gripper build-from-source steps, calibration procedures, and troubleshooting commands are in `docs/ros2_commands.txt`.

## Repo history and status — read this before running anything

The `ros2_nodes/` and `firmware/` folders intentionally keep **multiple iterations** of the same script side by side, because they represent real experimental evolution, not accidental duplication:

- `wrist_jog_screwdriver.py` → `wrist_jog_screwdriver_b.py` → `wrist_jog_screwdriver_demo.py` / `wrist_jog_screwdriver_demo_last.py`: the screwdriver-jog control node evolved from a trial-based, per-trial-homing design (serial handshake with `SSVEP_Screwdriver_Experiment.ino`) to a continuous-jog design with hysteresis voting, and finally to stripped-down demo variants. Check each file's docstring/header comment for its exact protocol and frequency-to-class mapping before wiring it to firmware — **the frequency-to-class mapping is not consistent across variants**.
- `scan_and_reach_ssvep_final.py` → `scan_and_reach_ssvep_table_final.py` → `scan_and_reach_online_experiment.py` / `scan_and_reach_verbal.py`: the scan-and-reach node evolved from hardcoded waypoints/tag mapping, to a YAML-driven action registry (`config/scan_config.yaml`), to versions with a JSONL event logger and a `/trial_event` bridge back to MATLAB for marker alignment, to a "verbal cue" variant with no LED cue phase (continuous flicker, experimenter names the target aloud). **`scan_and_reach_online_experiment.py` is the most complete/current version** for cued online experiments; `scan_and_reach_verbal.py` is the most current for the verbal-cue paradigm.
- `read_ssvep_ros2.m` → `read_ssvep_ros2_fbcca.m` → `read_ssvep_ros2_fbcca_banded.m`: classifier evolution from plain CCA, to Filter Bank CCA, to FBCCA with banded reference frequencies (±0.5 Hz per class) to tolerate FastLED timing drift. **`read_ssvep_ros2_fbcca_banded.m` is the current classifier** — the others are kept for comparison/ablation.
- `4colorLEDStrip.ino` → `4colorLEDStrip_2.ino` → `sequence_experiment*.ino` / `SSVEP_Offline_Experiment.ino` / `SSVEP_Screwdriver_Experiment.ino`: firmware evolution from a simple free-running 4-frequency flicker, to versions with EEG trigger pulses, cue phases, trial sequencing, and serial handshaking with the ROS 2 node. Match the firmware file to the ROS 2 node it was written for (see each file's header comment).

**Known repository issues flagged for fixing before wider/public use** (see `docs/analysis_notes/writeup_notes_analysis_phase.md`, section 7, for full detail):
- Joint-angle `unwrap()` should happen at logging time, not only at analysis time.
- The cued class should be written directly into the `cue_onset` marker payload rather than requiring manual re-entry during analysis (see "Cue / recording" gotchas below).
- At least one stale comment (`JOG_STEP_RAD`) does not match its actual value in one of the jog-node variants — verify constants against behaviour, not comments, when reusing a node.

## Known gotchas (hard-won, from the analysis phase)

**FBCCA / signal processing**
- `fb_low_base=16` sits on the filter rolloff shoulder and causes systematic misdetection; use `fb_low_base=14`, or the `m*8` scheme in `ssvep_combined_band.m`.
- `fb_low_base=8` gives a near-flat noise floor; `fb_low_base=16` skews hard toward 29 Hz (~75% on featureless/no-signal data). Always sanity-check on rest/no-stimulus data before trusting thresholds.
- Banded CCA references (±0.5 Hz, 5 candidates per class) are necessary — LED frequency drift from `FastLED.show()` is real and **non-uniform across LED groups** (the last group in the chain drifts worst).
- Higher-order Butterworth filters give negligible SNR benefit while introducing numerical instability and shrinking the usable window; use `zp2sos`/`sosfilt` if you need to go higher order.
- Apply Common Average Reference (CAR) unconditionally; add Laplacian-derived virtual channels as *additional* FBCCA inputs rather than replacing existing ones, and apply spatial filtering *after* bandpass filtering.

**Channels / hardware**
- The only reliably connected electrodes are indices `[2 14 13 19 16 17 18]` (Pz, P3, O1, Oz, O2, P4, with Fz as reference). Two channels that look plausible from the cap layout (indices 12/Pz and 21/P4 in some scripts) can be **floating** — std ~1279/1778 vs ~60 for genuinely connected channels — and feeding a floating channel into CCA produces spurious high-confidence "detections". Always resolve channels by index against the known-good list, not by label lookup against the full 32-ch montage.
- LSL "connected" only confirms UDP multicast metadata discovery, not actual TCP data flow. On multi-homed Ubuntu machines, multicast can broadcast on the wrong interface; fix by forcing unicast to the correct IP via `KnownPeers` in `lsl_api.cfg`.
- A development session run on 22 July logged the classifier running at ~83 Hz instead of the intended ~3.9 Hz decision rate — that session's data should be excluded from any timing-sensitive analysis.

**Robot / ROS 2**
- The Kinova Gen3's Ethernet port is hardware-limited to 100BASE-TX; do not chase gigabit throughput on that link.
- Do not hard-power-cycle the Gen3 via the physical button — this risks firmware corruption. Use the webapp/soft reset paths.
- All TF lookups must use the `world` frame, not `base_link` (`base_link` has `buffer_length=0` — no transform data is ever published for it; `world -> base_link` is a static identity transform, so `world` is always safe to use instead).
- Keep the control PC's CPU governor in performance mode during operation — throttling visibly degrades AprilTag detection throughput during close-range visual servoing.
- A disconnected TF tree (e.g. `camera_frame` not connected to the kinematic chain) causes IK to receive garbage poses and jump to a distant, unwanted redundancy branch. Diagnose with `ros2 run tf2_tools view_frames`.
- Step joint 7 from its *measured* current value rather than an accumulated target — an accumulator drifts from reality and causes joint faults.
- `send_joint_goal`-style trajectory publishers need an explicit `t=0` starting waypoint at the current position, or the joint trajectory controller can silently reject the goal.

**Cue / recording**
- The cued class is **not** recorded automatically in the online `.mat` files from the discrete-trial pipeline — it must be entered by hand during analysis (see `analyze_discrete_online.m`), which means online accuracy can't be independently re-verified from the data alone without that manual labeling step. Fixing this (writing the cue into the marker payload directly) is on the backlog.
- The filename timestamp on recordings is the flush/end time in local time, not the start; use the `t0u` field in the `.mat` file for the true UTC start.
- The `wrist_net` marker has inverted sign on rightward-rotation trials in at least one dataset; recompute net rotation from the unwrapped `wrist_jog` series referenced to the home joint angle (1.57 rad) rather than trusting the stored marker directly.
- The first logged jog target in a trial is already the *post-step* position — you need to reference back to the home angle to recover the actual first step size.

## Getting set up

This is not a one-command setup — see `docs/ros2_commands.txt` for the full terminal-by-terminal sequence (arm driver, MoveIt, wrist camera stream, AprilTag detection, and the main control node), including the one-time gripper build-from-source steps (the Robotiq 2F-85 hardware interface plugin isn't available via `apt` and must be built from source, with several required patches to `ros2_kortex` documented inline). Firmware on the Kinova arm must be **2.5.x or newer** — on 2.4.0 the gripper write path silently no-ops.

Rough order:
1. Flash the appropriate `firmware/*.ino` sketch to the Arduino for your task (screwdriver jog vs. scan-and-reach vs. offline experiment) — match it to the ROS 2 node you intend to run (see "Repo history and status" above).
2. Start BrainVision Recorder and confirm the LSL EEG stream is visible (`lsl_resolve_byprop(lib, 'type', 'EEG')` should return a result).
3. Bring up the ROS 2 stack per `docs/ros2_commands.txt` (arm driver → MoveIt → camera → AprilTag → control node).
4. Run the appropriate MATLAB classifier from `matlab/classifiers/` (`read_ssvep_ros2_fbcca_banded.m` is current) — it publishes `/ssvep_command`.
5. For offline work, point the scripts in `matlab/analysis/` at a recorded session; `plot_run_overview.m` / `batch_plot_recordings.m` are good starting points for a quick look at a new recording.

## License

**No license has been chosen yet.** This code is going into a private lab repository pending review by the supervisor and Imperial's IP/tech-transfer process before any decision on open-sourcing or public release. Do not treat the absence of a `LICENSE` file as "no restrictions" — default copyright applies, and Imperial College London's IP policy governs work produced as part of this MSc project. Revisit this once supervisor/Imperial sign-off is obtained.
