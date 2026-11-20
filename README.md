# ALOHA auto-track follower arms

YOLO-based object following for the left and right Mobile ALOHA follower arms, using an overhead RealSense camera and MoveIt planning.

The detector estimates an object's position, maps it into the selected arm's base frame, and asks the planner for a reachable pose. The planner compares several grasp orientations and IK solutions, scores the resulting paths, and displays or executes the selected path. The current application follows a toy vehicle at a fixed height; it does not grasp it.

## Project status

**Work in progress. This is a development snapshot, not a finished robot application.**

- **Left follower:** tracking operated in earlier testing. Current testing is blocked by a shoulder reference/calibration issue on our hardware. The folded physical pose reports shoulder encoder positions outside the configured joint range, so the planner refuses to execute. The software is intended to be reusable on a correctly configured ALOHA arm, but that has not yet been verified on another robot.
- **Right follower:** movement and repeated-target following have worked in testing, but accuracy has drifted across successive target moves. Return-to-rest has also failed, including an earlier path that lowered the end effector toward the table. The latest code adds an upward/inward retreat and clearance checks; extended hardware validation is still pending.
- **Detector shutdown:** Ctrl+C is intended to request rest before the detector exits. Earlier testing found that this worked during motion but could fail after the arm settled. Both cases still need a complete hardware retest with the current code.

These issues remain open. A successful preview or one successful movement does not establish that the full tracking and shutdown sequence works reliably.

## What is included

```text
src/
  aloha_follow_planner/          
  aloha_tracking_perception/   
  follower_left_moveit_config/
  follower_right_moveit_config/
  aloha/                     
  interbotix_xs_driver/       
  interbotix_ros_core/       
  interbotix_ros_manipulators/ 
  interbotix_ros_toolboxes/    
models/yolov8m.pt        
requirements-yolo.txt       
diagnostics/                  
reference/                     
```

The Interbotix directories are source snapshots with local changes, not Git submodules. Their original licenses are retained. Upstream URLs, revisions and the local modification list are recorded in `reference/upstream_versions.txt`. The documents in `reference/` describe earlier stages and can contain superseded settings; use this README and the current source for the present behavior.

## Environment and dependencies

The development system uses Ubuntu 22.04, ROS 2 Humble and system Python 3.10. Required software includes:

- MoveIt 2 **with `moveit_py`**, `moveit_configs_utils`, OMPL and RViz.
- The Interbotix X-Series SDK, messages, driver, arm control and ALOHA VX300S descriptions.
- Dynamixel Workbench and its dependencies for the driver.
- The ROS RealSense driver, `cv_bridge`, OpenCV, NumPy and PyYAML.
- PyTorch and Ultralytics for inference. CUDA is optional but HIGHLY recommended; install a PyTorch build appropriate for the machine.
- `colcon` and the usual ROS build tools. Offline tests also use pytest and SciPy.

`requirements-yolo.txt` covers inference dependencies only. It does not install ROS, MoveIt, the camera driver or Interbotix software, and it is not a pinned environment lockfile.

Use a Python 3.10 environment that can import the system ROS packages. An unrelated Conda environment can put a different Python interpreter ahead of ROS and prevent the planner from starting. Our commands use `/usr/bin/colcon` and explicitly select `/usr/bin/python3` for CMake packages.

A clean-machine installation from this snapshot has **not** been validated yet. Install the platform dependencies before building; copying this repository alone is not a complete system installation.

## Existing Interbotix installation or bundled source?

This choice depends on the software already installed, not the computer brand. A System76 machine does not need different project code.

### Reuse an existing installation

Keep the existing workspace intact and source it as an underlay. Build the project packages in this repository as a separate overlay. This avoids copying files over the existing installation.

The example below assumes the existing workspace is `~/interbotix_ws`. Change that path if necessary. This builds the bundled `aloha` package in the overlay because the bringup launches use its camera and follower mode configuration; it does not rebuild the bundled Interbotix driver trees.

Before using this option, compare the installed robot descriptions and driver against `reference/upstream_versions.txt`. In particular, this snapshot contains local changes to the description's `urdf/control.urdf.xacro`, driver/SDK build files and ALOHA configuration. Reusing an incompatible underlay has not been tested. Preserve your own camera and motor configuration when adapting the overlay.

```bash
# Run in a fresh terminal, outside Conda.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/local_setup.bash

cd ~/ALOHA-auto-track-follower-arms
/usr/bin/colcon build \
  --base-paths src/aloha src/aloha_follow_planner \
    src/aloha_tracking_perception \
    src/follower_left_moveit_config src/follower_right_moveit_config \
  --packages-select aloha aloha_follow_planner aloha_tracking_perception \
    follower_left_moveit_config follower_right_moveit_config \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### Build the bundled Interbotix source

Use this path when setting up a separate source workspace or when deliberately testing the recorded Interbotix versions. Do not copy these directories into another workspace that already contains packages with the same names.

After installing the external dependencies listed above, the following builds the packages used by the dedicated follower bringups. It intentionally selects the arm stack rather than every unrelated example and mobile-base package in the upstream snapshots. The bundled ALOHA package also declares dependencies for its broader mobile-base application; those applications are outside these follower-only instructions.

```bash
# Run in a fresh terminal, outside Conda; do not source another Interbotix workspace.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash

cd ~/ALOHA-auto-track-follower-arms
/usr/bin/colcon build --base-paths src \
  --packages-select \
    interbotix_common_modules interbotix_xs_modules \
    interbotix_xs_msgs interbotix_xs_driver interbotix_xs_sdk \
    interbotix_xsarm_descriptions interbotix_xsarm_control \
    aloha aloha_follow_planner aloha_tracking_perception \
    follower_left_moveit_config follower_right_moveit_config \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

This build path is provided for review and still needs clean-machine verification. Some upstream submodule directories in the snapshot are unpopulated; the commands above require external dependencies such as Dynamixel Workbench to be installed separately. Do not assume that running `git submodule update` on this repository will restore the upstream repositories' submodules.

Neither build command overwrites the original source workspace. Sourcing the new overlay makes its built packages take precedence in that shell. Use a fresh shell with only the old workspace sourced to return to the old installation.

## Configure the robot before running

The saved configuration describes our lab. Review these files for another setup:

| File | Setting to check |
| --- | --- |
| `src/aloha/config/rs_cam.yaml` | Overhead camera serial under `/cam_high/camera` |
| `src/aloha/config/follower_modes_left.yaml` | Left serial device, operating mode and motor profiles |
| `src/aloha/config/follower_modes_right.yaml` | Right serial device, operating mode and motor profiles |
| `src/aloha_tracking_perception/aloha_tracking_perception/calibration_candidate.yaml` | Camera-to-left-base calibration |
| `src/aloha_tracking_perception/aloha_tracking_perception/lab_workspace.yaml` | Lab workspace settings |
| `src/aloha_tracking_perception/aloha_tracking_perception/target_once.py` | Right-base separation and mapping in `map_target()` |
| `src/aloha_follow_planner/config/path_scoring.yaml` | Scoring and execution limits |

The current port names are `/dev/ttyDXL_follower_left` and `/dev/ttyDXL_follower_right`. Set up the appropriate device aliases or change the configuration to match the actual devices. Disconnect Dynamixel Wizard before starting ROS hardware bringup.

Both bringups set `load_configs=false`. They expect an already configured robot; they do not install or repair the saved EEPROM motor configuration. Normal SDK startup still initializes operating modes, calibrates shadow offsets as configured, and enables torque. **Bringup is a real hardware operation, even when the planner is in preview mode.**

Rebuild affected packages after changing source configuration, then source the overlay again in every terminal you restart.

## Coordinates and following behavior

All target coordinates are in metres. Detection is converted into the **left arm's base frame** first. The current target mapping is:

```python
# Left follower
x_target = x_left_base
y_target = y_left_base
z_target = 0.300

# Right follower: lab approximation for facing, aligned bases
x_target = 0.9652 - x_left_base
y_target = -y_left_base
z_target = 0.300
```

The right mapping assumes equal base heights, 180-degree relative yaw and separation along left-base X. Measure and replace this approximation for another robot. The table-centred workspace output is for visualization; it is not the arm target input.

`Z = 0.300 m` is an **absolute height in the arm base frame**, not a 30 cm offset above the detected object. Detected Z does not change the requested target height. Camera depth still contributes to estimating the object's base-frame position. The shared planner also enforces the fixed target Z.

The router accepts a fresh, single valid car/truck/train detection. It waits for the current plan to finish, then submits the latest target if it has moved at least 2 cm in XY. This is sequential object following, not continuous visual servoing or immediate replanning while the arm moves. Ambiguous or missing detections can prevent another target from being sent.

Only run one arm's detector/converter stack at a time with these commands. The two launches share perception node names and topics. Simultaneous dual-arm tracking has not been configured or validated.

## Run one arm

Start with preview. The commands below assume the repository is at `~/ALOHA-auto-track-follower-arms` and the build has succeeded.

### Environment in every terminal

For the existing-installation build, run:

```bash
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/local_setup.bash
source ~/ALOHA-auto-track-follower-arms/install/local_setup.bash
```

For the bundled-source build, omit the middle line. Do not source the old nested perception workspace after this overlay, because that can select the old package instead of this copy.

### Right follower: preview

**Terminal 1 — right arm and overhead camera**

```bash
ros2 launch aloha_tracking_perception right_tracking_bringup.launch.py
```

**Terminal 2 — planner, hardware execution disabled**

```bash
ros2 launch follower_right_moveit_config run_planner.launch.py \
  execute_motion:=false
```

**Terminal 3 — RViz trajectory preview**

```bash
ros2 launch follower_right_moveit_config moveit_rviz.launch.py
```

**Terminal 4 — detector and target publisher**

```bash
ros2 launch aloha_tracking_perception truck_target_right.launch.py \
  model:="$HOME/ALOHA-auto-track-follower-arms/models/yolov8m.pt" \
  preview_only:=false
```

### Left follower: preview

The current left shoulder reference issue must be resolved before hardware execution. These are the equivalent launch commands for a correctly configured left arm.

**Terminal 1 — left arm and overhead camera**

```bash
ros2 launch aloha_tracking_perception left_tracking_bringup.launch.py
```

**Terminal 2 — planner, hardware execution disabled**

```bash
ros2 launch follower_left_moveit_config run_planner.launch.py \
  execute_motion:=false
```

**Terminal 3 — RViz trajectory preview**

```bash
ros2 launch follower_left_moveit_config moveit_rviz.launch.py
```

**Terminal 4 — detector and target publisher**

```bash
ros2 launch aloha_tracking_perception truck_target_once.launch.py \
  model:="$HOME/ALOHA-auto-track-follower-arms/models/yolov8m.pt" \
  preview_only:=false
```

Despite the historical `truck_target_once` filename, the current router can publish successive targets after completion.

### What the two flags mean

| Planner `execute_motion` | Detector `preview_only` | Result |
| --- | --- | --- |
| `false` | `true` | Log/publish preview targets; no target reaches the planner's normal target topic |
| `false` | `false` | Plan and display the selected trajectory; no planner motor commands |
| `true` | `false` | Plan and execute on the selected arm when a valid target arrives |

For RViz path preview, use **planner false, detector false** as shown above. Detector `preview_only=true` is useful for inspecting coordinate mapping alone.

Open RViz before starting the detector so it is ready for the first plan. RViz does not locate the truck: the detector and coordinate converter do that. The planner publishes the resulting trajectory. The preview display uses `/follower_right/display_planned_path` or `/follower_left/display_planned_path` and receives its robot description through the RViz launch.

This preview uses real joint feedback and the real camera. It is not a Gazebo simulation. To execute after reviewing a preview, stop the detector, restart Terminal 2 with `execute_motion:=true`, then restart the detector with `preview_only:=false`. This computes a new plan from the current state; it does not approve and replay a stored preview. Movement can start as soon as a valid detection is received. RViz is optional during execution.

## Planning and execution

The shared planner evaluates A–D orientation candidates with multiple IK seeds and selects the lowest-scoring successful path found within its planning budget. This is a bounded search, not a guarantee of the globally best path.

The score combines weighted joint travel, maximum excursion, joint-limit margins and estimated static shoulder/elbow gravity moments from the URDF. It is a heuristic for comparing paths. It does not measure servo strain or predict overload reliably.

Execution publishes monitored `JointGroupCommand` setpoints to the Interbotix SDK. It does **not** use a connected MoveIt `FollowJointTrajectory` hardware controller. The included MoveIt controller configuration alone does not provide that bridge; use the planner's `execute_motion` flag for this implementation.

Current shared defaults in `path_scoring.yaml` are:

| Setting | Value |
| --- | --- |
| Nominal tracking speed | 0.08 rad/s |
| Command period | 0.1 s |
| Tracking pause error | 0.06 rad |
| Tracking hard error limit | 0.08 rad |
| Final waypoint tolerance | 0.035 rad, about 2 degrees |
| Final convergence timeout | 20 s |
| Execution timeout | 240 s |

The controller allows intermediate tracking lag rather than stopping at every sample. Its path clock can pause while the arm catches up, so physical motion may take longer than the nominal RViz animation. Servo profiles also affect motion. Increasing YAML profile values alone does not remove the planner's speed limit.

Motor faults, stale feedback and stalls stop execution and can latch a failed state. There is no automatic fault reset or torque re-enable. A target that cannot be planned is handled separately as a rejected target. Resolve a hardware/execution fault before restarting the planner and detector.

Collision checks cover the robot and modeled scene only. The detected toy, nearby people, cables, the other arm and arbitrary table objects are not automatically added as obstacles. Fixed target height is not a guarantee of physical fingertip or swept-path clearance.

## Ctrl+C and return to rest

With bringup and planner still running, pressing Ctrl+C **once in the detector terminal** stops new target publication and requests priority rest. The router keeps its ROS context alive while it waits for the planner's matching rest result, for up to 300 seconds.

The current rest code first attempts an upward/inward retreat, then a checked fold toward a rest pose inset from the configured limits. It includes collision, clearance, feedback and motor-health checks. A failed check prevents further planned commands; it does not force the fold through an obstacle or clear a motor fault.

Wait for the reported rest result before stopping the planner or bringup. This behavior is still experimental, particularly when requested after the arm has settled. Do not treat Ctrl+C as a verified emergency stop or a guaranteed successful rest maneuver.

- With planner execution disabled and detector `preview_only=false`, rest is planned/displayed without motor commands.
- With detector `preview_only=true`, no rest request is made.
- A second Ctrl+C abandons the detector's wait; it is not a guaranteed physical stop.
- Stopping the planner does not command a return to rest or torque-off. The motors may retain their last goal.
- Restart the planner and detector before tracking again after a rest request.

## Tests and diagnostic tools

Run package tests after building and sourcing the selected environment:

```bash
cd ~/ALOHA-auto-track-follower-arms
/usr/bin/colcon test \
  --packages-select aloha_follow_planner aloha_tracking_perception
/usr/bin/colcon test-result --verbose

/usr/bin/python3 -m pytest diagnostics/test_left_supported_test.py -q
```

The original development workspace reported 125 passing package tests and two skipped ROS integration tests because that test environment could not create the required network sockets. These results predate a clean build of this copied repository. Unit and geometry tests do not replace hardware validation.

`diagnostics/run_left_supported_test.py` is a lab-specific one-target recording harness. It still references the original workspace/model layout and should be adapted before use elsewhere. It is not the normal tracking entry point. Its `--execute` option authorizes hardware movement; do not use it to bypass the unresolved left shoulder bounds check.

## Remaining work

- Establish the correct left shoulder reference and retest on healthy hardware.
- Resolve and measure right-arm target error across repeated moves.
- Verify the rest path and detector shutdown both during motion and after settling.
- Validate fresh installation, dependency versions and the existing-underlay option.
- Review package metadata, project licensing and model redistribution before release. The perception package currently has an unspecified license; upstream components retain their own licenses.

No automatic calibration repair, fault clearing or motor-limit widening is part of this project.

## Credits
Robotics & Explainable AI Lab
* PI: Mingyu Cai
* Vignesh Thondikulam
* Anthony Martinez
* Nathan Chen
* Kamren James