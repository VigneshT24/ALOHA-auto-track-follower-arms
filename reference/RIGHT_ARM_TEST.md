# Experimental right follower

This uses the measured base separation of 0.9652 m and assumes facing bases
(180-degree yaw), equal heights, and alignment along left-base X. Small base
misalignment and measurement uncertainty remain; validate without movement first:

```python
# Input is /aloha/perception/detections_base_filtered in follower_left/base_link.
x_right = 0.9652 - x_left_base
y_right = -y_left_base
z_right = 0.300
```

The source is NOT RQT's table-centred workspace output. For input (.50, .02),
the right target is (.4652, -.02, .300) metres. The input remains unchanged.
The existing left launch still uses base X/Y without an additional offset.
The target requests the normalized (0, .7071, 0, .7071) quaternion. The shared
planner now automatically evaluates A-D orientations and multiple IK seeds.

Stop existing bringup/planner/detection processes first; disconnect Wizard.
The left shoulder overload is unresolved. The dedicated bringup below starts
only the right follower and high camera, not the left arm or leaders.
Bringup still initializes and enables the right hardware according to its normal
mode configuration. Preview refers to the new target/planner nodes, not bringup.

## Build

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/local_setup.bash
cd ~/interbotix_ws
/usr/bin/colcon build --base-paths src/aloha_follow_planner src/follower_left_moveit_config src/follower_right_moveit_config --packages-select aloha_follow_planner follower_left_moveit_config follower_right_moveit_config
source ~/interbotix_ws/install/local_setup.bash
cd ~/interbotix_ws/src/aloha_tracking_ws
/usr/bin/colcon build --base-paths aloha_yolo_kcf_tracking/aloha_perception --packages-select aloha_tracking_perception
```

## Terminal 1: right follower and high camera

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/local_setup.bash
source ~/interbotix_ws/src/aloha_tracking_ws/install/local_setup.bash
ros2 launch aloha_tracking_perception right_tracking_bringup.launch.py
```

This reuses the installed ALOHA camera serial and right mode configuration.
It skips EEPROM configuration loading (`load_configs=false`), expecting the
existing robot configuration. Normal driver startup still performs its existing
shadow calibration and mode setup. It is not a calibration repair procedure.

## Terminal 2: right planner, initially without hardware commands

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/local_setup.bash
ros2 launch follower_right_moveit_config run_planner.launch.py execute_motion:=false
```

## Terminal 3: inspect the mapping first

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/local_setup.bash
source ~/interbotix_ws/src/aloha_tracking_ws/install/local_setup.bash
ros2 launch aloha_tracking_perception truck_target_right.launch.py \
  model:="$HOME/interbotix_ws/src/aloha_tracking_ws/yolov8m.pt" \
  preview_only:=true
```

Input and mapped output are logged. Preview poses are sent only on
`/follower_right/grasp_pose_preview`, never to the arm target topic.

After inspecting those values, restarting Terminal 3 with `preview_only:=false`
feeds `/follower_right/grasp_pose` to Terminal 2. While Terminal 2 retains
`execute_motion:=false`, it automatically selects and displays a scored feasible path without hardware
commands. Run `ros2 launch follower_right_moveit_config moveit_rviz.launch.py`
to view it; the RViz-only launch supplies the right model directly.

## Explicit movement mode

Only after target/IK inspection and confirming the right hardware is healthy:
stop Terminal 3, restart Terminal 2 with `execute_motion:=true`, then restart
Terminal 3 with `preview_only:=false`. All other arguments stay the same.
Those flags can trigger right-arm movement immediately on a valid detection.
Do not run a left tracker alongside this detector/converter launch.

The shared `aloha_follow_planner` implementation scores complete planned paths
using weighted joint travel, maximum excursion, joint-limit margins and URDF
static shoulder/elbow gravity moment estimates. Lower score does not prove lower
physical motor strain. Revision 8 streams bounded timed setpoints along the selected path, allowing
intermediate tracking lag without requiring a stop at every tiny sample. It
pauses progress at 0.04 rad lag, stops above 0.06 rad, and detects per-joint stalls.
Final convergence and primary/shadow motor health checks remain required.
RViz uses the same nominal schedule; hardware can pause its path clock. Targets that cannot
plan are recoverably rejected; motor/feedback faults remain latched. No automatic
reboot, torque re-enable or shutdown fold is performed.

See `aloha_follow_planner/README.md` in the main workspace source tree for the
score formula, limits, matching left/right commands, and execution details.
Final convergence timeout is 20 s, execution timeout 240 s, acknowledgement timeout 300 s.
Right motor profiles remain unchanged (velocity 5, acceleration 2).

## Files for the eventual repository

Keep the shared `aloha_follow_planner` package, complete perception package and BOTH MoveIt configuration packages,
including their hidden `.setup_assistant` files. The additions are:

- `aloha_tracking_perception/right_target.py`
- `launch/truck_target_right.launch.py`
- `launch/right_tracking_bringup.launch.py`
- right planner/launch/dependency updates in `follower_right_moveit_config`
- shared target publisher and tests updated for both arms

The installed ALOHA, Interbotix description/control/driver and RealSense packages
remain external dependencies. The high-camera calibration is lab-specific.


## Revision 9: repeated following and detector Ctrl+C

Shared nominal tracking speed is 0.08 rad/s. The second-target shoulder lag seen
in the recorded run is permitted within a bounded 0.06-rad soft / 0.08-rad hard
tracking envelope; stall detection and hardware fault checks remain active.
The detector still requires a fresh, single valid car/truck/train detection.
It completes the current path before submitting the latest moved target (2 cm
minimum change). `FOLLOW WAIT` logs explain unavailable detections/status.

With bringup and planner running in separate terminals, Ctrl+C once in the
**detector** terminal requests priority rest. Wait for `Rest reached and verified;
detector exiting.` before stopping the planner or bringup. The arm returns to a
collision-checked legal folded pose derived from the driver's sleep positions;
factory sleep angles outside URDF/motor limits are clamped inward. Preview never
commands rest. A hardware fault or invalid path prevents rest and is reported.
A second Ctrl+C abandons the wait, not a guaranteed motor stop. Restart planner
and detector before tracking again after a rest request.
