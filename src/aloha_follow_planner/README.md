# Automatic scored following for either ALOHA follower

Both `follower_left_moveit_config` and `follower_right_moveit_config` now invoke
this one shared implementation. `execute_motion` defaults to **false** for both.
The detector/converter coordinate mapping is unchanged:

- Left: filtered left-base X/Y, fixed Z = 0.300 m.
- Right: X = 0.9652 - left-base X; Y = -left-base Y; Z = 0.300 m.

The right transform remains an approximation for facing, equally high bases.
Use one arm's workflow at a time. The separate single-arm models do not model
collisions with the other arm, and both detector launches use the same high camera.
The previously observed left shoulder/shadow overload is not repaired by this code.

## What happens automatically

1. Accept a fresh target with the correct arm frame and fresh measured joints.
2. Test the requested gripper orientation and base-frame yaw rotations of 180,
   +90 and -90 degrees (A-D). These are alternative orientations, not the same pose.
3. Seed IK from measured joints, the previous selected endpoint when available,
   zero/positive/negative forearm-roll branches, and the joint-range midpoint.
4. Sort distinct IK endpoints by an initial heuristic. Attempt up to two OMPL
   plans per orientation, with a 12-second search budget and 0.75 seconds per plan.
5. Check the path, including its start, against joint bounds and sampled collisions
   in the known scene. The ground box has its top at model Z = 0; it is not a
   camera-generated table model. No camera obstacle reconstruction is enabled.
6. Score successful complete paths, select the lowest score among those tested,
   publish it for RViz, and print `CANDIDATE` / `SELECTED` JSON records.
7. If execution is disabled, return ready without any motor service calls or
   commands. If enabled, track this same selected geometric path without rerunning IK.

The search is bounded and IK/OMPL can be nondeterministic. It does not find a
provably global optimum. Restarting with execution enabled replans from fresh
measurements; it does not approve or replay a saved preview from a prior process.

## Score

For each candidate, the score is:

```
sum(joint_weight[j] * total_absolute_joint_travel[j])
+ excursion_weight * largest_joint_excursion_from_start
+ margin_weight * near_limit_penalty
+ shoulder_gravity_weight * peak_estimated_shoulder_gravity_moment
+ elbow_gravity_weight * peak_estimated_elbow_gravity_moment
```

Travel/excursion use radians without wrapping bounded joints. The limit penalty
increases in the outer 10% of a joint range, using average path exposure plus
the endpoint penalty so the folded start cannot hide a poor endpoint. Gravity moments use URDF link masses,
inertial COM origins, joint axes, forward kinematics and downward gravity across
the sampled path. They are total joint moments in Nm, not an individual paired
servo's load. `payload_mass_kg` adds a point payload at `gripper_link` (default 0).

Defaults are in [config/path_scoring.yaml](config/path_scoring.yaml):

- Joint weights `[1, 3, 2, 1.5, 1.5, 1.5]` for waist, shoulder, elbow,
  forearm roll, wrist angle, wrist rotation.
- Excursion 2; near-limit penalty 2; shoulder gravity 3; elbow gravity 1.5.

This weighted score is a comparison heuristic, not measured motor strain. It
excludes dynamics, friction, cable forces, inaccurate masses/calibration and
motor-pair fighting. A successful path/low score cannot establish that holding
it is safe for the physical motors.

## Starting from the measured rest pose

The logged folded rest pose is up to about 0.041 rad outside some URDF limits.
The planner allows at most `start_bounds_tolerance=0.05` rad, explicitly prepends
a short checked connection into bounds, and displays that connection from the
actual measured start. Each initial violation must decrease monotonically; a
later excursion outside bounds is rejected. Larger errors fail and require
calibration inspection. This does not alter URDF or motor limits. The planner
rejects a colliding start instead of permitting an invisible start-state repair.

## Hardware execution — revision 9

The shared executor now streams timed positions along the selected geometric
path; it no longer requires the shoulder to settle at every tiny intermediate
sample. This corrects the software gate observed at waypoint 2 with a persistent
0.023-rad shoulder lag. It does not repair physical load sharing/calibration.

`tracking.py` defines a piecewise-linear schedule, shared by RViz and execution.
Each segment lasts at least one control period and obeys the nominal joint speed.
The execution clock can pause, so RViz shows nominal timing rather than exact
hardware wall time. Physical acceleration remains controlled by the unchanged
Interbotix motor profiles; this is not a new dynamically validated torque model
or a FollowJointTrajectory action bridge.

Defaults in `config/path_scoring.yaml`:

| Setting | Value | Behavior |
|---|---|---|
| tracking_speed | 0.08 rad/s | Maximum nominal joint command speed |
| tracking_period | 0.1 s | Nominal 10-Hz setpoint stream |
| tracking_pause_error | 0.06 rad | Pause the path clock if the next setpoint would lead feedback too far |
| tracking_error_limit | 0.08 rad | Fail on excessive tracking error (about 4.6 degrees) |
| stall_error_threshold | 0.048 rad | Begin checking lack of progress for an individual joint |
| stall_timeout | 5 s | Fail if a joint with persistent error makes no required forward progress |
| stall_min_progress | 0.003 rad | Required per-joint progress before resetting its stall timer |
| waypoint_tolerance | 0.035 rad | Final destination tolerance (about 2 degrees) |
| waypoint_timeout | 20 s | Final convergence timeout only; no intermediate settling timeout |
| execution_timeout | 240 s | Overall execution deadline |

The path clock cannot catch up with a jump after a lag pause, service delay, or
CPU delay. It advances by at most one control period and stops at the next
geometric vertex. Duplicate encoder goals are not republished. A frozen shoulder
cannot be hidden by wrist motion: stall monitoring is per joint.

Before and during execution:

- Require a fresh start near the selected start, correct driver mode/order,
  supported XM models, and nonzero profile velocity <= 6 / acceleration <= 3
  for the right arm (left remains <= 5 / <= 2).
  Existing motor profiles and all registers are read only.
- Identify exactly one matching XS driver by node name/namespace. Passive topic
  observers no longer prevent execution.
- Read the motors' stored encoder limits and reject commands that violate them.
- Never send out-of-bounds initial display samples; wait until the short checked
  measured-start connection reaches a legal target.
- Recheck each measured-to-target connection against sampled model collisions.
- Require fresh feedback; check primary arm and both shoulder/elbow shadow motor
  errors and torque before motion and approximately once per second.
- Use actuator-quantized targets for tracking/final error calculations.
- Require final measured convergence; a permanent final residual above 0.035 rad
  still fails instead of falsely reporting success.

Failures latch `failed`; infeasible targets before motion are recoverably rejected.
No automatic reboot, torque-enable, or profile/gain write is issued. Detector
Ctrl+C explicitly requests priority rest as described below; planner shutdown
itself does not command rest. Halting the stream stops new commands; **the last sent goal remains held
by the servo**, and the arm does not automatically torque off. This is not an
emergency stop. A stall/fault can therefore still require operator intervention.
Sampled collision checking does not guarantee continuous real-world clearance.

The SDK's queued trajectory implementation still has the whole-second timing
issue described in the audit, so this executor uses the existing group position
topic, not that queue or unprovided FollowJointTrajectory action servers.
The perception completion acknowledgement remains 300 s (> planning + execution).

## Build

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/local_setup.bash
cd ~/interbotix_ws
/usr/bin/colcon build \
  --base-paths src/aloha_follow_planner src/follower_left_moveit_config src/follower_right_moveit_config \
  --packages-select aloha_follow_planner follower_left_moveit_config follower_right_moveit_config \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source ~/interbotix_ws/install/local_setup.bash
cd ~/interbotix_ws/src/aloha_tracking_ws
/usr/bin/colcon build --base-paths aloha_yolo_kcf_tracking/aloha_perception \
  --packages-select aloha_tracking_perception
```

## Three terminals

Source this in each terminal (outside Conda):

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/local_setup.bash
source ~/interbotix_ws/src/aloha_tracking_ws/install/local_setup.bash
```

Choose ONE column, starting with preview mode:

| Terminal | Right follower | Left follower |
|---|---|---|
| 1: bringup | `ros2 launch aloha_tracking_perception right_tracking_bringup.launch.py` | `ros2 launch aloha_tracking_perception left_tracking_bringup.launch.py` |
| 2: planner | `ros2 launch follower_right_moveit_config run_planner.launch.py execute_motion:=false` | `ros2 launch follower_left_moveit_config run_planner.launch.py execute_motion:=false` |
| 3: detector | `ros2 launch aloha_tracking_perception truck_target_right.launch.py model:="$HOME/interbotix_ws/src/aloha_tracking_ws/yolov8m.pt" preview_only:=false` | `ros2 launch aloha_tracking_perception truck_target_once.launch.py model:="$HOME/interbotix_ws/src/aloha_tracking_ws/yolov8m.pt" preview_only:=false` |

`preview_only:=false` sends targets to the planner; it does not override
`execute_motion:=false`. With detector `preview_only:=true`, no targets reach
the planner, so it cannot generate an RViz path.

Bringup itself initializes/enables hardware according to the existing mode file;
preview protection applies to planner output, not hardware startup. Disconnect
Dynamixel Wizard before bringup. Profile files remain in the external ALOHA
package: `aloha/config/follower_modes_left.yaml` and `follower_modes_right.yaml`.
The planner does not change the gripper mode/configuration.

For RViz, source the same environment in another terminal and run the matching
command, preferably before starting the detector:

```bash
ros2 launch follower_right_moveit_config moveit_rviz.launch.py
# OR
ros2 launch follower_left_moveit_config moveit_rviz.launch.py
```

The RViz launchers load their model directly and use the arm-specific trajectory
topic. The planner also publishes `/display_planned_path` for old single-arm
viewers. For a candidate report:

```bash
ros2 topic echo /follower_right/path_selection --once --field data
# Left: /follower_left/path_selection
```

To execute automatically, stop detector first, restart the matching planner with
`execute_motion:=true` (leave legacy `compare_orientations` at default false), then
restart the detector with `preview_only:=false`. The first feasible detection can
cause movement. Preview review does not resolve the known left hardware fault.

## Repository contents

Keep these complete source packages together under your new repository's `src/`:

```
aloha_follow_planner/
follower_left_moveit_config/    # include hidden .setup_assistant
follower_right_moveit_config/   # include hidden .setup_assistant
aloha_perception/              # ROS package name aloha_tracking_perception
```

Keep calibration/config files, docs, requirements and tests. Exclude build,
install, log, cache directories, nested .git directories and downloaded weights
unless intentionally distributed. ALOHA/Interbotix, MoveItPy, RealSense and their
lab-specific configuration remain external dependencies; preserve their versions
and mode/calibration settings separately. No Git repository or commit is created
by these changes.

## Earlier validation and remaining native issue

64 automated tests passed. The actual MoveIt pipeline also generated and scored
feasible paths for both arm models using synthetic rest feedback in isolated ROS
domains, without creating motor publishers or service clients. Both final checks
selected an A path with 110 small waypoints for (.362, .023, .300).

Those isolated runs still reproduced the previously observed native MoveIt
shutdown segmentation fault after path selection completed. That teardown issue
is unresolved; successful path selection is not a clean process-exit test. No
physical motion or motor-load comparison has been performed for this change.

## Command readback diagnostics (introduced in revision 5)

Executor revision 5 logs `COMMAND_READBACK` immediately before/after the first
command and at a waypoint timeout. These snapshots read Goal_Position,
Present_Position and Present_Current through the existing driver; they do not
change settings or open a second USB connection. Use these logs to distinguish
a goal that was not stored from one accepted but not reached.

## Encoder-resolution arrival checks

Revision 6 compares measured angles with the quantized XM actuator goal rather
than the unrepresentable ideal floating-point target. The conversion matches
Workbench float32 arithmetic and truncation for XM540-W270 (1120) and
XM430-W350 (1020); other model numbers are rejected before any command.
For example, the shoulder request -1.850049 rad encodes to 841 counts. A measured
828 counts is 0.01994 rad from that actuator goal (inside the unchanged 0.02-rad
tolerance), although it is 0.02141 rad from the ideal request. A 14-count residual
still fails. Health checks, timeout, bounds/collision checks and the deviation
limit remain active. Revision 8 logs each emitted setpoint and periodic tracking errors instead of
requiring or reporting intermediate settling.

## Detailed execution diagnostics (introduced in revision 7)

`EXEC_DIAG` JSON records identify the active stage. Revision 8 emits
STREAM_READY, SETPOINT_SENT, TRACKING_SAMPLE, TRACKING_PAUSED/RESUMED,
TRACKING_STALL, FINAL_SETTLING, EXECUTION_FAILED and EXECUTION_FINISHED events.
The earlier waypoint events applied to the removed stop-and-settle executor. TRACKING_SAMPLE records, at most once per second
by default (`diagnostic_period=1.0`), include all joint errors in radians/degrees,
blocking joints, worst error, settling resets, remaining time and feedback ages.
Motor health checks log their duration. Failure records retain the active
waypoint/stage and feedback received/accepted/rejected counters and last rejection
reason. No additional motor reads are performed for these new log events; the
existing first-command/timeout register snapshots remain. Diagnostic instrumentation itself does not change motor settings; revision 8
changes the execution policy as described above.

For a controlled test, stop the detector and use a single PoseStamped target
rather than the continuous detector launch. Save the planner output with `tee`
and keep bringup running until completion/failure is recorded. Inspect:

```bash
grep -E 'EXEC_DIAG|COMMAND_READBACK|ERROR' ~/right_waypoint_diagnostics.log
```

Final convergence allows 0.035 rad (about 2 degrees) per joint. A successful
completion releases the detector to submit its latest fresh moved target.
Following is serial: it completes the current path before replanning; hardware
faults and tracking-limit failures still stop following.


## Detector Ctrl+C and priority return to rest — revision 9

Keep hardware bringup and planner running. Press Ctrl+C once in the detector
launch terminal. The target router keeps its ROS context alive, stops target
publication, and calls `/follower_<arm>/return_to_rest` (std_srvs/Trigger).
The planner latches this request above tracking, cancels the tracking worker
before another motor publish, and waits for it to stop before starting rest.
No two workers publish arm commands concurrently.

The planner reads the driver's six joint sleep positions. The factory values
are outside the configured URDF/motor limits for this setup, so rest uses the
nearest legal folded target with a 0.015-rad margin inside both sets of limits.
It plans a fresh collision-checked joint-space path from measured feedback,
displays it in RViz, and executes it through the same monitored controller.
Gripper commands are unchanged. It acknowledges completion only after measured
arrival within 0.035 rad and a final health check.

The router waits for the matching request ID to report `completed` before
exiting. Both detector launches allow 320 seconds before escalating to SIGTERM
rather than killing this process after the standard five seconds. The rest
handshake itself is bounded to 300 seconds. Preview sends no rest commands.
Faults, lost feedback, unavailable planner/driver, collision or convergence
failure are explicitly reported; rest never overrides a latched hardware fault.
A second Ctrl+C stops waiting; it does not guarantee stopping the arm. To reuse
tracking after a rest request, restart the planner and detector. Stop them only
after `Rest reached and verified; detector exiting.` Then stop bringup.

This is a controlled rest procedure, not an emergency stop. The last motor goal
remains held if execution is interrupted; motor torque is not disabled.

The intermediate controller envelope now accommodates the recorded 0.0445-rad
shoulder lag that previously paused the second target and latched failure. Soft
pause is 0.06 rad, hard stop 0.08 rad, per-joint stall threshold 0.048 rad for 5 s.
Final destination tolerance stays 0.035 rad. A frozen shoulder is still rejected
in simulation; this change is not a repair or proof of correct load sharing.


## Revision 10 (historical, rest strategy replaced): staged rest and fixed tracking height

The recorded return-to-rest failure pulled the loaded shoulder while extended.
Rest now compares the direct route with collision-checked distal-first staging:
hold shoulder and waist while folding elbow/wrist, then plan the remaining fold.
Candidates are ranked first by peak and mean URDF gravity load **while the
shoulder moves**, then by total joint travel. An unavoidable high load at the
initial held pose no longer masks the benefit of unloading before retraction.
All original encoder limits, health checks, stall detection, and final arrival
checks remain active. This is a static model heuristic, not a torque guarantee.

The perception bridge already ignores detected height. The planner now also
forces every tracking IK request to Z=0.300 m in follower_<arm>/base_link,
including manually published poses. It logs when it replaces an incoming Z.
This is an absolute base-frame height, not camera depth and not object Z+0.300.
The tool goal is gripper_link; rest changes height as needed to fold the arm.
A fixed goal height alone cannot guarantee real-world path/fingertip clearance
when obstacles are not represented or the joints have tracking error.


## Revision 11: clearance-first rest (replaces distal-first rest)

Do not use revision 10's distal-first rest strategy. It could lower the fingers
onto the workspace while the shoulder stayed fixed. Revision 11 removes it and
has no unguarded direct folding fallback from a tracking pose.

Rest first solves a continuous Cartesian retreat: the gripper frame moves upward
and inward toward the base while the fingers turn level. IK is seeded from each
previous solution; branch jumps and inaccurate solutions are rejected. Only then
may the planner fold toward rest. Efficient checked interpolation is tried before
an OMPL alternative. Candidate selection prefers a shorter valid route; clearance
is a hard condition and shoulder load is secondary.

`rest_clearance.py` loads the actual STL collision meshes, including wrist,
gripper, bar, prop and fingers, and transforms conservative bounding boxes into
the arm base frame. The whole route is checked at <=0.005-rad joint spacing.
Protected geometry must remain >=0.10 m above the modeled table; tool geometry
outside a 0.22-m base cylinder remains >=0.20 m high. The same tool-height guard
also applies within 0.12 m of the tracked target/current extended gripper XY,
even if the target is near the base. A low initial measured pose may recover
monotonically upward; it is never permission to sweep farther downward.

The folded goal now stays 0.15 rad inside URDF and encoder limits, rather than
approaching the limits. If the goal or full path violates clearance, the planner
refuses rest **before sending any motion**. Fresh measured geometry is also
checked during execution, with a 0.03-m tracking allowance and a 0.003-m recovery
noise allowance; losing clearance stops further commands. Existing collision,
fault, feedback, torque, encoder, tracking and stall checks remain active.

To preview rest, run the planner with `execute_motion:=false` and the detector
with `preview_only:=false`, with bringup/RViz running. Detector Ctrl+C requests
the same rest plan from the current measured arm pose, displays it in RViz, and
reports `Rest preview displayed ... no hardware command sent`. It never claims
physical arrival in preview. Restart planner/detector with execution enabled for
a hardware trial. A robot already contacting the table or in modeled collision
is refused automatic rest; do not use this as a contact-recovery controller.

Tracking target height stays fixed at Z=0.300 m. Rest intentionally changes Z.
These are modeled clearances; mounting/calibration, unmodeled obstacles and motor
behavior still require physical verification. No software test guarantees a
collision-free physical move on an unvalidated robot setup.
