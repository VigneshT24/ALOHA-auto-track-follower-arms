#!/usr/bin/env python3
"""One supported left-arm truck trial; read registers through the existing SDK.

Run after left_tracking_bringup.launch.py, with the ROS workspaces sourced.
Default is planning-only; --execute authorizes one physical target.
No direct serial access, register writes, fault resets, or automatic rest.
"""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import RegisterValues
import yaml

from aloha_tracking_perception.target_once import select_point

MOTORS = dict(waist=1, shoulder=2, shoulder_shadow=3, elbow=4,
              elbow_shadow=5, forearm_roll=6, wrist_angle=7, wrist_rotate=8, gripper=9)
STATIC = ('Hardware_Error_Status', 'Torque_Enable', 'Operating_Mode', 'Drive_Mode',
          'Homing_Offset', 'Model_Number', 'Position_P_Gain', 'Position_I_Gain',
          'Position_D_Gain', 'Profile_Velocity', 'Profile_Acceleration',
          'Current_Limit', 'Min_Position_Limit', 'Max_Position_Limit',
          'Goal_Position', 'Present_Position', 'Present_Current')
LIVE = ('Hardware_Error_Status', 'Torque_Enable', 'Goal_Position',
        'Present_Position', 'Present_Current')


def ros_environment(environment=None):
    """env-python shebangs must resolve to Humble's system Python, not Conda."""
    result = dict(os.environ if environment is None else environment)
    result['PATH'] = '/usr/bin:/bin:/opt/ros/humble/bin:'+result.get('PATH','')
    result.pop('PYTHONHOME',None)
    return result


def validate_preflight(registers):
    for name in MOTORS:
        values = registers[name]
        if values['Hardware_Error_Status'] != 0:
            raise RuntimeError(f'{name} ID {MOTORS[name]} already faulted: {values["Hardware_Error_Status"]}; NO target sent')
        if values['Torque_Enable'] != 1 or values['Operating_Mode'] != 3:
            raise RuntimeError(f'{name}: require torque already enabled and position mode 3; NO settings changed')
        # Match the planner's existing 0.05-rad folded-start allowance (~32
        # encoder counts). Do not wrap/relabel a large encoder-reference error.
        if 'Present_Position' in values:
            lo,hi = values['Min_Position_Limit'],values['Max_Position_Limit']
            measured = values['Present_Position']
            if not lo-32 <= measured <= hi+32:
                raise RuntimeError(f'{name} ID {MOTORS[name]} measured {measured} counts outside '
                    f'[{lo}, {hi}] plus the 32-count startup allowance; correct physical pose/reference; NO target sent')
    for name, drive in (('shoulder', 1), ('shoulder_shadow', 0)):
        if registers[name]['Drive_Mode'] != drive:
            raise RuntimeError(f'{name}: unexpected Drive_Mode; NO target sent')


def make_target(point, stamp):
    x, y, z, detected_stamp = point
    if not all(math.isfinite(v) for v in point):
        raise ValueError('Nonfinite truck coordinates')
    message = PoseStamped()
    message.header.frame_id = 'follower_left/base_link'
    message.header.stamp = stamp
    message.pose.position.x, message.pose.position.y, message.pose.position.z = float(x), float(y), .300
    message.pose.orientation.y = message.pose.orientation.w = 2**-.5
    return message


class Recorder(Node):
    def __init__(self, folder):
        super().__init__('left_supported_trial')
        self.file = (folder/'motor_readout.jsonl').open('w', buffering=1)
        self.joints_file = (folder/'joint_states.jsonl').open('w', buffering=1)
        self.status, self.latest, self.joint_receipt = None, None, None
        self.client = self.create_client(RegisterValues, '/follower_left/get_motor_registers')
        self.publisher = self.create_publisher(PoseStamped, '/grasp_pose', 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/arm_follow/status', self.on_status, qos)
        self.create_subscription(String, '/aloha/perception/detections_base_filtered', self.on_detection, 1)
        self.create_subscription(JointState, '/follower_left/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(JointGroupCommand, '/follower_left/commands/joint_group',
            lambda msg:self.write('command', name=msg.name, radians=list(msg.cmd)), 100)

    def write(self, event, **fields):
        self.file.write(json.dumps(dict(event=event, monotonic=time.monotonic(),
            ros_time=self.get_clock().now().nanoseconds/1e9, **fields))+'\n')

    def on_status(self, message):
        self.status = json.loads(message.data)
        self.write('planner_status', status=self.status)

    def on_detection(self, message):
        self.latest = message.data

    def on_joints(self, message):
        self.joint_receipt = time.monotonic()
        self.joints_file.write(json.dumps(dict(received=self.joint_receipt,
            stamp=[message.header.stamp.sec,message.header.stamp.nanosec],
            names=list(message.name),positions=list(message.position),
            velocities=list(message.velocity),effort=list(message.effort)))+'\n')

    def read(self, name, register):
        future = self.client.call_async(RegisterValues.Request(cmd_type='single', name=name, reg=register))
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.)
        if not future.done():
            self.client.remove_pending_request(future)
            raise RuntimeError(f'Register timeout: {name}/{register}')
        result = future.result()
        if result is None or len(result.values) != 1:
            raise RuntimeError(f'Invalid register response: {name}/{register}')
        return int(result.values[0])

    def snapshot(self, names, registers, stage):
        result = {}
        for name in names:
            values = {}
            for register in registers:
                values[register] = self.read(name,register)
                self.write('register', stage=stage, motor=name, id=MOTORS[name], register=register, value=values[register])
            result[name] = values
        self.write('snapshot', stage=stage, motors=result)
        return result


def stop(process):
    if process is None or process.poll() is not None:
        return
    try:os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:return
    try:
        process.wait(timeout=5.)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:process.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL);process.wait()


def run(args):
    if not args.model.is_file():raise RuntimeError('YOLO model file is missing')
    folder = args.output/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    folder.mkdir(parents=True)
    print(f'RESULTS: {folder}', flush=True)
    rclpy.init()
    node = Recorder(folder)
    planner = detector = None
    handles = []
    try:
        if not node.client.wait_for_service(timeout_sec=10.):
            raise RuntimeError('Left XS driver unavailable. Start LEFT bringup first; do not start a second driver.')
        for _ in range(10):rclpy.spin_once(node, timeout_sec=.1)
        if node.get_publishers_info_by_topic('/follower_left/commands/joint_group'):
            raise RuntimeError('Another left command publisher exists. Stop the old planner before this trial.')
        if any(info.node_name != node.get_name() for info in node.get_publishers_info_by_topic('/grasp_pose')):
            raise RuntimeError('Another left target publisher exists. Stop the old detector/target publisher first.')
        baseline = node.snapshot(MOTORS, STATIC, 'preflight')
        (folder/'preflight.json').write_text(json.dumps(baseline,indent=2))
        validate_preflight(baseline)
        config = yaml.safe_load((Path(get_package_share_directory('aloha_follow_planner'))/'config/path_scoring.yaml').read_text())
        config['/**']['ros__parameters']['tracking_speed'] = .04
        config_file = folder/'test_parameters.yaml';config_file.write_text(yaml.safe_dump(config))
        def launch(command, logfile):
            handle = (folder/logfile).open('w');handles.append(handle)
            return subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,
                start_new_session=True,env=ros_environment())
        planner = launch(['ros2','launch','follower_left_moveit_config','run_planner.launch.py',
            'execute_motion:='+str(args.execute).lower(),'scoring_config:='+str(config_file)],'planner.log')
        deadline = time.monotonic()+30.
        while time.monotonic()<deadline:
            if planner.poll() is not None:raise RuntimeError('Planner exited; see planner.log')
            rclpy.spin_once(node,timeout_sec=.1)
            if node.status and node.status.get('state')=='ready' and node.status.get('execute_motion') is args.execute:break
        else:raise RuntimeError('Planner did not become ready')
        # Perception is preview-only: only this recorder can send a motion target.
        detector = launch(['ros2','launch','aloha_tracking_perception','truck_target_once.launch.py',
            'model:='+str(args.model),'preview_only:=true'],'detector.log')
        deadline = time.monotonic()+30.
        point = None
        while time.monotonic()<deadline:
            if detector.poll() is not None:raise RuntimeError('Detector exited; see detector.log')
            rclpy.spin_once(node,timeout_sec=.1)
            try:point=select_point(node.latest,'follower_left/base_link',node.get_clock().now().nanoseconds/1e9,1.)
            except (ValueError,TypeError):continue
            break
        if point is None:raise RuntimeError('No single fresh truck/car/train target; NO motion sent')
        # Recheck health immediately before the sole target publication.
        health = node.snapshot(MOTORS, ('Hardware_Error_Status','Torque_Enable','Operating_Mode','Drive_Mode',
            'Present_Position','Min_Position_Limit','Max_Position_Limit'), 'before_target')
        validate_preflight(health)
        point=select_point(node.latest,'follower_left/base_link',node.get_clock().now().nanoseconds/1e9,1.)
        if node.joint_receipt is None or time.monotonic()-node.joint_receipt>.5:
            raise RuntimeError('Joint feedback is stale; NO target sent')
        owners=node.get_publishers_info_by_topic('/follower_left/commands/joint_group')
        if len(owners)!=1 or owners[0].node_name!='left_grasp_subscriber':
            raise RuntimeError('Unexpected or multiple left command publishers; NO target sent')
        message=make_target(point,node.get_clock().now().to_msg())
        stamp=[message.header.stamp.sec,message.header.stamp.nanosec]
        node.write('one_target',xyz=[point[0],point[1],.300],execute=args.execute,nominal_speed_rad_s=.04)
        print(f'ONE TARGET: left base ({point[0]:.3f}, {point[1]:.3f}, 0.300) m; execute={args.execute}',flush=True)
        node.publisher.publish(message)
        deadline=time.monotonic()+260.
        while time.monotonic()<deadline:
            pair=node.snapshot(('shoulder','shoulder_shadow'),LIVE,'during_trial')
            print('ID 2:',pair['shoulder'],'ID 3:',pair['shoulder_shadow'],flush=True)
            if any(values['Hardware_Error_Status'] or values['Torque_Enable']!=1 for values in pair.values()):
                node.write('hardware_fault',motors=pair);stop(planner);break
            if planner.poll() is not None:raise RuntimeError('Planner exited during trial; see planner.log')
            if node.status and node.status.get('goal_stamp')==stamp and node.status.get('state') in ('ready','rejected','failed'):
                node.write('trial_result',status=node.status);print('RESULT:',node.status,flush=True);break
            until=time.monotonic()+.4
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        else:raise RuntimeError('Trial timed out; stopping planner')
        stop(planner)
        for _ in range(3):node.snapshot(MOTORS,LIVE,'after_trial')
    except BaseException as exc:
        node.write('aborted',reason=str(exc))
        print(f'STOPPED: {exc}',flush=True)
        raise
    finally:
        stop(planner);stop(detector)  # Detector is preview-only: no rest motion.
        for handle in handles:handle.close()
        print(f'Recordings saved in {folder}. Bringup is left running; no fault reset or torque-off was sent.',flush=True)
        node.file.close();node.joints_file.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',help='Enable one physical move; otherwise planner preview only')
    parser.add_argument('--model',type=Path,default=Path.home()/'interbotix_ws/src/aloha_tracking_ws/yolov8m.pt')
    parser.add_argument('--output',type=Path,default=Path.home()/'left_supported_tests')
    run(parser.parse_args())
