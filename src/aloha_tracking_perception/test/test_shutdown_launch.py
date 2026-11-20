"""Real launch/SIGINT/ROS handshake on isolated test topics, without hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid
import socket

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_launch_sigint_waits_beyond_default_kill_deadline_for_rest(arm,tmp_path):
    try:
        transport=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        transport.close()
    except PermissionError:
        pytest.skip('Full ROS/DDS integration requires UDP sockets; sandbox denies them. Signal/handshake unit tests still run.')
    root='/shutdown_test_'+uuid.uuid4().hex
    context=Context()
    rclpy.init(context=context)
    server=rclpy.create_node('test_rest_'+uuid.uuid4().hex,context=context)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher=server.create_publisher(String,root+'/status',qos)
    requested=[]
    def request(req,response):
        requested.append(time.monotonic())
        response.success=True
        response.message=json.dumps(dict(request_id='integration'))
        return response
    service=server.create_service(Trigger,root+'/rest',request)
    def status():
        state='idle' if not requested else ('completed' if time.monotonic()-requested[0]>=6. else 'moving')
        publisher.publish(String(data=json.dumps(dict(state='ready',goal_stamp=None,
            rest_id='integration',rest_state=state))))
    timer=server.create_timer(.1,status)
    executor=SingleThreadedExecutor(context=context);executor.add_node(server)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    code=f'from aloha_tracking_perception.target_runtime import main_for_arm; main_for_arm({arm!r})'
    status_topic='/arm_follow/status' if arm=='left' else '/follower_right/arm_follow/status'
    command=[sys.executable,'-c',code,'--ros-args','-p','preview_only:=false',
        '-p','output_topic:='+root+'/target','-p','input_topic:='+root+'/detections',
        '-r',f'/follower_{arm}/return_to_rest:='+root+'/rest',
        '-r',status_topic+':='+root+'/status']
    # Use the same graceful-shutdown deadlines as the real detector launches.
    launch_file=tmp_path/'shutdown_test.launch.py'
    launch_file.write_text('from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n'
        f'def generate_launch_description():\n return LaunchDescription([ExecuteProcess(cmd={command!r},output="screen",sigterm_timeout="320",sigkill_timeout="5")])\n')
    process=subprocess.Popen(['ros2','launch',str(launch_file)],stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,text=True,env=os.environ.copy())
    try:
        deadline=time.monotonic()+15.
        while server.count_subscribers(root+'/status')==0 and process.poll() is None and time.monotonic()<deadline:
            time.sleep(.1)
        assert server.count_subscribers(root+'/status')>0,'target did not discover isolated status publisher'
        started=time.monotonic()
        process.send_signal(signal.SIGINT)
        try:
            output,_=process.communicate(timeout=22.)
        except subprocess.TimeoutExpired as exc:
            pytest.fail('Detector launch did not finish: '+str(exc.output))
        assert process.returncode==0,output
        assert len(requested)==1,output
        assert time.monotonic()-started>=6.,output
        assert 'Rest reached and verified; detector exiting.' in output,output
        assert 'sending signal \'SIGTERM\'' not in output,output
    finally:
        if process.poll() is None:
            process.kill();process.communicate()
        executor.shutdown(timeout_sec=2.)
        thread.join(timeout=2.)
        server.destroy_node();context.shutdown()
