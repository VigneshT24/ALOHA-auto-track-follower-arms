"""Exercise actual process signals with a fake ROS backend (no network/motors)."""
import os
import signal
import subprocess
import sys
import time

import pytest


@pytest.mark.parametrize('arm',['left','right'])
def test_sigint_waits_for_rest_before_process_exit(arm,tmp_path):
    marker=tmp_path/'started'
    done=tmp_path/'rest_finished'
    code=f'''
import time
from pathlib import Path
from aloha_tracking_perception import target_runtime as runtime
runtime.rclpy.init=lambda **kwargs:None
runtime.rclpy.ok=lambda:True
runtime.rclpy.shutdown=lambda:None
class Executor:
    def add_node(self,node):pass
    def spin_once(self,**kwargs):time.sleep(.01)
    def shutdown(self,**kwargs):pass
class Target:
    preview_only=False
    def __init__(self,arm):Path({str(marker)!r}).write_text(arm)
    def return_to_rest_on_exit(self,abort):
        time.sleep(6.)
        assert not abort.is_set()
        Path({str(done)!r}).write_text("verified")
        return True
    def destroy_node(self):pass
runtime.SingleThreadedExecutor=Executor
runtime.TargetOnce=Target
runtime.main_for_arm({arm!r})
'''
    command=[sys.executable,'-c',code]
    process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    try:
        deadline=time.monotonic()+10.
        while not marker.exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.05)
        assert marker.exists()
        process.send_signal(signal.SIGINT)
        time.sleep(.05)
        process.send_signal(signal.SIGINT)  # Same keypress forwarded by launch.
        output,_=process.communicate(timeout=15.)
        assert process.returncode==0,output
        assert done.read_text()=='verified'
        assert "sending signal 'SIGTERM'" not in output
    finally:
        if process.poll() is None:
            process.kill();process.communicate()


def test_actual_detector_launches_allow_rest_before_signal_escalation():
    import ast
    from pathlib import Path
    base=Path(__file__).resolve().parents[1]/'launch'
    for filename,executable in [('truck_target_once.launch.py','target_once'),('truck_target_right.launch.py','right_target')]:
        tree=ast.parse((base/filename).read_text())
        nodes=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='Node']
        matches=[n for n in nodes if any(k.arg=='executable' and isinstance(k.value,ast.Constant) and k.value.value==executable for k in n.keywords)]
        assert len(matches)==1
        timeout=next(k.value.value for k in matches[0].keywords if k.arg=='sigterm_timeout')
        assert float(timeout)>=315.
