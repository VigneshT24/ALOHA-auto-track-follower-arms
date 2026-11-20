"""Regression against recorded ground-seeking rest, using actual collision meshes."""
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock

import numpy as np
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from aloha_follow_planner.scoring import JOINTS,UrdfModel,expand_path
from aloha_follow_planner.rest_clearance import RestClearance
from aloha_follow_planner.actuator_units import quantized_target
from test_selection import selector


@pytest.fixture(params=['left','right'])
def rest_model(request,tmp_path):
    arm=request.param;prefix='follower_'+arm
    source=get_package_share_directory('interbotix_xsarm_descriptions')+'/urdf/aloha_vx300s.urdf.xacro'
    xml=xacro.process_file(source,mappings=dict(robot_name=prefix,use_world_frame='true',hardware_type='actual',
        show_ar_tag='false',show_gripper_bar='true',show_gripper_fingers='true')).toxml()
    file=tmp_path/'robot.urdf';file.write_text(xml)
    model=RobotModel(str(file),get_package_share_directory(prefix+'_moveit_config')+'/config/aloha_vx300s.srdf')
    scene=PlanningScene(model)
    table=CollisionObject();table.header.frame_id=model.model_frame
    table.id='table_surface';table.operation=CollisionObject.ADD
    table.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[2.,2.,.05])]
    pose=Pose();pose.orientation.w=1.;pose.position.z=-.025;table.primitive_poses=[pose]
    scene.apply_collision_object(table)
    n,_=selector();n.arm=arm;n.prefix=prefix;n.tip=prefix+'/gripper_link'
    n.urdf=UrdfModel(xml);n.options['robot_description']=xml
    n.rest_clearance=RestClearance(xml,n.urdf,prefix)
    n.motor_limits=list(zip([0,841,898,0,830,0],[4095,2867,3094,4095,3504,4095]))
    n.info_client=Mock();n.check_driver=Mock()
    n.service_call=Mock(return_value=S(joint_names=list(JOINTS),joint_sleep_positions=[0.,-2.05,1.7,0.,-2.,0.]))
    state=RobotState(model);state.set_to_default_values()
    def check(points,positions):
        if not n.urdf.valid_start_connection(points):return False
        if not n.rest_clearance.valid_path(expand_path(points,.005),positions):return False
        for q in points:
            state.joint_positions=dict(positions,**dict(zip(JOINTS,q)));state.update()
            if scene.is_state_colliding(state,'interbotix_arm',False):return False
        return True
    n.check_path=check
    lower=np.array([n.urdf.limits[name][0] for name in JOINTS])+1e-6
    upper=np.array([n.urdf.limits[name][1] for name in JOINTS])-1e-6
    class NumericalState:
        # Offline IK for the *test*; production uses the configured MoveIt solver.
        def __init__(self,positions):self.q=np.array([positions[name] for name in JOINTS])
        def get_joint_group_positions(self,group):return self.q.tolist()
        def set_from_ik(self,group,pose,tip,timeout):
            target=np.array([pose.position.x,pose.position.y,pose.position.z])
            quat=pose.orientation;rotation=Rotation.from_quat([quat.x,quat.y,quat.z,quat.w]).as_matrix()
            def error(q):
                t=n.urdf.transforms(dict(zip(JOINTS,q)))[0][tip]
                return np.r_[t[:3,3]-target,.3*Rotation.from_matrix(rotation.T@t[:3,:3]).as_rotvec()]
            result=least_squares(error,np.clip(self.q,lower,upper),bounds=(lower,upper),max_nfev=100,
                ftol=1e-7,xtol=1e-7,gtol=1e-7)
            self.q=result.x
            return np.linalg.norm(error(self.q))<.001
    n.make_state=lambda positions:NumericalState(positions)
    return n


def test_recorded_ground_seeking_segment_is_rejected_before_motion(rest_model):
    n=rest_model
    initial=[.4893399,.34054375,-.2086214,.00153398,1.4894954,.4862719]
    bad=[.4878059,.3390098,.4062168,.001534,.0611035,.4862719]
    assert n.rest_clearance.metrics(bad,{})['minimum_z']<.04
    assert not n.check_path(expand_path([initial,bad],.005),{})


def test_new_rest_retreats_up_and_in_before_any_low_fold(rest_model):
    n=rest_model
    initial=[.4893399,.34054375,-.2086214,.00153398,1.4894954,.4862719]
    positions=dict(zip(JOINTS,initial))
    path=n.plan_rest(positions)
    assert n.check_path(path,positions)
    assert n.urdf.in_bounds(path[-1])
    counts,_=quantized_target(path[-1])
    assert all(lo<=value<=hi for value,(lo,hi) in zip(counts,n.motor_limits))
    transforms=n.urdf.transforms(positions)[0]
    starting_tool=transforms[n.tip][:3,3]
    first=[]
    for q in path:
        m=n.rest_clearance.metrics(q,positions)
        first.append(m)
        if m['tool_radius']<n.rest_clearance.inner_radius:break
        tool=n.urdf.transforms(dict(positions,**dict(zip(JOINTS,q))))[0][n.tip][:3,3]
        assert tool[2]>=starting_tool[2]-.003
    assert first[-1]['tool_radius']<n.rest_clearance.inner_radius
    # Once initial measured clearance is recovered, it may never be lost.
    assert n.rest_clearance.valid_path(path,positions)
    assert n.rest_clearance.metrics(path[-1],positions)['minimum_z']>=.12
    from aloha_follow_planner.tracking import JointPath
    assert JointPath(path,.08,.1).duration<69.08  # Previous recorded unsafe route.


def test_no_ik_retreat_has_no_unguarded_folding_fallback(rest_model):
    n=rest_model;n.make_state=lambda positions:Mock(set_from_ik=Mock(return_value=False))
    n.arm_pub=Mock()
    initial=[.4893399,.34054375,-.2086214,.00153398,1.4894954,.4862719]
    with pytest.raises(ValueError,match='No upward/inward rest path'):
        n.plan_rest(dict(zip(JOINTS,initial)))
    n.arm_pub.publish.assert_not_called()


def test_already_folded_arm_only_needs_short_checked_adjustment(rest_model):
    n=rest_model
    initial=[0.,-1.87146,1.61375,0.,-1.90674,0.]
    positions=dict(zip(JOINTS,initial))
    path=n.plan_rest(positions)
    assert n.check_path(path,positions)
    from aloha_follow_planner.tracking import JointPath
    assert JointPath(path,.08,.1).duration<8.
