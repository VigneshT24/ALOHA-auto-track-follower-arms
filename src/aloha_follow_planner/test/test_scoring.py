import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from aloha_follow_planner.scoring import JOINTS, UrdfModel, ScoreWeights, expand_path, score_path


@pytest.fixture
def simple_model():
    # One 1-kg mass 1 metre from the shoulder; all other links are massless.
    root = ET.Element('robot', name='test')
    ET.SubElement(root, 'link', name='base')
    parent = 'base'
    for name in JOINTS:
        child = name+'_link'
        link = ET.SubElement(root, 'link', name=child)
        if name == 'shoulder':
            inertial = ET.SubElement(link, 'inertial')
            ET.SubElement(inertial, 'origin', xyz='1 0 0')
            ET.SubElement(inertial, 'mass', value='1')
        joint = ET.SubElement(root, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'axis', xyz='0 1 0')
        ET.SubElement(joint, 'limit', lower=str(-math.pi), upper=str(math.pi))
        parent = child
    return UrdfModel(ET.tostring(root, encoding='unicode'))


def test_static_gravity_known_lever_and_vertical_pose(simple_model):
    positions = dict.fromkeys(JOINTS, 0.)
    assert simple_model.gravity_loads(positions, 'wrist_rotate_link') == pytest.approx([9.80665, 0.])
    positions['shoulder'] = math.pi/2
    assert simple_model.gravity_loads(positions, 'wrist_rotate_link') == pytest.approx([0., 0.], abs=1e-10)


def test_payload_increases_gravity_moment(simple_model):
    # Move the tip to the mass COM to create a known extra payload moment.
    simple_model.joints['wrist_rotate']['origin'][0, 3] = 1.
    positions = dict.fromkeys(JOINTS, 0.)
    assert simple_model.gravity_loads(positions, 'wrist_rotate_link', 2.) == pytest.approx([29.41995, 19.6133])


def test_expansion_preserves_vertices_and_does_not_wrap_bounded_joints():
    a, b = [0.]*6, [0.]*6
    a[3], b[3] = -3., 3.
    points = expand_path([a, b, a], .04)
    assert points[0] == a and points[-1] == a and b in points
    assert max(abs(y-x) for p, q in zip(points, points[1:]) for x, y in zip(p, q)) <= .040000001
    assert sum(abs(q[3]-p[3]) for p, q in zip(points, points[1:])) == pytest.approx(12.)


def test_full_path_cost_penalizes_detour_even_with_same_endpoint(simple_model):
    zero = [0.]*6
    goal = [.1]*6
    detour = [1.]*6
    w = ScoreWeights(shoulder_load=0., elbow_load=0.)
    direct = score_path([zero, goal], simple_model, {}, 'wrist_rotate_link', w)
    indirect = score_path([zero, detour, goal], simple_model, {}, 'wrist_rotate_link', w)
    assert direct['score'] < indirect['score']


def test_weighting_prioritizes_shoulder_travel(simple_model):
    zero, shoulder, wrist = [0.]*6, [0.]*6, [0.]*6
    shoulder[1], wrist[5] = 1., 1.
    w = ScoreWeights(excursion=0., margin=0., shoulder_load=0., elbow_load=0.)
    scores = [score_path([zero, p], simple_model, {}, 'wrist_rotate_link', w)['score'] for p in (shoulder, wrist)]
    assert scores[0] > scores[1]


def test_initial_bounds_connector_must_be_small_and_monotone(simple_model):
    outside = [0.]*6
    outside[1] = math.pi+.04
    bounded = simple_model.bounded_start(outside)
    assert bounded[1] == math.pi
    assert simple_model.valid_start_connection(expand_path([outside, bounded, [0.]*6], .02))
    further = outside.copy(); further[1] += .01
    assert not simple_model.valid_start_connection([outside, further, bounded])
    assert not simple_model.valid_start_connection([[0.]*6, outside])
    outside[1] = math.pi+.1
    with pytest.raises(ValueError):
        simple_model.bounded_start(outside)


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_gravity_fk_matches_installed_moveit_model(arm, tmp_path):
    import xacro
    from ament_index_python.packages import get_package_share_directory
    from moveit.core.robot_model import RobotModel
    from moveit.core.robot_state import RobotState
    source = get_package_share_directory('interbotix_xsarm_descriptions')+'/urdf/aloha_vx300s.urdf.xacro'
    xml = xacro.process_file(source, mappings=dict(robot_name='follower_'+arm,
        use_world_frame='true', hardware_type='actual', show_ar_tag='false',
        show_gripper_bar='true', show_gripper_fingers='true')).toxml()
    urdf = tmp_path/'robot.urdf'; urdf.write_text(xml)
    srdf = get_package_share_directory('follower_'+arm+'_moveit_config')+'/config/aloha_vx300s.srdf'
    model = RobotModel(str(urdf), srdf)
    state = RobotState(model)
    state.set_to_default_values()
    state.joint_positions = dict(zip(JOINTS, [.1, -.5, .5, .3, -.6, .2]))
    state.update()
    parsed = UrdfModel(xml)
    transforms, _ = parsed.transforms(state.joint_positions)
    for name, matrix in transforms.items():
        assert np.allclose(matrix, state.get_global_link_transform(name), atol=1e-8), name
    rest = [0., -1.8714566, 1.61835, .055, -1.90827, .0414]
    assert parsed.valid_start_connection([rest, parsed.bounded_start(rest)])


def test_limit_penalty_distinguishes_goal_despite_marginal_start(simple_model):
    start = [0.]*6; start[1] = -math.pi-.04
    comfortable, boundary = [0.]*6, [0.]*6
    boundary[3] = math.pi
    weights = ScoreWeights(travel=(0.,)*6, excursion=0., shoulder_load=0., elbow_load=0.)
    a = score_path(expand_path([start, comfortable], .02), simple_model, {}, 'wrist_rotate_link', weights)
    b = score_path(expand_path([start, boundary], .02), simple_model, {}, 'wrist_rotate_link', weights)
    assert a['components']['limit_margin'] < b['components']['limit_margin']
