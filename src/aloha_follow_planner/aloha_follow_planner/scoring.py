"""Deterministic path scoring and URDF static gravity estimates (no ROS I/O)."""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

JOINTS = ('waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate')


def xyz(text):
    return np.array([float(x) for x in text.split()])


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross @ cross)


def origin_matrix(element):
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, 3] = xyz(element.get('xyz', '0 0 0'))
        roll, pitch, yaw = xyz(element.get('rpy', '0 0 0'))
        matrix[:3, :3] = (rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch)
                            @ rotation([1, 0, 0], roll))
    return matrix


class UrdfModel:
    """FK, bounds and total joint gravity moment; not per-servo/dynamic torque."""
    def __init__(self, xml):
        root = ET.fromstring(xml)
        self.links = {}
        for link in root.findall('link'):
            inertial = link.find('inertial')
            mass, com = 0., np.zeros(3)
            if inertial is not None:
                mass = float(inertial.find('mass').get('value'))
                com = origin_matrix(inertial.find('origin'))[:3, 3]
            self.links[link.get('name')] = (mass, com)
        self.joints, self.limits = {}, {}
        children = set()
        for joint in root.findall('joint'):
            name = joint.get('name')
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            axis = joint.find('axis')
            mimic = joint.find('mimic')
            self.joints[name] = dict(
                parent=parent, child=child, kind=joint.get('type'),
                origin=origin_matrix(joint.find('origin')),
                axis=xyz(axis.get('xyz', '1 0 0') if axis is not None else '1 0 0'),
                mimic=None if mimic is None else (mimic.get('joint'),
                    float(mimic.get('multiplier', '1')), float(mimic.get('offset', '0'))))
            children.add(child)
            limit = joint.find('limit')
            if joint.get('type') in ('revolute', 'prismatic') and limit is not None:
                self.limits[name] = (float(limit.get('lower')), float(limit.get('upper')))
        self.roots = set(self.links)-children
        self.descendants = {}
        for name, joint in self.joints.items():
            downstream = {joint['child']}
            changed = True
            while changed:
                count = len(downstream)
                downstream.update(j['child'] for j in self.joints.values() if j['parent'] in downstream)
                changed = count != len(downstream)
            self.descendants[name] = downstream
        if any(name not in self.limits for name in JOINTS):
            raise ValueError('URDF must provide bounded positions for all six arm joints')

    def in_bounds(self, positions):
        return len(positions) == 6 and all(math.isfinite(q) and self.limits[n][0] <= q <= self.limits[n][1]
                                          for n, q in zip(JOINTS, positions))

    def bounded_start(self, positions, tolerance=.05):
        if len(positions) != 6 or not all(math.isfinite(q) for q in positions):
            raise ValueError('Invalid measured positions')
        projected = [min(max(q, self.limits[n][0]), self.limits[n][1]) for n, q in zip(JOINTS, positions)]
        if max(abs(a-b) for a, b in zip(positions, projected)) > tolerance:
            raise ValueError('Measured start exceeds URDF bounds tolerance; inspect calibration')
        return projected

    def valid_start_connection(self, points, tolerance=.05):
        """Only the initial out-of-bounds prefix may converge monotonically into limits."""
        if not points:
            return False
        try:
            bounded = self.bounded_start(points[0], tolerance)
        except ValueError:
            return False
        previous = [abs(a-b) for a, b in zip(points[0], bounded)]
        for point in points:
            if len(point) != 6 or not all(math.isfinite(q) for q in point):
                return False
            violation = [max(lo-q, q-hi, 0.) for q, (lo, hi) in zip(point, (self.limits[n] for n in JOINTS))]
            if any(v > old+1e-9 for v, old in zip(violation, previous)):
                return False
            previous = violation
        return True

    def transforms(self, positions):
        transforms = {name: np.eye(4) for name in self.roots}
        frames = {}
        pending = dict(self.joints)
        for _ in range(len(pending)+1):
            for name, joint in list(pending.items()):
                if joint['parent'] not in transforms:
                    continue
                frame = transforms[joint['parent']] @ joint['origin']
                frames[name] = frame
                value = positions.get(name, 0.)
                if joint['mimic']:
                    source, multiplier, offset = joint['mimic']
                    value = positions.get(source, 0.)*multiplier+offset
                motion = np.eye(4)
                if joint['kind'] in ('revolute', 'continuous'):
                    motion[:3, :3] = rotation(joint['axis'], value)
                elif joint['kind'] == 'prismatic':
                    motion[:3, 3] = joint['axis']*value
                transforms[joint['child']] = frame @ motion
                del pending[name]
            if not pending:
                return transforms, frames
        raise ValueError('URDF is not a connected acyclic tree')

    def gravity_loads(self, positions, tip, payload_mass=0.):
        transforms, frames = self.transforms(positions)
        loads = []
        for name in ('shoulder', 'elbow'):
            pivot = frames[name][:3, 3]
            axis = frames[name][:3, :3] @ self.joints[name]['axis']
            axis = axis / np.linalg.norm(axis)
            moment = np.zeros(3)
            for link in self.descendants[name]:
                mass, com = self.links[link]
                center = transforms[link][:3, :3] @ com + transforms[link][:3, 3]
                moment += np.cross(center-pivot, [0., 0., -9.80665*mass])
            if payload_mass:
                moment += np.cross(transforms[tip][:3, 3]-pivot, [0., 0., -9.80665*payload_mass])
            loads.append(abs(float(moment @ axis)))
        return loads


def expand_path(points, max_step):
    """Preserve every planned vertex; subdivide with bounded, unwrapped angles."""
    if not points or max_step <= 0 or not math.isfinite(max_step):
        raise ValueError('Require nonempty path and positive finite step')
    if any(len(p) != 6 or not all(math.isfinite(q) for q in p) for p in points):
        raise ValueError('Require finite six-joint waypoints')
    expanded = [list(points[0])]
    for goal in points[1:]:
        start = expanded[-1]
        distance = max(abs(b-a) for a, b in zip(start, goal))
        if distance < 1e-10:
            continue
        count = max(1, math.ceil(distance/max_step))
        expanded.extend([[a+(b-a)*i/count for a, b in zip(start, goal)] for i in range(1, count+1)])
    return expanded


@dataclass
class ScoreWeights:
    travel: tuple = (1., 3., 2., 1.5, 1.5, 1.5)
    excursion: float = 2.
    margin: float = 2.
    shoulder_load: float = 3.
    elbow_load: float = 1.5


def score_path(points, model, other_joints, tip, weights, payload_mass=0., start_bounds_tolerance=.05):
    if not model.valid_start_connection(points, start_bounds_tolerance):
        raise ValueError('Path violates URDF bounds')
    travel = [sum(abs(b[j]-a[j]) for a, b in zip(points, points[1:])) for j in range(6)]
    excursion = max(abs(q[j]-points[0][j]) for q in points for j in range(6))
    margins, loads = [], []
    for point in points:
        margins.append(min(min(q-model.limits[n][0], model.limits[n][1]-q) /
                           (model.limits[n][1]-model.limits[n][0]) for n, q in zip(JOINTS, point)))
        positions = dict(other_joints)
        positions.update(zip(JOINTS, point))
        loads.append(model.gravity_loads(positions, tip, payload_mass))
    peak = [max(row[j] for row in loads) for j in range(2)]
    # Penalty increases within the outer 10% of a joint's allowed range.
    penalties = [max(0., (0.1-margin)/0.1)**2 for margin in margins]
    # Average exposure plus the endpoint penalty prevents an already marginal
    # folded start from hiding a candidate that ends against another limit.
    margin_penalty = sum(penalties)/len(penalties) + penalties[-1]
    components = dict(weighted_travel=sum(w*t for w, t in zip(weights.travel, travel)),
                      excursion=weights.excursion*excursion,
                      limit_margin=weights.margin*margin_penalty,
                      shoulder_gravity=weights.shoulder_load*peak[0],
                      elbow_gravity=weights.elbow_load*peak[1])
    return dict(score=sum(components.values()), components=components,
                joint_travel_deg=[math.degrees(x) for x in travel],
                max_excursion_deg=math.degrees(excursion), min_limit_margin_fraction=min(margins),
                goal_limit_margin_fraction=margins[-1],
                peak_gravity_nm=peak)
