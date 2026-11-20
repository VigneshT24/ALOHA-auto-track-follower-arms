"""Conservative collision-mesh envelopes for the controlled return to rest."""
import itertools
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory

from .scoring import JOINTS, origin_matrix


def mesh_vertices(filename):
    if filename.startswith('package://'):
        package, relative = filename[len('package://'):].split('/', 1)
        filename = Path(get_package_share_directory(package))/relative
    data = Path(filename).read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) == 84+50*count:
            dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
            return np.frombuffer(data,dtype=dtype,count=count,offset=84)['vertices'].reshape(-1,3)
    # This robot uses STL. Unknown mesh formats must not silently lose guards.
    vertices = [line.split()[1:] for line in data.decode('ascii').splitlines()
                if line.strip().startswith('vertex ')]
    if not vertices:
        raise ValueError('Rest clearance requires supported STL collision meshes')
    return np.array(vertices,dtype=float)


class RestClearance:
    """AABB corners enclose full meshes, including fingertips and wrist.

    Outside the base retreat cylinder the tool must stay above the workspace
    clearance plane. A low initial measured pose may only recover monotonically
    upward; it is never permission to sweep downward from that initial pose.
    """
    floor = .10
    tool_floor = .20
    inner_radius = .22

    def __init__(self, xml, model, prefix, protected_xy=()):
        self.model, self.base = model, prefix+'/base_link'
        self.protected_xy = [np.array(point,dtype=float) for point in protected_xy]
        self.clouds = []
        for link in ET.fromstring(xml).findall('link'):
            name = link.get('name')
            if not name.startswith(prefix+'/') or not any(tag in name for tag in ('forearm','wrist','gripper','finger')):
                continue
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                mesh = geometry.find('mesh')
                if mesh is not None:
                    vertices = mesh_vertices(mesh.get('filename'))
                    vertices = vertices*np.array([float(v) for v in mesh.get('scale','1 1 1').split()])
                    lo,hi = vertices.min(axis=0),vertices.max(axis=0)
                elif geometry.find('box') is not None:
                    hi = np.array([float(v) for v in geometry.find('box').get('size').split()])/2
                    lo = -hi
                elif geometry.find('cylinder') is not None:
                    cylinder = geometry.find('cylinder')
                    hi = np.array([float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))/2])
                    lo = -hi
                elif geometry.find('sphere') is not None:
                    hi = np.full(3,float(geometry.find('sphere').get('radius')));lo=-hi
                else:
                    raise ValueError('Unsupported protected collision geometry')
                corners = np.array(list(itertools.product(*zip(lo,hi))))
                origin = origin_matrix(collision.find('origin'))
                corners = corners@origin[:3,:3].T+origin[:3,3]
                self.clouds.append((name,corners,any(tag in name for tag in ('wrist','gripper','finger'))))
        if not self.clouds or not any(tool for _,_,tool in self.clouds):
            raise ValueError('Missing end-effector collision geometry for rest clearance')

    def metrics(self, point, positions):
        transforms,_ = self.model.transforms(dict(positions,**dict(zip(JOINTS,point))))
        inverse = np.linalg.inv(transforms[self.base])
        minimum, outer_minimum, radius = math.inf, math.inf, 0.
        for name,corners,tool in self.clouds:
            transform = inverse@transforms[name]
            vertices = corners@transform[:3,:3].T+transform[:3,3]
            low = float(vertices[:,2].min())
            minimum = min(minimum,low)
            if tool:
                extent = float(np.linalg.norm(vertices[:,:2],axis=1).max())
                radius = max(radius,extent)
                lo,hi = vertices[:,:2].min(axis=0),vertices[:,:2].max(axis=0)
                near_target = any(float(np.linalg.norm(center-np.clip(center,lo,hi))) < .12
                                  for center in self.protected_xy)
                if extent > self.inner_radius or near_target:
                    outer_minimum = min(outer_minimum,low)
        return dict(minimum_z=minimum,outer_tool_z=outer_minimum,tool_radius=radius)

    def violations(self, point, positions, margin=0.):
        metrics = self.metrics(point,positions)
        return (max(0., self.floor-margin-metrics['minimum_z']),
                max(0., self.tool_floor-margin-metrics['outer_tool_z']))

    def violation(self, point, positions):
        return max(self.violations(point,positions))

    def valid_path(self, points, positions):
        if not points:
            return False
        best = self.violations(points[0],positions)
        for point in points:
            violation = self.violations(point,positions)
            if any(v > old+.001 for v,old in zip(violation,best)):
                return False
            best = tuple(min(a,b) for a,b in zip(best,violation))
        return True
