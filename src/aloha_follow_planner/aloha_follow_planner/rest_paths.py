"""Upward/inward rest waypoints and secondary load estimates; no hardware I/O."""
import math
import numpy as np
from .scoring import JOINTS, rotation


def shoulder_load(point, model, positions, tip):
    return model.gravity_loads(dict(positions, **dict(zip(JOINTS, point))), tip, 0.)[0]


def moving_shoulder_load(points, model, positions, tip):
    """Rank load while the shoulder moves, rather than unavoidable initial load."""
    loads = []
    for a,b in zip(points,points[1:]):
        if abs(b[1]-a[1]) > 1e-6:
            loads.extend((shoulder_load(a,model,positions,tip),
                          shoulder_load(b,model,positions,tip)))
    return (max(loads,default=0.), sum(loads)/max(1,len(loads)))


def matrix_quaternion(matrix):
    """Return XYZW, including rotations near pi (no optional TF dependency)."""
    m = np.asarray(matrix)[:3,:3]
    trace = float(np.trace(m))
    if trace > 0:
        s = math.sqrt(trace+1.)*2
        q = [(m[2,1]-m[1,2])/s,(m[0,2]-m[2,0])/s,(m[1,0]-m[0,1])/s,.25*s]
    else:
        i = int(np.argmax(np.diag(m)));j=(i+1)%3;k=(i+2)%3
        s = math.sqrt(max(0.,1.+m[i,i]-m[j,j]-m[k,k]))*2
        q = [0.,0.,0.,0.]
        q[i]=.25*s;q[j]=(m[j,i]+m[i,j])/s;q[k]=(m[k,i]+m[i,k])/s
        q[3]=(m[k,j]-m[j,k])/s
    q = np.asarray(q,dtype=float)
    return q/np.linalg.norm(q)


def quaternion_matrix(q):
    x,y,z,w = np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def retreat_matrices(start, base, radius, height):
    """Continuous upward/inward TCP retreat while turning the fingers level."""
    relative = np.linalg.inv(base)@start
    yaw = math.atan2(relative[1,3],relative[0,3])
    finish = np.eye(4)
    finish[:3,3] = (radius*math.cos(yaw),radius*math.sin(yaw),height)
    finish[:3,:3] = rotation([0,0,1],yaw)
    finish = base@finish
    a,b = matrix_quaternion(start),matrix_quaternion(finish)
    dot = float(a@b)
    if dot < 0:b=-b;dot=-dot
    dot = min(1.,max(-1.,dot));angle=math.acos(dot)
    count = max(1,math.ceil(float(np.linalg.norm(finish[:3,3]-start[:3,3]))/.01),
                math.ceil(2*angle/.08))
    result=[]
    for index in range(1,count+1):
        f=index/count
        q = ((1-f)*a+f*b if dot>.9995 else
             (math.sin((1-f)*angle)*a+math.sin(f*angle)*b)/math.sin(angle))
        matrix=np.eye(4);matrix[:3,:3]=quaternion_matrix(q)
        matrix[:3,3]=(1-f)*start[:3,3]+f*finish[:3,3]
        result.append(matrix)
    return result
