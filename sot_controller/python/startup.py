from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.reem.prologue import robot, solver
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint

import numpy
import time
import math

def push(task):
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    solver.push(taskName)
    
def mat2rpy(m):
    rpy = numpy.zeros(3)
    rpy[0] = math.atan2(m[1,0],m[0,0])
    rpy[1] = math.atan2(-m[2,0],math.sqrt(m[2,1]**2+m[2,2]**2))
    rpy[2] = math.atan2(m[2,1],m[2,2])
    return rpy

def mat2quat(m):
    quat = numpy.zeros(4)
    quat[3] = 0.5 * math.sqrt(m[0,0]+m[1,1]+m[2,2]+1)
    quat[0] = 0.5 * numpy.sign(m[2,1]-m[1,2])*math.sqrt(m[0,0]-m[1,1]-m[2,2]+1)
    quat[1] = 0.5 * numpy.sign(m[0,2]-m[2,0])*math.sqrt(m[1,1]-m[2,2]-m[0,0]+1)
    quat[2] = 0.5 * numpy.sign(m[1,0]-m[0,1])*math.sqrt(m[2,2]-m[0,0]-m[1,1]+1)
    return quat

def quat2mat(q):
    w = q[3]
    x = q[0]
    y = q[1]
    z = q[2]
    m = numpy.matrix( [[ 2*(w**2+x**2)-1 , 2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),2*(w**2+y**2)-1,2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),2*(w**2+z**2)-1]])
    return m

def rotx(angle):
    m = numpy.matrix([[1,0,0],[0,numpy.cos(angle),-numpy.sin(angle)],[0,numpy.sin(angle),numpy.cos(angle)]])
    return m

def roty(angle):
    m = numpy.matrix([[numpy.cos(angle),0,numpy.sin(angle)],[0,1,0],[-numpy.sin(angle),0,numpy.cos(angle)]])
    return m

def rotz(angle):
    m = numpy.matrix([[numpy.cos(angle),-numpy.sin(angle),0],[numpy.sin(angle),numpy.cos(angle),0],[0,0,1]])
    return m
