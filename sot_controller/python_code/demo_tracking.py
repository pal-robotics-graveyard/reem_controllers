#!/usr/bin/env python
from startup import *

from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped

def callback(data):
    quat = numpy.array([data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z , data.pose.orientation.w])
    xyz = numpy.array([data.pose.position.x , data.pose.position.y , data.pose.position.z])
    moveTarget(xyz,quat)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("right_hand_ref_pose_filt",PoseStamped, callback)

def moveTarget(xyz,quat):
    goal_rw = goalDef(xyz,quat)
    taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]))
    gotoNd(taskRW,(xyz[0],xyz[1],xyz[2]),'111111',10)

if __name__ == '__main__':
    taskRW = MetaTaskKine6d('rw',robot.dynamic,'arm_right_tool_joint','arm_right_tool_joint')
    taskRW.feature.frame('current')
    taskGAZE = MetaTaskVisualPoint('gz',robot.dynamic,'camera_joint','camera_joint')
    taskGAZE.gain.setConstant(1000)
    taskGAZE.featureDes.xy.value = (0,0)
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    taskJL = TaskJointLimits('taskJL')
    plug(robot.dynamic.position,taskJL.position)
    taskJL.controlGain.value = 1000
    taskJL.referenceInf.value = robot.dynamic.lowerJl.value
    taskJL.referenceSup.value = robot.dynamic.upperJl.value
    taskJL.dt.value = 0.001
    taskJL.selec.value = toFlags(range(6,robot.dimension))
    taskWT = MetaTaskKine6d('wt',robot.dynamic,'base_joint','base_joint')
    taskWT.feature.frame('desired')
    taskWT.gain.setConstant(1000)
    push(taskJL)
    solver.addContact(taskWT)
    push(taskRW)
    push(taskGAZE)
    
    listener()
    
    rospy.spin()
