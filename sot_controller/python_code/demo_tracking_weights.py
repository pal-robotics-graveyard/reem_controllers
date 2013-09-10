#!/usr/bin/env python
from startup import *

from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped, PointStamped
from dynamic_graph.sot.core.meta_task_joint_weights import MetaTaskJointWeights


def callback_right_hand(data):
    quat = numpy.array([data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z , data.pose.orientation.w])
    xyz = numpy.array([data.pose.position.x , data.pose.position.y , data.pose.position.z])
    move_target_right(xyz,quat)

def callback_left_hand(data):
    quat = numpy.array([data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z , data.pose.orientation.w])
    xyz = numpy.array([data.pose.position.x , data.pose.position.y , data.pose.position.z])
    move_target_left(xyz,quat)

def callback_head(data):
    xyz = numpy.array([data.point.x , data.point.y , data.point.z])
    taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]))

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("right_hand_ref_pose_filt",PoseStamped, callback_right_hand)
    rospy.Subscriber("left_hand_ref_pose_filt",PoseStamped, callback_left_hand)
    rospy.Subscriber("head_ref_point_filt",PointStamped, callback_head)

def move_target_right(xyz,quat):
    goal = goalDef(xyz,quat)
    gotoNd(taskRW,goal,'111111',10)

def move_target_left(xyz,quat):
    goal = goalDef(xyz,quat)
    gotoNd(taskLW,(xyz[0],xyz[1],xyz[2]),'111',10)

if __name__ == '__main__':
    taskRW = MetaTaskKine6d('rw',robot.dynamic,'arm_right_tool_joint','arm_right_tool_joint')
    taskRW.feature.frame('current')
    taskLW = MetaTaskKine6d('lw',robot.dynamic,'arm_left_tool_joint','arm_left_tool_joint')
    taskLW.feature.frame('current')
    taskGAZE = MetaTaskVisualPoint('gz',robot.dynamic,'camera_joint','camera_joint')
    taskGAZE.gain.setConstant(1000)
    taskGAZE.featureDes.xy.value = (0,0)
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    taskJL = TaskJointLimits('taskJL')
    plug(robot.dynamic.position,taskJL.position)
    taskJL.controlGain.value = 1
    taskJL.referenceInf.value = robot.dynamic.lowerJl.value
    taskJL.referenceSup.value = robot.dynamic.upperJl.value
    taskJL.dt.value = 0.001
    taskJL.selec.value = toFlags(range(6,robot.dimension))
    taskWT = MetaTaskKine6d('wt',robot.dynamic,'base_joint','base_joint')
    taskWT.feature.frame('desired')
    taskWT.gain.setConstant(1000)
    weights_diag_flag = 1
    if (weights_diag_flag):
        diag = (0,0,0,0,0,0,3,3,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
    else:
        diag = None
    taskWEIGHTS = MetaTaskJointWeights('weights',robot,diag)
    push(taskJL)
    solver.addContact(taskWT)
    push(taskRW)
    push(taskLW)
    push(taskGAZE)
    push(taskWEIGHTS)
    
    listener()
    
    rospy.spin()
