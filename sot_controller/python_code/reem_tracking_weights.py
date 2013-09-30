#!/usr/bin/env python
'''
@author: Gennaro Raiola, Karsten Knese
'''
from sot_ros_api import *

from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped, PointStamped

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
    taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]),10)

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
    taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
    taskLW = createEqualityTask('leftWrist', 'arm_left_tool_joint')
    taskBASE = createEqualityTask('baseContact','base_joint',1000)
    taskGAZE = createGazeTask('camera_joint')
    taskJL = createJointLimitsTask(1000, 0.001)
    weights_diag_flag = 1
    if (weights_diag_flag):
        diag = (0,0,0,0,0,0,3,3,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
    else:
        diag = None
    taskWEIGHTS = createWeightsTask(diag)
    
    push(taskJL)
    solver.addContact(taskBASE)
    push(taskRW)
    push(taskLW)
    push(taskGAZE)
    push(taskWEIGHTS)
    
    listener()
    
    rospy.spin()
