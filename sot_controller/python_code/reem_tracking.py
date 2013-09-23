#!/usr/bin/env python
'''
@author: Gennaro Raiola, Karsten Knese
'''
from sot_ros_api import *

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
    taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]),10)
    gotoNd(taskRW,(xyz[0],xyz[1],xyz[2]),'111111',10)

if __name__ == '__main__':
    taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
    taskBASE = createEqualityTask('baseContact','base_joint',1000)
    taskGAZE = createGazeTask('camera_joint')
    taskJL = createJointLimitsTask(1000, 0.001)
    
    push(taskJL)
    solver.addContact(taskBASE)
    push(taskRW)
    push(taskGAZE)
    
    listener()
    
    rospy.spin()
