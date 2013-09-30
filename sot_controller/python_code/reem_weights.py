'''
@author: Gennaro Raiola, Karsten Knese
'''
from sot_ros_api import *

rospy.init_node('tf_reem')
listener = tf.TransformListener()
def transformation(frame_1,frame_2):
    now = rospy.Time.now()
    listener.waitForTransform(frame_1,frame_2, now, rospy.Duration(4.0))
    (xyz,quat) = listener.lookupTransform(frame_1, frame_2, now)
    return {'quat':quat,'xyz':xyz}

xyz = numpy.array([0,-100,1.483])

taskBASE = createEqualityTask('baseContact','base_joint',1000)
taskGAZE = createGazeTask('camera_joint')
taskJL = createJointLimitsTask(1000, 0.001)

weights_diag_flag = 1
if (weights_diag_flag):
    diag = (0,0,0,0,0,0,1.36499,1.49997,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
else:
    diag = None

taskWEIGHTS = createWeightsTask(diag)

push(taskJL)
solver.addContact(taskBASE)
push(taskGAZE)
push(taskWEIGHTS)

taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]),1000)

camera_base = transformation("/head_1_link","/base_link")
camera_base_rpy = mat2rpy(quat2mat(camera_base["quat"]))
camera_base_dg = camera_base_rpy * 180/numpy.pi

camera_torso = transformation("/head_1_link","/torso_1_link")
camera_torso_rpy = mat2rpy(quat2mat(camera_torso["quat"]))
camera_torso_dg = camera_torso_rpy * 180/numpy.pi

torso_base = transformation("/torso_1_link","/base_link")
torso_base_rpy = mat2rpy(quat2mat(torso_base["quat"]))
torso_base_dg = torso_base_rpy * 180/numpy.pi

print camera_base_dg
print camera_torso_dg
print torso_base_dg