from startup import *

rospy.init_node('tf_reem')
listener = tf.TransformListener()

def transformation(frame_1,frame_2):
    now = rospy.Time.now()
    listener.waitForTransform(frame_1,frame_2, now, rospy.Duration(4.0))
    (xyz,quat) = listener.lookupTransform(frame_1, frame_2, now)
    return {'quat':quat,'xyz':xyz}

def goalDef(frame_1,frame_2,xyz,quat):
    R = quat2mat(quat)
    goal = numpy.matrix([[0 , 0, 0, xyz[0]], [0,  0, 0, xyz[1]], [0, 0, 0, xyz[2]], [0, 0, 0, 1]])
    cur_pose = transformation(frame_1,frame_2)
    R_comp = quat2mat(cur_pose['quat'])
    goal_r = R * R_comp.transpose()
    goal[0:3,0:3] = goal_r
    return goal

def visDef(frame_1,frame_2,xyz):
    cur_pose = transformation(frame_1,frame_2)
    cur_xyz = cur_pose['xyz'] 
    R_comp = quat2mat(cur_pose['quat'])
    pos = numpy.array([(xyz - cur_xyz)]).T
    axis = R_comp * pos
    theta = math.atan2(axis[1],axis[0])
    alpha = math.atan2(axis[2],axis[1])
    Rz = rotz(-theta)
    Rx = rotx(-alpha)
    goal = numpy.matrix([[0 , 0, 0, cur_xyz[0]], [0,  0, 0, cur_xyz[1]], [0, 0, 0, cur_xyz[2]], [0, 0, 0, 1]])
    goal_r = Rz * Rx
    goal_r = goal_r * R_comp.transpose()
    goal[0:3,0:3] = goal_r.transpose()
    return goal

quat = numpy.array([-0.377,-0.06,-0.142,0.91])
xyz = numpy.array([0.4,-0.384,1.25])

taskGAZE = MetaTaskKine6d('gz',robot.dynamic,'gaze','gaze')
taskGAZE.feature.frame('current')

robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskJL = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskJL.position)
taskJL.controlGain.value = 1000
taskJL.referenceInf.value = robot.dynamic.lowerJl.value
taskJL.referenceSup.value = robot.dynamic.upperJl.value
taskJL.dt.value = 0.001
taskJL.selec.value = toFlags(range(6,robot.dimension))

taskWT = MetaTaskKine6d('wt',robot.dynamic,'waist','waist')
taskWT.feature.frame('desired')
taskWT.gain.setConstant(1000)
taskWT.feature.position.value

push(taskJL)

solver.addContact(taskWT)

push(taskGAZE)

x0 = taskGAZE.feature.position.value[0][3]
y0 = taskGAZE.feature.position.value[1][3]
z0 = taskGAZE.feature.position.value[2][3]
dx = 0.0
dy = -10
dz = 0.0
xyz = numpy.array([x0+dx,y0+dy,z0+dz])
goal_gz = visDef("/torso_base_link","/head_2_link",xyz)
gotoNd(taskGAZE,goal_gz,'111000',1)







