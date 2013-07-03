from dynamic_graph.sot.reem.startup import *
import roslib; roslib.load_manifest('sot_controller')
sys.argv = "example_orientation"
import tf
import rospy

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

filename = "/tmp/out_from_py.txt"
out_file = open(filename,"w")

jointLimits_flag = 1
contact_waist_flag = 1
gaze_flag = 1

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

if jointLimits_flag:
    push(taskJL)

taskWT = MetaTaskKine6d('wt',robot.dynamic,'waist','waist')
taskWT.feature.frame('desired')
taskWT.gain.setConstant(1000)
taskWT.feature.position.value

if contact_waist_flag:
    solver.addContact(taskWT)

time.sleep(5)

if gaze_flag:
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

time.sleep(15)

count = 0
for elem in robot.device.state.value:
    u = robot.dynamic.upperJl.value[count]
    l = robot.dynamic.lowerJl.value[count]
    count = count + 1
    if (elem < l):
        #print ("Rank: " + str(count) +  " state value: " + str(elem) + " lower bound: " + str(l))
        out_file.write("Rank: " + str(count) +  " state value: " + str(elem) + " lower bound: " + str(l) + "\n")
    if (elem > u):
        #print ("Rank: " + str(count) +  " state value: " + str(elem) + " upper bound: " + str(u))
        out_file.write("Rank: " + str(count) +  " state value: " + str(elem) + " upper bound: " + str(u) + "\n")

if gaze_flag:
    out_file.write("Error taskGAZE\n")
    err = [ [ 0 for i in range(4) ] for j in range(4) ]
    for i in range(0,4):
        for j in range(0,4):
            err[i][j] = taskGAZE.featureDes.position.value[i][j] - taskGAZE.feature.position.value[i][j]
        out_file.write(str(err[i])+"\n")

if contact_waist_flag:
    out_file.write("Error taskWT\n")
    err = [ [ 0 for i in range(4) ] for j in range(4) ]
    for i in range(0,4):
        for j in range(0,4):
            err[i][j] = taskWT.featureDes.position.value[i][j] - taskWT.feature.position.value[i][j]
        out_file.write(str(err[i])+"\n")

out_file.close()





