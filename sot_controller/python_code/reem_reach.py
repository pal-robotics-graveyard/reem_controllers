'''
@author: Gennaro Raiola, Karsten Knese
'''
from sot_ros_api import *

filename = "/tmp/out_from_py.txt"

gaze_flag = 1
rw_flag = 1
lw_flag = 0
com_eq_flag = 0

quat = numpy.array([-0.377,-0.06,-0.142,0.91])
xyz = numpy.array([0.35,-0.3,1.25])

goal_rw = goalDef(xyz,quat)
goal_lw = goalDef(xyz,quat)

taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
taskLW = createEqualityTask('leftWrist', 'arm_left_tool_joint')
taskBASE = createEqualityTask('baseContact','base_joint',1000)
taskGAZE = createGazeTask('camera_joint')
taskJL = createJointLimitsTask(1000, 0.001)

if (com_eq_flag):
    taskCOM = createComEqTask(1000)
else:
    taskCOM = createComIneqTask(1000,0.001, referenceInf = (-0.23,-0.24,0), referenceSup = (0.1,0.24,0))

push(taskJL)
solver.addContact(taskBASE)

if rw_flag:
    push(taskRW)

if lw_flag:
    push(taskLW)

if gaze_flag:
    push(taskGAZE)

push(taskCOM)

if gaze_flag:
    taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]),100)

if rw_flag:
    gotoNd(taskRW,goal_rw,'111111',100)

if lw_flag:
    gotoNd(taskLW,goal_lw,'111111',100)

time.sleep(15)

err2file(taskRW,filename,"w")
err2file(taskLW,filename,"a")
err2file(taskGAZE,filename,"a")
  
out_file = open("/tmp/joints_limits","w")
count = 0
for elem in robot.device.state.value:
    u = robot.dynamic.upperJl.value[count]
    l = robot.dynamic.lowerJl.value[count]
    count = count + 1
    out_file.write("Rank: " + str(count) +  " state value: " + str(elem) + " lower bound: " + str(l) + "\n")
    out_file.write("Rank: " + str(count) +  " state value: " + str(elem) + " upper bound: " + str(u) + "\n")  

out_file.close()

