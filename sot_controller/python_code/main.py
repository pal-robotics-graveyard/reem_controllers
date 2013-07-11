from startup import *

def goalDef(xyz,quat):
    goal = numpy.matrix([[0 , 0, 0, xyz[0]], [0,  0, 0, xyz[1]], [0, 0, 0, xyz[2]], [0, 0, 0, 1]])
    goal_r = quat2mat(quat)
    goal[0:3,0:3] = goal_r
    return goal

jointLimits_flag = 1
contact_waist_flag = 1
gaze_flag = 1
rw_flag = 1
lw_flag = 0
com_eq_flag = 0

quat = numpy.array([-0.377,-0.06,-0.142,0.91])
xyz = numpy.array([0.35,-0.3,1.25])

goal_rw = goalDef(xyz,quat)
goal_lw = goalDef(xyz,quat)

taskRW = MetaTaskKine6d('rw',robot.dynamic,'right-wrist','right-wrist')
taskRW.feature.frame('current')

taskLW = MetaTaskKine6d('lw',robot.dynamic,'left-wrist','left-wrist')
taskLW.feature.frame('current')

taskGAZE = MetaTaskVisualPoint('gz',robot.dynamic,'gaze','gaze')
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

taskWT = MetaTaskKine6d('wt',robot.dynamic,'waist','waist')
taskWT.feature.frame('desired')
taskWT.gain.setConstant(1000)

if com_eq_flag:
    taskCOM = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    taskCOM.featureDes.errorIN.value = robot.dynamic.com.value
    taskCOM.task.controlGain.value = 10 
else:
    featureCOM = FeatureGeneric('featureCom')
    plug(robot.dynamic.com,featureCOM.errorIN)
    plug(robot.dynamic.Jcom,featureCOM.jacobianIN)
    taskCOM = TaskInequality('com')
    taskCOM.add(featureCOM.name)
    taskCOM.selec.value = '011'
    taskCOM.referenceInf.value = (-0.23,-0.24,0)
    taskCOM.referenceSup.value = (0.1,0.24,0)
    taskCOM.dt.value = 0.001
    robot.dynamic.com.recompute(0)
    taskCOM.controlGain.value = 10

if jointLimits_flag:
    push(taskJL)

if contact_waist_flag:
    solver.addContact(taskWT)

if rw_flag:
    push(taskRW)

if lw_flag:
    push(taskLW)

if gaze_flag:
    push(taskGAZE)

push(taskCOM)

if gaze_flag:
    taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]))

if rw_flag:
    gotoNd(taskRW,goal_rw,'111111',100)

if lw_flag:
    gotoNd(taskLW,goal_lw,'111111',100)

time.sleep(15)

err2file(taskRW,filename,"w")
err2file(taskLW,filename,"a")
err2file(taskGAZE,filename,"a") 
  
#Unused code:
"""      
out_file = open("/tmp/joints_limits","w")
count = 0
for elem in robot.device.state.value:
    u = robot.dynamic.upperJl.value[count]
    l = robot.dynamic.lowerJl.value[count]
    count = count + 1
    out_file.write("Rank: " + str(count) +  " state value: " + str(elem) + " lower bound: " + str(l) + "\n")
    out_file.write("Rank: " + str(count) +  " state value: " + str(elem) + " upper bound: " + str(u) + "\n")  

out_file.close()     
"""





