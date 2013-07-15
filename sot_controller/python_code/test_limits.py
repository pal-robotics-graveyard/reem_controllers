from startup import *

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

taskWT = MetaTaskKine6d('wt',robot.dynamic,'torso_base_joint','torso_base_joint')
taskWT.feature.frame('desired')
taskWT.gain.setConstant(1000)

push(taskJL)

solver.addContact(taskWT)

push(taskGAZE)

x = 0.0
y = -10
z = 0.77

taskGAZE.goto3D((x,y,z))







