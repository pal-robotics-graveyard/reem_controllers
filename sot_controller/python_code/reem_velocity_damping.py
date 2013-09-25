'''
@author: Karsten Knese
'''

from sot_ros_api import *

taskBASE = createEqualityTask('baseContact','base_joint',1000)
taskJL = createJointLimitsTask(1000, 0.001)

taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
gotoNd(taskRW, (0.2,-0.3,1.1), '111', 100)

goalP2 = goalDef((-0.2,-0.3,1.1))

taskVelDamp = createVelocityDampingTask('velDamp', 'arm_right_tool_joint', goalP2, 0.2, 0.1)

# taskVelDamp = TaskVelocityDamping('taskVelDamp')
# taskVelDamp.di.value = 0.2
# taskVelDamp.ds.value = 0.1
# taskVelDamp.dt.value = 0.001
# taskVelDamp.controlGain.value = 1
# 
# taskVelDamp.p2.value = matrixToTuple(goalP2)
# robot.dynamic.Jarm_right_tool_joint.recompute(0)
# robot.dynamic.arm_right_tool_joint.recompute(0)
# plug(robot.dynamic.signal("arm_right_tool_joint"), taskVelDamp.p1)
# plug(robot.dynamic.signal("Jarm_right_tool_joint"), taskVelDamp.jVel)

push(taskJL)
solver.addContact(taskBASE)
push(taskVelDamp)
push(taskRW)



