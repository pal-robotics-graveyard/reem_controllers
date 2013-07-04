# Use the graph
from dynamic_graph import writeGraph
writeGraph ("/tmp/graph.dot")
dot -o graph.pdf -Tpdf graph.dot

from dynamic_graph import plug
plug(solver.sot.control, robot.device.control)

# Example taken from Thomas's email
solver.push(robot.tasks['com'])

push(robot.tasks['waist'])

push(robot.tasks['right-wrist'])

x0 = robot.features['right-wrist'].position.value[0][3]
y0 = robot.features['right-wrist'].position.value[1][3]
z0 = robot.features['right-wrist'].position.value[2][3]
print x0, y0, z0
dx = 0
dy = 0
dz = 0.3
x = x0 + dx
y = y0 + dy
z = z0 + dz

robot.tasks['right-wrist'].controlGain.value = 100

robot.features['right-wrist']._reference.signal('position').value = ((1, 0, 0, x), (0, 1, 0, y), (0, 0, 1, z), (0, 0, 0, 1))

robot.tasks['right-wrist'].controlGain.value = 10
robot.features['right-wrist']._reference.signal('position').value = ((1, 0, 0, 0.4), (0, 1, 0, -0.9), (0, 0, 1, 0.9), (0, 0, 0, 1))

robot.features['right-wrist']._reference.signal('position').value = ((1, 0, 0, 0.7), (0, 1, 0, -0.2), (0, 0, 1, 0.9), (0, 0, 0, 1))

solver.push(robot.tasks['left-wrist'])
robot.features['left-wrist']._reference.signal('position').value = ((1, 0, 0, 0.5), (0, 1, 0, 0.5), (0, 0, 1, 0.5), (0, 0, 0, 1))


# Example taken from p105_demo_romeo.py
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple, rotate, matrixToRPY
import numpy
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d, toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture

def push(task):
    if isinstance(task, str): taskName = task
    elif "task" in task.__dict__:  taskName = task.task.name
    else: taskName = task.name
    solver.push(taskName)

taskRW = MetaTaskKine6d('rw', robot.dynamic, 'right-wrist', 'right-wrist')
taskRW.feature.frame('desired')

robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskJL = TaskJointLimits('taskJL')
plug(robot.dynamic.position, taskJL.position)
taskJL.controlGain.value = 10
taskJL.referenceInf.value = robot.dynamic.lowerJl.value
taskJL.referenceSup.value = robot.dynamic.upperJl.value
taskJL.dt.value = 0.001
taskJL.selec.value = toFlags(range(6, 52))

taskWT = MetaTaskKine6d('wt', robot.dynamic, 'waist', 'waist')
taskWT.feature.frame('desired')
taskWT.gain.setConstant(100)

push(taskRW)

push(taskJL)

solver.addContact(taskWT)

goto6d(taskRW, (0.7, -0.3, 0.8), 100)

taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
taskCom.featureDes.errorIN.value = robot.dynamic.com.value
taskCom.task.controlGain.value = 10
push(taskCom)

jointLimitator = JointLimitator('joint_limitator')
plug(robot.dynamic.position, jointLimitator.joint)
plug(robot.dynamic.upperJl, jointLimitator.upperJl)
plug(robot.dynamic.lowerJl, jointLimitator.lowerJl)
plug(solver.control, jointLimitator.controlIN)
plug(jointLimitator.control, robot.device.control)
plug(robot.device.state, robot.dynamic.position)

taskJL.selec.value = [1] * robot.dimension
# taskJL.selec.value = toFlags(range(1,52)) # ????????????????


handMgrip = eye(4); handMgrip[0:3, 3] = (0.1, 0.1, 0.1)
taskRH.opmodif = matrixToTuple(handMgrip)

taskRH.feature.frame('desired')
gotoNd(taskRW, (0.5, 0.5, 0.5), '111', (4.9, 0.9, 0.01, 0.9))


taskRW.gain.setConstant(100)

for name, joint in [ ['LF', 'left-ankle'], ['RF', 'right-ankle' ] ]:
    contact = MetaTaskKine6d('_contact' + name, robot.dynamic, name, joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    locals()['contact' + name] = contact
    
solver.push(contactRF.task)
solver.push(contactLF.task)

# from dynamic_graph import plug
# plug(solver.sot.control,robot.device.control)
# handMgrip=numpy.eye(4); handMgrip[0:3,3] = (100,100,100)
# taskRW.opmodif = matrixToTuple(handMgrip)
# robot.tasks['right-wrist'] = taskRW
# solver.push(robot.tasks['right-wrist'])


