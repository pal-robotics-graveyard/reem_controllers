'''
Created on 30 Aug 2013

@author: Karsten Knese, Gennaro Raiola
'''
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.meta_task_velocity_damping import MetaTaskVelocityDamping
#from dynamic_graph.sot.core.meta_task_joint_weights import MetaTaskJointWeights

from sot_robot.prologue import robot, solver
from dynamic_graph.sot.dyninv import *

from utilities.sot import pop
from utilities.sot import push

def createJointLimitsTask(gain = 1, dt = 0.001):
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    taskJL = TaskJointLimits('jointLimits')
    plug(robot.dynamic.position, taskJL.position)
    taskJL.controlGain.value = gain
    taskJL.referenceInf.value = robot.dynamic.lowerJl.value
    taskJL.referenceSup.value = robot.dynamic.upperJl.value
    taskJL.dt.value = dt
    taskJL.selec.value = toFlags(range(6, robot.dimension))
    return taskJL

def createEqualityTask(taskName, jointName, gain = None):
    taskEq = MetaTaskKine6d(taskName, robot.dynamic, jointName, jointName)
    taskEq.feature.frame('desired')
    if (gain):
        taskEq.gain.setConstant(gain)
    return taskEq

def createInequalityTask(taskName, jointName, selectionMask='000111', positionVector=(0,0,0), referenceInf=(-100,-100,-100), referenceSup=(100,100,100)):
    taskIneq = MetaTaskIneqKine6d(taskName, robot.dynamic, jointName, jointName)
    taskIneq.feature.frame('desired')
    taskIneq.feature.selec.value = '111111'
    taskIneq.task.add(taskIneq.feature.name)
    taskIneq.task.referenceSup.value = referenceSup
    taskIneq.task.referenceInf.value = referenceInf
    taskIneq.task.selec.value = selectionMask
    taskIneq.task.dt.value = 0.001
    taskIneq.task.controlGain.value = 0.9
#     gotoNd(taskIneq, positionVector, '111')
    return taskIneq

def createVelocityDampingTask(taskName, jointName, collisionCenter, di, ds):
    taskVelDamp = MetaTaskVelocityDamping(taskName, robot.dynamic, jointName, jointName, collisionCenter, di, ds)
    return taskVelDamp
#     taskVelDamp = TaskVelocityDamping(taskName)
#     taskVelDamp.di.value = 0.2
#     taskVelDamp.ds.value = 0.1
#     taskVelDamp.dt.value = 0.001
#     taskVelDamp.controlGain.value = 1
#     
#     taskVelDamp.p2.value = matrixToTuple(goalP2)
#     robot.dynamic.Jarm_right_tool_joint.recompute(0)
#     robot.dynamic.arm_right_tool_joint.recompute(0)
#     plug(robot.dynamic.signal("arm_right_tool_joint"), taskVelDamp.p1)
#     plug(robot.dynamic.signal("Jarm_right_tool_joint"), taskVelDamp.jVel)


"""
def createWeightsTask(diag = None):
    taskWeights = MetaTaskJointWeights('jointWeights',robot,diag)
    return taskWeights
"""
def createGazeTask(jointName):
    taskGaze = MetaTaskVisualPoint('gaze',robot.dynamic,jointName,jointName)
    taskGaze.featureDes.xy.value = (0,0)
    return taskGaze

def createComEqTask(gain = 1):
    taskCom = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    taskCom.featureDes.errorIN.value = robot.dynamic.com.value
    taskCom.task.controlGain.value = gain
    return taskCom

def createComIneqTask(gain = 1, dt = 0.001, referenceInf = (-1,-1, 0), referenceSup = (1,1,0)):
    featureCom = FeatureGeneric('featureCom')
    plug(robot.dynamic.com,featureCom.errorIN)
    plug(robot.dynamic.Jcom,featureCom.jacobianIN)
    taskCom = TaskInequality('com')
    taskCom.add(featureCom.name)
    taskCom.selec.value = '011'
    taskCom.referenceInf.value = referenceInf
    taskCom.referenceSup.value = referenceSup
    taskCom.dt.value = dt
    robot.dynamic.com.recompute(0)
    taskCom.controlGain.value = gain
    return taskCom

