'''
Created on 30 Aug 2013

@author: Karsten Knese, Gennaro Raiola
'''
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import *
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.meta_task_velocity_damping import MetaTaskVelocityDamping
from dynamic_graph.sot.core.meta_task_joint_weights import MetaTaskJointWeights

from sot_robot.prologue import robot, solver
from dynamic_graph.sot.dyninv import *

from utilities.sot import pop
from utilities.sot import push

class MetaTaskIneqKine6d(MetaTaskKine6d):
    def createTask(self):
        self.task = TaskInequality('inequalitytask'+self.name)
        
    def createFeatures(self):
        self.feature    = FeaturePoint6d('ineqfeature'+self.name)
        self.featureDes = FeaturePoint6d('ineqfeature'+self.name+'_ref')
        self.feature.selec.value = '111111'
        self.feature.frame('current')

def createJointLimitsTask(gain = 1, dt = None):
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    taskJL = TaskJointLimits('jointLimits')
    plug(robot.dynamic.position, taskJL.position)
    """
    The internal gain for the joint limits task is defined as 1/gain, i.e. is used to limit the maximum reachable 
    velocity in a single time interval (dt).
    A high value can be used to limit oscillations around the goal point but it slows down the motion.
    """
    taskJL.controlGain.value = gain
    taskJL.referenceInf.value = robot.dynamic.lowerJl.value
    taskJL.referenceSup.value = robot.dynamic.upperJl.value
    taskJL.selec.value = toFlags(range(6, robot.dimension))
    if(dt):
        taskJL.dt.value = dt
    else:
        plug(robot.device.dt,taskJL.dt)
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
    gotoNd(taskIneq, positionVector, '111')
    taskIneq.feature.selec.value = '111111'
#     taskIneq.task.add(taskIneq.feature.name)
    taskIneq.task.referenceSup.value = referenceSup
    taskIneq.task.referenceInf.value = referenceInf
    taskIneq.task.selec.value = selectionMask
    taskIneq.task.dt.value = 0.001
    taskIneq.task.controlGain.value = 0.9
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


def createWeightsTask(diag = None, gain = 1000, sampleInterval = 0, selec=None):
    selec = toFlags(range(0,robot.dimension))
    taskWeights = MetaTaskJointWeights('jointWeights',selec,robot,diag,gain,sampleInterval)
    return taskWeights

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

def gotoNdComp(task,position,selec=None,gain=None,resetJacobian=True,comp=[[0,1,0],[0,0,1],[1,0,0]]):
    '''
    gotoNdComp takes care of the different frame orientations used in jrl-dynamics. 
    Be careful about the comp matrix, it is specific for reem and reemc (and could be different among
    different joint frames)
    '''
    M = generic6dReference(position.copy())
    R = M[0:3,0:3]
    R = R*comp
    M[0:3,0:3] = R
    if selec!=None:
        if isinstance(selec,str):   task.feature.selec.value = selec
        else: task.feature.selec.value = toFlags(selec)
    task.featureDes.position.value = matrixToTuple(M)
    setGain(task.gain,gain)
    if 'resetJacobianDerivative' in task.task.__class__.__dict__.keys() and resetJacobian:
        task.task.resetJacobianDerivative()