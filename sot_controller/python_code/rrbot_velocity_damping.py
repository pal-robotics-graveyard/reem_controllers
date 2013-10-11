from sot_ros_api import *

taskBASE = createEqualityTask('baseContact','fixed',1000)
taskCAM = createEqualityTask('camera', 'hokuyo_joint')
gotoNd(taskCAM, (0.5,0.2,3),'111',10)
createRosImport('matrixHomo', taskCAM.feature.position, 'rviz_marker_collisioncheck')

goalP2 = goalDef((0.6,0.2, 2))
taskVelDamp = createVelocityDampingTask('velDamp', 'hokuyo_joint', goalP2, 0.3, 0.2)
taskVelDamp.task.controlGain.value = 1

solver.addContact(taskBASE)
push(taskVelDamp)
push(taskCAM)

# working solution 
# >>> taskVelDamp.task.controlGain.value = 1
# >>> gotoNd(taskCAM, (0.8,0.2,1),'111',1)
# >>> gotoNd(taskCAM, (0.6,0.2,2.5),'111',1)
