from sot_ros_api import *

taskBASE = createEqualityTask('baseContact','fixed',1000)
taskCAM = createEqualityTask('camera', 'camera_joint')
gotoNd(taskCAM, (0.5,0.2,3),'111',100)
createRosImport('matrixHomo', taskCAM.feature.position, 'rviz_marker_collisioncheck')

goalP2 = goalDef((0.6,0.2, 2))
taskVelDamp = createVelocityDampingTask('velDamp', 'camera_joint', goalP2, 0.4, 0.2)
taskVelDamp.task.controlGain.value = 10

solver.addContact(taskBASE)
push(taskVelDamp)
push(taskCAM)