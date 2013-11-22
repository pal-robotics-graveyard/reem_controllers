'''
Created on 30 Aug 2013

@author: Karsten Knese
'''
print 'importing rosinterface'
from dynamic_graph.ros import *
from sot_robot.prologue import robot
from dynamic_graph import plug
# import roslib; roslib.load_manifest("interactive_markers")
# import rospy

# http://wiki.ros.org/dynamic_graph_bridge

exportcounter = 0
importcounter = 0

# Export from ROS
def createRosExport(type, exportSignal, topic):
    # get global ros instance  
    global exportcounter
    exportcounter += 1
    rosExportSignalIntern = 'rosExportIntern'+str(exportcounter)
    ros.rosExport.add(type, rosExportSignalIntern, topic)
    # cannot be initialized empty due to serialization errors
    if (type == 'Vector'):
        ros.rosExport.rosExportSignalIntern = (100000,)
    plug(ros.rosExport.signal(rosExportSignalIntern), exportSignal)

# Import in ROS
def createRosImport(type, importSignal, topic):
    global importcounter
    importcounter += 1
    rosImportSignalIntern = 'rosImportIntern'+str(importcounter)
    ros.rosImport.add(type, str(rosImportSignalIntern), topic)
    plug(importSignal, ros.rosImport.signal(str(rosImportSignalIntern)))

ros = Ros(robot)

# createRosImport('vector', taskRightArmBoundary.referenceInf, '/rviz_marker_listener_sot_inf')



