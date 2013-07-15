# Create the reem robot model for the stack of tasks
from sot_reem.prologue import robot, solver

# Useful modules to interact with the stack of tasks
# They should be shared between the examples...
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint

# User defined modules
from utilities.kinematics import *
from utilities.sot import *

# Python modules
import time
import numpy

# Ros modules
import roslib; roslib.load_manifest('sot_controller')
import tf
import rospy