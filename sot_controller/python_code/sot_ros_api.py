# Create the reem robot model and a solver for the stack of tasks
from sot_robot.prologue import robot, solver

# Useful modules to interact with the stack of tasks
# They should be shared between the demos...
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY

# User defined modules
from utilities.kinematics import *
from utilities.sot import *
from utilities.tasks import *
from utilities.rosinterface import *

# Python modules
import time
import numpy

# Ros modules
import roslib; roslib.load_manifest('sot_controller')
import tf
import rospy
