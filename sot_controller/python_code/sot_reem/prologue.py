# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of dynamic-graph.
# dynamic-graph is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# dynamic-graph is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

print("sot_reem")
print("Compiled for robot REEM.")

from dynamic_graph import plug
from robot import Reem
from dynamic_graph.entity import PyEntityFactoryClass
#from dynamic_graph.sot.dynamics.solver import Solver

# Create the OpenHRP enabled device.
# This entity behaves exactly like robotsimu except:
# 1. it does not provide the increment method
# 2. it forwards the robot control to the OpenHRP
#    controller.
Device = PyEntityFactoryClass('SotReemDevice')

robot = Reem(name = 'robot', device = Device('robot_device'))

# FIXME: this must be set so that the graph can be evaluated.
robot.device.zmp.value = (0., 0., 0.)

# Create a solver.
from dynamic_graph.sot.dyninv import SolverKine
def toList(solver):
    return map(lambda x: x[1:-1],solver.dispStack().split('|')[1:])
SolverKine.toList = toList
solver = SolverKine('sot')
solver.setSize(robot.dimension)
robot.device.control.unplug()
plug(solver.control,robot.device.control)
plug(robot.device.state,robot.dynamic.position)

#solver = Solver(robot)

print("Prologue ran successfully.")

# Make sure only robot and solver are visible from the outside.
__all__ = ["robot", "solver"]

####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
