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

from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph.ros import RosRobotModel

class Reem(AbstractHumanoidRobot):
    """
    This class instanciates a Reem robot.
    """

    #OperationalPoints = ['right-wrist','left-wrist','waist','right-ankle','left-ankle']
    OperationalPoints = ['right-wrist','left-wrist','waist','gaze']

    tracedSignals = {
        'dynamic': ["com", "zmp", "position", "velocity", "acceleration"],
        'device': ['zmp', 'control', 'state']
        }

    def __init__(self, name, device = None, tracer = None, halfSitting = None):
        AbstractHumanoidRobot.__init__ (self, name, tracer)
        self.device = device
        self.dynamic = RosRobotModel("{0}_dynamic".format(name))
        self.dynamic.loadFromParameterServer()

        self.dimension = self.dynamic.getDimension()
        if halfSitting:
        		self.halfSitting = halfSitting
        else:
        		self.halfSitting = (0.,) * self.dimension #???
        print(self.halfSitting)
        self.initializeRobot()

__all__ = ["Reem"]
