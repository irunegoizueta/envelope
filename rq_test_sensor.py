#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$
#
# Author: Irune Goizueta Zubimendi

import rospy
from std_msgs.msg import String
from robotiq_force_torque_sensor.msg import ft_sensor
from robotiq_force_torque_sensor.srv import sensor_accessor
from geometry_msgs.msg import WrenchStamped

class ForceSensor():

    def __init__(self):
        self._fx = 0
        self._fy = 0
        self._fz = 0
        self._mx = 0
        self._my = 0
        self._mz = 0
        self.force_subscriber = rospy.Subscriber('robotiq_force_torque_wrench', WrenchStamped, self.Cb_function)

    def Cb_function(self,data):
        self._fx = data.wrench.force.x
        self._fy = data.wrench.force.y
        self._fz = data.wrench.force.z
        self._mx = data.wrench.torque.x
        self._my = data.wrench.torque.y
        self._mz = data.wrench.torque.z

    def print_values(self):
        print('F(x): '+ str(self._fx))
        print('F(y): '+ str(self._fy))
        print('F(z): '+ str(self._fz))
        print('M(x): '+ str(self._mx))
        print('M(y): '+ str(self._my))
        print('M(z): '+ str(self._mz))
	print(" ")

    def return_force(self):
        v = [self._fx, self._fy, self._fz, self._mx, self._my, self._mz]
	return v

    def mean(self):
	mx_ave=0
	my_ave=0
	for i in range(30):
		mx_ave += self._mx
		my_ave += self._my
	mx_ave = mx_ave/30
	my_ave = my_ave/30
	m = [mx_ave, my_ave]
	return m

if __name__ == '__main__':

    rospy.init_node('robotiq_sensor_test')
    force_sensor_obj = ForceSensor()

    r = rospy.Rate(50)

    while not rospy.is_shutdown():
        force_sensor_obj.print_values()
        r.sleep()

    rospy.spin()
