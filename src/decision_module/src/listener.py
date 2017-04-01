#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
# Revision $Id$

## Listener that listens to std_msgs/Strings published 
## to the 'D_M_direction' topic
## to the 'D_M_rotation' topic

from breezycreate2 import Robot
import time
import rospy
from std_msgs.msg import String

def callback_direction(data):
	global	direction_data
	direction_data = data.data
	rospy.loginfo(rospy.get_caller_id() + 'Move %s', direction_data)
	
def callback_rotation(data):
	global rotation_data
	rotation_data = data.data
    	rospy.loginfo(rospy.get_caller_id() + 'Rotation %s', rotation_data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('D_M_direction', String, callback_direction)
    rospy.Subscriber('D_M_rotation', String, callback_rotation)

    # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
	listener()
    
	bot = Robot()
	
	while(1):
		# Direction	    
		if direction_data == str("forward"):				
			bot.setForwardSpeed(50)	

		elif direction_data == str("backward"):
			bot.setForwardSpeed(-50)

		elif direction_data == str("stop"):
			bot.setForwardSpeed(0)

		# Rotation
		if rotation_data == str("left"):
			bot.setTurnSpeed(-50)
			    
		elif rotation_data == str("right"):
			bot.setTurnSpeed(50)
	
		elif rotation_data == str("stop"):
			bot.setTurnSpeed(0)
	
	bot.close() 
