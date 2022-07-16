#!/usr/bin/python3
# ********************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *   Daniel Heß 
# *	  Thomas Lobig
# ********************************************************************************

import rospy

#std_msgs::Float64
from std_msgs.msg import Float64
#std_msgs::Bool
from std_msgs.msg import Bool

#global variable
timeout = 0

def callback_time(msg):
	global timeout
	if timeout == 0:
		timeout = rospy.get_param('/PARAMS/SIM/timeout')
		print("timeout set to ", timeout, "\n")
	if msg.data > timeout:
		print("timeout reached, shutting down")
		rospy.signal_shutdown("timeout reached")

def callback_term(msg):
	if msg.data:
		print("termination signal received, shutting down")
		rospy.signal_shutdown("termination signal received")	
    
def listener():

	rospy.init_node('listener', anonymous=True)

	s1 = rospy.Subscriber("/SIM/utc", Float64, callback_time)
	s2 = rospy.Subscriber("/SIM/terminate", Bool, callback_term)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()
