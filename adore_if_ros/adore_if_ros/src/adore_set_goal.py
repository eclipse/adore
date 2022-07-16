#!/usr/bin/env python
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
# ********************************************************************************
import math
import rospy
from adore_if_ros_msg.msg import NavigationGoal
from rospy.timer import sleep


if __name__ == '__main__':
    rospy.init_node('adore_set_goal', anonymous=True)
    output='ENV/NavigationGoal'
    marker_name = rospy.get_param('~marker')
    x0 = rospy.get_param(marker_name+'_x0',0.0)
    y0 = rospy.get_param(marker_name+'_y0',0.0)
    z0 = rospy.get_param(marker_name+'_z0',0.0)
    pub = rospy.Publisher(output,NavigationGoal,queue_size=1)
    g = NavigationGoal()
    g.target.x = x0
    g.target.y = y0
    g.target.z = z0
    while pub.get_num_connections()<2:
        sleep(0.1)
    for i in range(10):
        pub.publish(g)
        sleep(1.0)
    rospy.spin()