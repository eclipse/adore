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
from geometry_msgs.msg import Pose
from rospy.timer import sleep


if __name__ == '__main__':
    rospy.init_node('adore_set_pose', anonymous=True)
    output='SIM/ResetVehiclePose'
    marker_name = rospy.get_param('~marker')
    x0 = rospy.get_param(marker_name+'_x0',0.0)
    y0 = rospy.get_param(marker_name+'_y0',0.0)
    z0 = rospy.get_param(marker_name+'_z0',0.0)
    x1 = rospy.get_param(marker_name+'_x1',1.0)
    y1 = rospy.get_param(marker_name+'_y1',0.0)
    z1 = rospy.get_param(marker_name+'_z1',0.0)
    pub = rospy.Publisher(output,Pose,queue_size=1)
    p0 = Pose()
    p0.position.x = x0
    p0.position.y = y0
    p0.position.z = z0
    yaw = math.atan2(y1-y0,x1-x0)
    p0.orientation.x = 0.0
    p0.orientation.y = 0.0
    p0.orientation.z = math.sin(yaw*0.5)
    p0.orientation.w = math.cos(yaw*0.5)
    while pub.get_num_connections()==0:
        sleep(0.1)
    pub.publish(p0)
    rospy.spin()