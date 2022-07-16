#!/usr/bin/env python3
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
from pickle import GLOBAL
import sys
import rospy
import pathlib
from adore_if_ros_msg.msg import NavigationGoal
import csv

class Observer(object):
    def __init__(self,filename):
        self._topic = 'ENV/NavigationGoal'
        self._navgoal_pub = rospy.Publisher(self._topic, NavigationGoal,queue_size=1)
        self._filename = filename
        self._fname = pathlib.Path(filename)
        assert self._fname.exists(), f'No such file: {self._fname}'  # check that the file exists
        self._last_change = self._fname.stat().st_mtime

    def update(self):
        try:
            mtime = self._fname.stat().st_mtime
            if mtime>self._last_change:
                self._last_change = mtime
                file = open(self._filename, 'r')
                lines = file.readlines()
                data = False
                x=0.0
                y=0.0
                for line in lines:
                    try:
                        values = list(csv.reader([line]))
                        x = float(values[0][0])
                        y = float(values[0][1])
                        data = True
                    except:
                        a=0
                if data:
                    goal=NavigationGoal()
                    goal.target.x = x
                    goal.target.y = y
                    self._navgoal_pub.publish(goal)
                    print('setting goal to [',x,',',y,']')
        except:
            print('could not parse line')


if __name__ == '__main__':
    rospy.init_node('plotlab2navigationgoal', anonymous=True)
    observer = Observer(sys.argv[1])
    r = rospy.Rate(1) 
    while not rospy.is_shutdown():
        observer.update()
        r.sleep()
