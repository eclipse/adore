#********************************************************************************
 # Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 # Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 #
 # This program and the accompanying materials are made available under the 
 # terms of the Eclipse Public License 2.0 which is available at
 # http://www.eclipse.org/legal/epl-2.0.
 #
 # SPDX-License-Identifier: EPL-2.0 
 #
 # Contributors: 
 #   Matthias Nichting - initial API and implementation
 #*******************************************************************************/

import rospy
from adore_if_ros_msg.msg import SchedulerNotification


class PySchedulerNotificationManager:
    def __init__(self, id, duration_nsec: int, register=True):
        self.notification_publisher = rospy.Publisher(
            '/SIM/scheduling', SchedulerNotification, queue_size=1)
        self.sn = PySchedulerNotification(id)
        self.duration_nsec: int
        self.duration_nsec = duration_nsec
        if register:
            self.register_at_scheduler()

    def notify_scheduler(self, sec: int, nsec: int):
        self.sn.setUpperTimeLimit(sec, nsec+self.duration_nsec)
        self.notification_publisher.publish(self.sn.getMessage())

    def register_at_scheduler(self):
        self.sn.setUpperTimeLimit(0, self.duration_nsec)
        # wait until connection is established to prevent the first message being omitted
        while self.notification_publisher.get_num_connections() == 0:
            pass
        self.notification_publisher.publish(self.sn.getMessage())


class PySchedulerNotification:

    def __init__(self, id):
        self.upper_time_limit_sec = 0
        self.upper_time_limit_nsec = 0
        self.msg = SchedulerNotification()
        self.msg.identifier = id

    def getUpperTimeLimit_sec(self):
        return self.upper_time_limit_sec

    def getUpperTimeLimit_nsec(self):
        return self.upper_time_limit_nsecs

    def setUpperTimeLimit(self, sec: int, nsec: int):
        while (nsec >= 1e9):
            nsec -= 1e9
            sec += 1
        self.upper_time_limit_sec = sec
        self.upper_time_limit_nsec = nsec

    def getID(self):
        return self.id

    def getMessage(self):
        self.msg.upperTimeLimit.secs = (int)(self.upper_time_limit_sec)
        self.msg.upperTimeLimit.nsecs = (int)(self.upper_time_limit_nsec)
        return self.msg


class BaseApp:
    
    def __init__(self, node_name, duration, allow_multiple_instances=False):
        self.callbacks = []
        self.duration = duration
        self.first_run = True
        print("init")
        rospy.init_node(node_name, anonymous=allow_multiple_instances)
        if duration != 0:
            print("create timer callback")
            self.snm = PySchedulerNotificationManager(100123, self.duration)
            self.timer = rospy.Timer(rospy.Duration(0,self.duration/2), self.auxiliary_callback)
    
    def auxiliary_callback(self,event):
        if self.first_run:
            self.first_run = False
            return
        self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(0,self.duration), self.periodic_callback)
        self.periodic_callback(event)
        

    def run(self):
        rospy.spin()

    def periodic_callback(self, event):
        # iterate list of callbacks
        for f in self.callbacks:
            f()
        self.snm.notify_scheduler(
            rospy.Time.now().secs, rospy.Time.now().nsecs)

    def add_callback(self, func):
        self.callbacks.append(func)

    def get_param(self, name):
        return rospy.get_param(name)
