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
import sys
import rospy
from importlib import import_module

#refer to: https://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
class listener(object):
    def __init__(self,topic,field):
        self._topic = topic
        self._field = field
        self._any_msg_sub = rospy.Subscriber(topic, rospy.AnyMsg, self.any_msg_callback)

    def any_msg_callback(self,data):
        connection_header =  data._connection_header['type'].split('/')
        ros_pkg = connection_header[0] + '.msg'
        msg_type = connection_header[1]
        print('Message type detected as ' + msg_type)
        msg_class = getattr(import_module(ros_pkg), msg_type)
        self._any_msg_sub.unregister()
        self._deserialized_sub = rospy.Subscriber(self._topic, msg_class, self.deserialized_callback)

    def deserialized_callback(self, data):
        print("len(msg."+self._field+")="+str(len(getattr(data,self._field))))

if __name__ == '__main__':
    rospy.init_node('test_vectorsize', anonymous=True)
    listener(sys.argv[1],sys.argv[2])
    rospy.spin()
