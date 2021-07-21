/*
 *
 * Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)
 * Institute of Transportation Systems. (https://www.dlr.de/ts/)
 *
 * 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 * 
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 * 
 * SPDX-License-Identifier: EPL-2.0
 * 
 *
 * 
 * File automatically generated with DLR Wind v2 (2021)
 * 
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:odds:12.0
 * 
 * Module: ODDInfo {}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef ODDS_ODDINFO_SENDER_H
#define ODDS_ODDINFO_SENDER_H

#include <ros/ros.h>

#include <wind_constants.h>
#include <odds_oddinfo/ODDMSG.h>   // From ROS
#include <odds_oddinfo.h>
#include <odds_oddinfo_translator_ros2wind.h>

#include <udp_sender.h>

namespace wind {
namespace wind_ros {

class Sender_odds_oddinfo
{
private:
    std::string            name;
    ros::NodeHandle        node_handle;
    ros::Subscriber        subscriber;
    ros::Subscriber        mirrorSubscriber;
    std::string            topic;
    wind::comm::UDPSender* udp_sender;

public:
    Sender_odds_oddinfo(wind::comm::UDPSender* udp_sender);
    ~Sender_odds_oddinfo() {};

    void start();
    void callback(const odds_oddinfo::ODDMSG::ConstPtr& msg);
    void mirrorCallback(const odds_oddinfo::ODDMSG::ConstPtr& msg);
};

} // Closing namespace ros
} // Closing namespace wind

#endif //ODDS_ODDINFO_SENDER_H
