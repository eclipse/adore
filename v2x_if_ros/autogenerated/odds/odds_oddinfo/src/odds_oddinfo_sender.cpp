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

#include <odds_oddinfo_sender.h>

wind::wind_ros::Sender_odds_oddinfo::Sender_odds_oddinfo(wind::comm::UDPSender* udp_sender) {
    this->udp_sender = udp_sender;
}

void
wind::wind_ros::Sender_odds_oddinfo::start() {
    this->topic = SENDER_ROS_TOPIC;

    // SENDER_ROS_TOPIC and SENDER_ROS_TOPIC_MIRROR are defined in wind_constants.h

    this->subscriber = node_handle.subscribe(SENDER_ROS_TOPIC, 10, 
        &wind::wind_ros::Sender_odds_oddinfo::callback, this);
    ROS_INFO_STREAM("[Sender_odds_oddinfo] Subscribing to topic: " << SENDER_ROS_TOPIC);

    this->mirrorSubscriber = node_handle.subscribe(SENDER_ROS_TOPIC_MIRROR, 10, 
        &wind::wind_ros::Sender_odds_oddinfo::mirrorCallback, this);
    ROS_INFO_STREAM("[Sender_odds_oddinfo] Subscribing to topic: " << SENDER_ROS_TOPIC_MIRROR);
}

void
wind::wind_ros::Sender_odds_oddinfo::callback(
        const odds_oddinfo::ODDMSG::ConstPtr& msg)
{

    wind::cpp::ODDInfo::ODDMSG wind = {};
    ros2wind(msg, &wind);

    ROS_INFO_STREAM("[Sender_odds_oddinfo] Sending UDP message (" << sizeof(wind::cpp::ODDInfo::ODDMSG) << ") to Wind recipient.");
    this->udp_sender->send(&wind, sizeof(wind::cpp::ODDInfo::ODDMSG));
}

void
wind::wind_ros::Sender_odds_oddinfo::mirrorCallback(
        const odds_oddinfo::ODDMSG::ConstPtr& msg)
{

    wind::cpp::ODDInfo::ODDMSG wind = {};
    ros2wind(msg, &wind);

    ROS_INFO_STREAM("[Sender_odds_oddinfo] Mirror: Sending UDP message (" << sizeof(wind::cpp::ODDInfo::ODDMSG) << ") to Wind recipient.");
    this->udp_sender->send(&wind, sizeof(wind::cpp::ODDInfo::ODDMSG));
}
