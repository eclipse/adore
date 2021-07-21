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

#include <odds_oddinfo_receiver.h>

/**
 * Constructor
 */
wind::wind_ros::Receiver_odds_oddinfo::Receiver_odds_oddinfo() : node_handle("~")
{
    name = ros::this_node::getName();

    publisher        = node_handle.advertise<odds_oddinfo::ODDMSG>(RECEIVER_ROS_TOPIC, 0);
    mirrorPublisher  = node_handle.advertise<odds_oddinfo::ODDMSG>(RECEIVER_ROS_TOPIC_MIRROR, 0);
}

/**
 * 
 */
uint32_t
wind::wind_ros::Receiver_odds_oddinfo::message_id() {
    return sizeof(wind::cpp::ODDInfo::ODDMSG);
}

/**
 * 
 */
void
wind::wind_ros::Receiver_odds_oddinfo::incoming_message(const void* buf)
{
    const char *bufchar = (const char *)buf;
    unsigned char flags = bufchar[2];
    int bit0 = flags & 1;                          // client mirror flag
    int bit1 = ((unsigned char)flags >> 1) & 1;    // UDP mirror flag
    int bit2 = ((unsigned char)flags >> 2) & 1;    // CCU mirror flag
    int bit3 = ((unsigned char)flags >> 3) & 1;    // test message flag

    if(bit3)
        ROS_INFO_STREAM("Incoming ODDMSG TEST message");
    else
        ROS_INFO_STREAM("Incoming ODDMSG message");

    wind::cpp::ODDInfo::ODDMSG* wind = (wind::cpp::ODDInfo::ODDMSG*)buf;
    odds_oddinfo::ODDMSG ros;

    wind::wind_ros::wind2ros(&ros, wind);

    if(bit0) {
        ROS_INFO_STREAM("Mirroring ODDMSG message");
        mirrorPublisher.publish(ros);
    } else
        publisher.publish(ros);
}
