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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_dmove:3.0
 * 
 * Module: MCM_DMove {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) mcm(1) version(3)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <mcm_dmove_mcm_dmove_receiver.h>

/**
 * Constructor
 */
wind::wind_ros::Receiver_mcm_dmove_mcm_dmove::Receiver_mcm_dmove_mcm_dmove() : node_handle("~")
{
}

void
wind::wind_ros::Receiver_mcm_dmove_mcm_dmove::start() {
    publisher        = node_handle.advertise<mcm_dmove_mcm_dmove::MCM>(topic, 0);
    mirrorPublisher  = node_handle.advertise<mcm_dmove_mcm_dmove::MCM>(RECEIVER_ROS_TOPIC_MIRROR, 0);
}

uint32_t
wind::wind_ros::Receiver_mcm_dmove_mcm_dmove::message_id() {
    return sizeof(wind::cpp::MCM_DMove::MCM);
}


void
wind::wind_ros::Receiver_mcm_dmove_mcm_dmove::set_topic(std::string t) {
    topic = t;
}

std::string
wind::wind_ros::Receiver_mcm_dmove_mcm_dmove::message_name() {
    return "MCM";
}

/**
 * 
 */
void
wind::wind_ros::Receiver_mcm_dmove_mcm_dmove::incoming_message(const void* buf)
{
    const char *bufchar = (const char *)buf;
    unsigned char flags = bufchar[2];
    int bit0 = flags & 1;                          // client mirror flag
    int bit1 = ((unsigned char)flags >> 1) & 1;    // UDP mirror flag
    int bit2 = ((unsigned char)flags >> 2) & 1;    // CCU mirror flag
    int bit3 = ((unsigned char)flags >> 3) & 1;    // test message flag

    if(bit3)
        ROS_INFO_STREAM("Incoming MCM TEST message");
    else
        ROS_INFO_STREAM("Incoming MCM message");

    wind::cpp::MCM_DMove::MCM* wind = (wind::cpp::MCM_DMove::MCM*)buf;
    mcm_dmove_mcm_dmove::MCM ros;

    wind::wind_ros::wind2ros(&ros, wind);

    if(bit0) {
        ROS_INFO_STREAM("Mirroring MCM message");
        mirrorPublisher.publish(ros);
    } else
        publisher.publish(ros);
}
