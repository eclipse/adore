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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:cpm_v1_19:1.3
 * 
 * Module: CPM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) ts(103324) cpm(1) version1(1)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <cpm_v1_19_cpm_pdu_descriptions_receiver.h>

/**
 * Constructor
 */
wind::wind_ros::Receiver_cpm_v1_19_cpm_pdu_descriptions::Receiver_cpm_v1_19_cpm_pdu_descriptions() : node_handle("~")
{
}

void
wind::wind_ros::Receiver_cpm_v1_19_cpm_pdu_descriptions::start() {
    publisher        = node_handle.advertise<cpm_v1_19_cpm_pdu_descriptions::CPM>(topic, 0);
    mirrorPublisher  = node_handle.advertise<cpm_v1_19_cpm_pdu_descriptions::CPM>(RECEIVER_ROS_TOPIC_MIRROR, 0);
}

uint32_t
wind::wind_ros::Receiver_cpm_v1_19_cpm_pdu_descriptions::message_id() {
    return sizeof(wind::cpp::CPM_PDU_Descriptions::CPM);
}


void
wind::wind_ros::Receiver_cpm_v1_19_cpm_pdu_descriptions::set_topic(std::string t) {
    topic = t;
}

std::string
wind::wind_ros::Receiver_cpm_v1_19_cpm_pdu_descriptions::message_name() {
    return "CPM";
}

/**
 * 
 */
void
wind::wind_ros::Receiver_cpm_v1_19_cpm_pdu_descriptions::incoming_message(const void* buf)
{
    const char *bufchar = (const char *)buf;
    unsigned char flags = bufchar[2];
    int bit0 = flags & 1;                          // client mirror flag
    int bit1 = ((unsigned char)flags >> 1) & 1;    // UDP mirror flag
    int bit2 = ((unsigned char)flags >> 2) & 1;    // CCU mirror flag
    int bit3 = ((unsigned char)flags >> 3) & 1;    // test message flag

    if(bit3)
        ROS_INFO_STREAM("Incoming CPM TEST message");
    else
        ROS_INFO_STREAM("Incoming CPM message");

    wind::cpp::CPM_PDU_Descriptions::CPM* wind = (wind::cpp::CPM_PDU_Descriptions::CPM*)buf;
    cpm_v1_19_cpm_pdu_descriptions::CPM ros;

    wind::wind_ros::wind2ros(&ros, wind);

    if(bit0) {
        ROS_INFO_STREAM("Mirroring CPM message");
        mirrorPublisher.publish(ros);
    } else
        publisher.publish(ros);
}
