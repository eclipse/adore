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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:dsrc_v2:2.5
 * 
 * Module: RTCMEM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(103301) rtcmem(5) version1(1)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <dsrc_v2_rtcmem_pdu_descriptions_receiver.h>

/**
 * Constructor
 */
wind::wind_ros::Receiver_dsrc_v2_rtcmem_pdu_descriptions::Receiver_dsrc_v2_rtcmem_pdu_descriptions() : node_handle("~")
{
    messagesCounter = 0;
}

void
wind::wind_ros::Receiver_dsrc_v2_rtcmem_pdu_descriptions::start() {
    publisher        = node_handle.advertise<dsrc_v2_rtcmem_pdu_descriptions::RTCMEM>(topic, 0);
    mirrorPublisher  = node_handle.advertise<dsrc_v2_rtcmem_pdu_descriptions::RTCMEM>(RECEIVER_ROS_TOPIC_MIRROR, 0);
}

uint32_t
wind::wind_ros::Receiver_dsrc_v2_rtcmem_pdu_descriptions::message_id() {
    return sizeof(wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM);
}


void
wind::wind_ros::Receiver_dsrc_v2_rtcmem_pdu_descriptions::set_topic(std::string t) {
    topic = t;
}

std::string
wind::wind_ros::Receiver_dsrc_v2_rtcmem_pdu_descriptions::message_name() {
    return "RTCMEM";
}

/**
 * 
 */
void
wind::wind_ros::Receiver_dsrc_v2_rtcmem_pdu_descriptions::incoming_message(const void* buf)
{
    const char *buffer = (const char *)buf;
    unsigned char flags = buffer[2];
    int bit0 = flags & 1;                          // client mirror flag
    int bit1 = ((unsigned char)flags >> 1) & 1;    // UDP mirror flag
    int bit2 = ((unsigned char)flags >> 2) & 1;    // CCU mirror flag
    int bit3 = ((unsigned char)flags >> 3) & 1;    // test message flag

    if(bit3)
        ROS_INFO_STREAM("***** Incoming RTCMEM TEST indication message: counter(" << messagesCounter++ << ") *****");
    else
        ROS_INFO_STREAM("***** Incoming RTCMEM indication message: counter(" << messagesCounter++ << ") *****");

    unsigned int stationId;
    if(buffer[0] == 1) //little endian
        stationId = int((unsigned char)(buffer[13]) << 24 |
                    (unsigned char)(buffer[12]) << 16 |
                    (unsigned char)(buffer[11]) << 8 |
                    (unsigned char)(buffer[10]));
    else               // big endian
        stationId = int((unsigned char)(buffer[10]) << 24 |
                    (unsigned char)(buffer[11]) << 16 |
                    (unsigned char)(buffer[12]) << 8 |
                    (unsigned char)(buffer[13]));

    ROS_INFO_STREAM("V2X data: messageId(" << int(buffer[9]) << ") protocolVersion(" << int(buffer[8])
                << ") stationId(" << stationId << ")");


    wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM* wind = (wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM*)buf;
    dsrc_v2_rtcmem_pdu_descriptions::RTCMEM ros;

    wind::wind_ros::wind2ros(&ros, wind);

    if(bit0) {
        ROS_INFO_STREAM("Mirroring RTCMEM message");
        mirrorPublisher.publish(ros);
    } else
        publisher.publish(ros);
}
