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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:denm_v2:1.5
 * 
 * Module: DENM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) denm(1) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <denm_v2_denm_pdu_descriptions_receiver.h>

/**
 * Constructor
 */
wind::wind_ros::Receiver_denm_v2_denm_pdu_descriptions::Receiver_denm_v2_denm_pdu_descriptions() : node_handle("~")
{
    messagesCounter = 0;
}

void
wind::wind_ros::Receiver_denm_v2_denm_pdu_descriptions::start() {
    publisher        = node_handle.advertise<denm_v2_denm_pdu_descriptions::DENM>(topic, 0);
    mirrorPublisher  = node_handle.advertise<denm_v2_denm_pdu_descriptions::DENM>(RECEIVER_ROS_TOPIC_MIRROR, 0);
}

uint32_t
wind::wind_ros::Receiver_denm_v2_denm_pdu_descriptions::message_id() {
    return sizeof(wind::cpp::DENM_PDU_Descriptions::DENM);
}


void
wind::wind_ros::Receiver_denm_v2_denm_pdu_descriptions::set_topic(std::string t) {
    topic = t;
}

std::string
wind::wind_ros::Receiver_denm_v2_denm_pdu_descriptions::message_name() {
    return "DENM";
}

/**
 * 
 */
void
wind::wind_ros::Receiver_denm_v2_denm_pdu_descriptions::incoming_message(const void* buf)
{
    const char *buffer = (const char *)buf;
    unsigned char flags = buffer[2];
    int bit0 = flags & 1;                          // client mirror flag
    int bit1 = ((unsigned char)flags >> 1) & 1;    // UDP mirror flag
    int bit2 = ((unsigned char)flags >> 2) & 1;    // CCU mirror flag
    int bit3 = ((unsigned char)flags >> 3) & 1;    // test message flag

    if(bit3)
        ROS_INFO_STREAM("***** Incoming DENM TEST indication message: counter(" << messagesCounter++ << ") *****");
    else
        ROS_INFO_STREAM("***** Incoming DENM indication message: counter(" << messagesCounter++ << ") *****");

    const int HEADER_SIZE = 8;
    unsigned int stationId = int((unsigned char)(buffer[2+HEADER_SIZE]) << 24 |
                (unsigned char)(buffer[3+HEADER_SIZE]) << 16 |
                (unsigned char)(buffer[4+HEADER_SIZE]) << 8 |
                (unsigned char)(buffer[5+HEADER_SIZE]));

    ROS_INFO_STREAM("V2X data: messageId(" << int(buffer[1+HEADER_SIZE]) << ") protocolVersion(" << int(buffer[0+HEADER_SIZE])
                << ") stationId(" << stationId << ")");


    denm_v2_denm_pdu_descriptions::DENM ros;

    #if !RSERIAL_ENABLED
        wind::cpp::DENM_PDU_Descriptions::DENM* wind = (wind::cpp::DENM_PDU_Descriptions::DENM*)buf;
        wind::wind_ros::wind2ros(&ros, wind);
    #else
        wind::wind_ros::decode(&ros, &buffer[8]);
    #endif

    if(true || bit0) {
        ROS_INFO_STREAM("Mirroring DENM message");
        mirrorPublisher.publish(ros);
    } else
        publisher.publish(ros);
}
