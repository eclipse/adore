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

#include <dsrc_v2_rtcmem_pdu_descriptions_sender.h>

wind::wind_ros::Sender_dsrc_v2_rtcmem_pdu_descriptions::Sender_dsrc_v2_rtcmem_pdu_descriptions(wind::comm::UDPSender* udp_sender) {
    this->udp_sender = udp_sender;
    messagesCounter = 0;
}

void
wind::wind_ros::Sender_dsrc_v2_rtcmem_pdu_descriptions::start()
{
    this->subscriber = node_handle.subscribe(topic, 10, 
        &wind::wind_ros::Sender_dsrc_v2_rtcmem_pdu_descriptions::callback, this);
    ROS_INFO_STREAM("Subscribing to topic: " << topic);

    this->mirrorSubscriber = node_handle.subscribe(SENDER_ROS_TOPIC_MIRROR, 10, 
        &wind::wind_ros::Sender_dsrc_v2_rtcmem_pdu_descriptions::mirrorCallback, this);
}

void
wind::wind_ros::Sender_dsrc_v2_rtcmem_pdu_descriptions::callback(
        const dsrc_v2_rtcmem_pdu_descriptions::RTCMEM::ConstPtr& msg)
{
    wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM wind = {};
    ros2wind(msg, &wind);

    const char *buffer = (const char *)&wind;
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

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] Sending UDP message (" << sizeof(wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM) << ") to " << output_ip << ":" << output_port);
    this->udp_sender->send(&wind, sizeof(wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM));
}

void
wind::wind_ros::Sender_dsrc_v2_rtcmem_pdu_descriptions::mirrorCallback(
        const dsrc_v2_rtcmem_pdu_descriptions::RTCMEM::ConstPtr& msg)
{
    wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM wind = {};
    ros2wind(msg, &wind);

    ROS_INFO_STREAM("***** Outgoing RTCMEM request message:  counter(" << messagesCounter++ << ") *****");

    const char *buffer = (const char *)&wind;
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

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] [Mirror] Sending UDP message (" << sizeof(wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM) << ") to " << output_ip << ":" << output_port);
    this->udp_sender->send(&wind, sizeof(wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM));
}
