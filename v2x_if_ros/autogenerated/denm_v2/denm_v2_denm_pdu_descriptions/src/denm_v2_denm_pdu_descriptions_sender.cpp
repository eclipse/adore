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

#include <denm_v2_denm_pdu_descriptions_sender.h>

wind::wind_ros::Sender_denm_v2_denm_pdu_descriptions::Sender_denm_v2_denm_pdu_descriptions(wind::comm::UDPSender* udp_sender) {
    this->udp_sender = udp_sender;
    messagesCounter = 0;

    buff = new char[1024*1024*10];
}

void
wind::wind_ros::Sender_denm_v2_denm_pdu_descriptions::start()
{
    this->subscriber = node_handle.subscribe(topic, 10, 
        &wind::wind_ros::Sender_denm_v2_denm_pdu_descriptions::callback, this);
    ROS_INFO_STREAM("Subscribing to topic: " << topic);

    this->mirrorSubscriber = node_handle.subscribe(SENDER_ROS_TOPIC_MIRROR, 10, 
        &wind::wind_ros::Sender_denm_v2_denm_pdu_descriptions::callbackMirror, this);
}

void
wind::wind_ros::Sender_denm_v2_denm_pdu_descriptions::callbackMirror(
        const denm_v2_denm_pdu_descriptions::DENM::ConstPtr& msg)
{
    #if !RSERIAL_ENABLED
        int buf_len = sizeof(wind::cpp::DENM_PDU_Descriptions::DENM);
        wind::cpp::DENM_PDU_Descriptions::DENM wind = {};
        ros2wind(msg, &wind);
        const char *buff = (const char *)&wind;
    #else
        int buf_len = encode(msg, buff);

    #endif

    ROS_INFO_STREAM("***** Outgoing DENM request message:  counter(" << messagesCounter++ << ") *****");

    unsigned int stationId;
        stationId = int((unsigned char)(buff[2]) << 24 |
                    (unsigned char)(buff[3]) << 16 |
                    (unsigned char)(buff[4]) << 8 |
                    (unsigned char)(buff[5]));

    ROS_INFO_STREAM("V2X data: messageId(" << int(buff[1]) << ") protocolVersion(" << int(buff[0])
                << ") stationId(" << stationId << ")");
    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] [Mirror] Sending UDP message (" << buf_len << ") to " << output_ip << ":" << output_port);

    #if !RSERIAL_ENABLED
        this->udp_sender->send(&wind, buf_len);
    #else
        this->udp_sender->send(buff, buf_len);
    #endif
}

void
wind::wind_ros::Sender_denm_v2_denm_pdu_descriptions::callback(
        const denm_v2_denm_pdu_descriptions::DENM::ConstPtr& msg)
{
    #if !RSERIAL_ENABLED
        int buf_len = sizeof(wind::cpp::DENM_PDU_Descriptions::DENM);
        wind::cpp::DENM_PDU_Descriptions::DENM wind = {};
        ros2wind(msg, &wind);
        const char *buff = (const char *)&wind;
    #else
        int buf_len = encode(msg, buff);

    #endif

    ROS_INFO_STREAM("***** Outgoing DENM request message:  counter(" << messagesCounter++ << ") *****");

    unsigned int stationId;
        stationId = int((unsigned char)(buff[2]) << 24 |
                    (unsigned char)(buff[3]) << 16 |
                    (unsigned char)(buff[4]) << 8 |
                    (unsigned char)(buff[5]));

    ROS_INFO_STREAM("V2X data: messageId(" << int(buff[1]) << ") protocolVersion(" << int(buff[0])
                << ") stationId(" << stationId << ")");
    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] Sending UDP message (" << buf_len << ") to " << output_ip << ":" << output_port);

    #if !RSERIAL_ENABLED
        this->udp_sender->send(&wind, buf_len);
    #else
        this->udp_sender->send(buff, buf_len);
    #endif
}
