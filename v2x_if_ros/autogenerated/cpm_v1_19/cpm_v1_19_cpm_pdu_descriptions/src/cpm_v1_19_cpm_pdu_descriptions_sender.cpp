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

#include <cpm_v1_19_cpm_pdu_descriptions_sender.h>

wind::wind_ros::Sender_cpm_v1_19_cpm_pdu_descriptions::Sender_cpm_v1_19_cpm_pdu_descriptions(wind::comm::UDPSender* udp_sender) {
    this->udp_sender = udp_sender;
}

void
wind::wind_ros::Sender_cpm_v1_19_cpm_pdu_descriptions::start()
{
    this->subscriber = node_handle.subscribe(topic, 10, 
        &wind::wind_ros::Sender_cpm_v1_19_cpm_pdu_descriptions::callback, this);
    ROS_INFO_STREAM("Subscribing to topic: " << topic);

    this->mirrorSubscriber = node_handle.subscribe(SENDER_ROS_TOPIC_MIRROR, 10, 
        &wind::wind_ros::Sender_cpm_v1_19_cpm_pdu_descriptions::mirrorCallback, this);
}

void
wind::wind_ros::Sender_cpm_v1_19_cpm_pdu_descriptions::callback(
        const cpm_v1_19_cpm_pdu_descriptions::CPM::ConstPtr& msg)
{
    wind::cpp::CPM_PDU_Descriptions::CPM wind = {};
    ros2wind(msg, &wind);

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] Sending UDP message (" << sizeof(wind::cpp::CPM_PDU_Descriptions::CPM) << ") to " << output_ip << ":" << output_port);
    this->udp_sender->send(&wind, sizeof(wind::cpp::CPM_PDU_Descriptions::CPM));
}

void
wind::wind_ros::Sender_cpm_v1_19_cpm_pdu_descriptions::mirrorCallback(
        const cpm_v1_19_cpm_pdu_descriptions::CPM::ConstPtr& msg)
{
    wind::cpp::CPM_PDU_Descriptions::CPM wind = {};
    ros2wind(msg, &wind);

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] [Mirror] Sending UDP message (" << sizeof(wind::cpp::CPM_PDU_Descriptions::CPM) << ") to " << output_ip << ":" << output_port);
    this->udp_sender->send(&wind, sizeof(wind::cpp::CPM_PDU_Descriptions::CPM));
}
