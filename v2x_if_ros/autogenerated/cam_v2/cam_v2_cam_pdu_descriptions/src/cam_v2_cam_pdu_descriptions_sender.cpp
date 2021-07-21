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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:cam_v2:1.4
 * 
 * Module: CAM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) cam(2) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <cam_v2_cam_pdu_descriptions_sender.h>

wind::wind_ros::Sender_cam_v2_cam_pdu_descriptions::Sender_cam_v2_cam_pdu_descriptions(wind::comm::UDPSender* udp_sender) {
    this->udp_sender = udp_sender;
}

void
wind::wind_ros::Sender_cam_v2_cam_pdu_descriptions::start()
{
    this->subscriber = node_handle.subscribe(topic, 10, 
        &wind::wind_ros::Sender_cam_v2_cam_pdu_descriptions::callback, this);
    ROS_INFO_STREAM("Subscribing to topic: " << topic);

    this->mirrorSubscriber = node_handle.subscribe(SENDER_ROS_TOPIC_MIRROR, 10, 
        &wind::wind_ros::Sender_cam_v2_cam_pdu_descriptions::mirrorCallback, this);
}

void
wind::wind_ros::Sender_cam_v2_cam_pdu_descriptions::callback(
        const cam_v2_cam_pdu_descriptions::CAM::ConstPtr& msg)
{
    wind::cpp::CAM_PDU_Descriptions::CAM wind = {};
    ros2wind(msg, &wind);

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] Sending UDP message (" << sizeof(wind::cpp::CAM_PDU_Descriptions::CAM) << ") to " << output_ip << ":" << output_port);
    this->udp_sender->send(&wind, sizeof(wind::cpp::CAM_PDU_Descriptions::CAM));
}

void
wind::wind_ros::Sender_cam_v2_cam_pdu_descriptions::mirrorCallback(
        const cam_v2_cam_pdu_descriptions::CAM::ConstPtr& msg)
{
    wind::cpp::CAM_PDU_Descriptions::CAM wind = {};
    ros2wind(msg, &wind);

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] [Mirror] Sending UDP message (" << sizeof(wind::cpp::CAM_PDU_Descriptions::CAM) << ") to " << output_ip << ":" << output_port);
    this->udp_sender->send(&wind, sizeof(wind::cpp::CAM_PDU_Descriptions::CAM));
}
