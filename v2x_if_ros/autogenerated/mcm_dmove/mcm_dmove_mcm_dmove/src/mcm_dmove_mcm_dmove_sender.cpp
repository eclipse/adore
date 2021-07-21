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

#include <mcm_dmove_mcm_dmove_sender.h>

wind::wind_ros::Sender_mcm_dmove_mcm_dmove::Sender_mcm_dmove_mcm_dmove(wind::comm::UDPSender* udp_sender) {
    this->udp_sender = udp_sender;
}

void
wind::wind_ros::Sender_mcm_dmove_mcm_dmove::start()
{
    this->subscriber = node_handle.subscribe(topic, 10, 
        &wind::wind_ros::Sender_mcm_dmove_mcm_dmove::callback, this);
    ROS_INFO_STREAM("Subscribing to topic: " << topic);

    this->mirrorSubscriber = node_handle.subscribe(SENDER_ROS_TOPIC_MIRROR, 10, 
        &wind::wind_ros::Sender_mcm_dmove_mcm_dmove::mirrorCallback, this);
}

void
wind::wind_ros::Sender_mcm_dmove_mcm_dmove::callback(
        const mcm_dmove_mcm_dmove::MCM::ConstPtr& msg)
{
    wind::cpp::MCM_DMove::MCM wind = {};
    ros2wind(msg, &wind);

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] Sending UDP message (" << sizeof(wind::cpp::MCM_DMove::MCM) << ") to " << output_ip << ":" << output_port);
    this->udp_sender->send(&wind, sizeof(wind::cpp::MCM_DMove::MCM));
}

void
wind::wind_ros::Sender_mcm_dmove_mcm_dmove::mirrorCallback(
        const mcm_dmove_mcm_dmove::MCM::ConstPtr& msg)
{
    wind::cpp::MCM_DMove::MCM wind = {};
    ros2wind(msg, &wind);

    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] [Mirror] Sending UDP message (" << sizeof(wind::cpp::MCM_DMove::MCM) << ") to " << output_ip << ":" << output_port);
    this->udp_sender->send(&wind, sizeof(wind::cpp::MCM_DMove::MCM));
}
