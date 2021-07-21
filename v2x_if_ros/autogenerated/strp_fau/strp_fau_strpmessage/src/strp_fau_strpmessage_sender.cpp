/*
 *
 * Copyright (C) 2017-2020 German Aerospace Center e.V. (https://www.dlr.de)
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
 * File automatically generated with DLR Wind v2 (30.11.2020 11:00:04)
 * 
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:strp_fau:1.0
 * 
 * Module: STRPMessage {}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */
#define WIND_DEBUG 0

#include <strp_fau_strpmessage_sender.h>

wind::wind_ros::Sender_strp_fau_strpmessage::Sender_strp_fau_strpmessage(wind::comm::UDPSender* udp_sender) {
    this->udp_sender = udp_sender;
}

void
wind::wind_ros::Sender_strp_fau_strpmessage::start() {
    this->topic = "SpaceTimeReservationProcedure_out";
    this->subscriber = node_handle.subscribe(topic, 10, 
        &wind::wind_ros::Sender_strp_fau_strpmessage::callback, this);

    ROS_INFO_STREAM("[Sender_strp_fau_strpmessage] Subscribing to topic: " << this->topic);
}

void
wind::wind_ros::Sender_strp_fau_strpmessage::callback(
        const strp_fau_strpmessage::SpaceTimeReservationProcedure::ConstPtr& msg)
{
    if(WIND_DEBUG) ROS_DEBUG("[Sender_strp_fau_strpmessage] Called :strp_fau_strpmessage Callback().");

    wind::cpp::STRPMessage::SpaceTimeReservationProcedure wind = {};
    ros2wind(msg, &wind);

    if(WIND_DEBUG) ROS_INFO_STREAM("[Sender_strp_fau_strpmessage] Sending UDP message (" << sizeof(wind::cpp::STRPMessage::SpaceTimeReservationProcedure) << ") to Wind recipient.");
    this->udp_sender->send(&wind, sizeof(wind::cpp::STRPMessage::SpaceTimeReservationProcedure));
}
