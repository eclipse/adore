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

#include <strp_fau_strpmessage_receiver.h>

/**
 * Constructor
 */
wind::wind_ros::Receiver_strp_fau_strpmessage::Receiver_strp_fau_strpmessage() : node_handle("~")
{
    topic = "SpaceTimeReservationProcedure_in";
    name = ros::this_node::getName();
    publisher  = node_handle.advertise<strp_fau_strpmessage::SpaceTimeReservationProcedure>(topic, 0);
}

/**
 * 
 */
uint32_t
wind::wind_ros::Receiver_strp_fau_strpmessage::message_id() {
    return sizeof(wind::cpp::STRPMessage::SpaceTimeReservationProcedure);
}

/**
 * 
 */
void
wind::wind_ros::Receiver_strp_fau_strpmessage::incoming_message(const void* buf)
{
    wind::cpp::STRPMessage::SpaceTimeReservationProcedure* wind = (wind::cpp::STRPMessage::SpaceTimeReservationProcedure*)buf;
    strp_fau_strpmessage::SpaceTimeReservationProcedure ros;

    wind::wind_ros::wind2ros(&ros, wind);

    publisher.publish(ros);
}
