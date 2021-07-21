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

#include <strp_fau_strpmessage_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const strp_fau_strpmessage::SpaceTimeReservationProcedure::ConstPtr& ros, wind::cpp::STRPMessage::SpaceTimeReservationProcedure* wind)
{
    if(WIND_DEBUG) ROS_INFO_STREAM("########### Start of translation ###########");

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;

    // START CHOICE STRPMessageType
    if(WIND_DEBUG) ROS_INFO_STREAM("   Choice STRPMessageType");
    wind->data.choice = ros->data.choice;

    if(ros->data.choice == 0) {

        wind->data.request.generationTime.value =
            ros->data.request.generationTime.value;

        wind->data.request.timeout.value =
            ros->data.request.timeout.value;

        wind->data.request.requestID.value =
            ros->data.request.requestID.value;

        wind->data.request.marker0.latitude.value =
            ros->data.request.marker0.latitude.value;

        wind->data.request.marker0.longitude.value =
            ros->data.request.marker0.longitude.value;

        wind->data.request.marker0.altitude.value =
            ros->data.request.marker0.altitude.value;

        wind->data.request.marker1.east.value =
            ros->data.request.marker1.east.value;

        wind->data.request.marker1.north.value =
            ros->data.request.marker1.north.value;

        wind->data.request.marker1.up.value =
            ros->data.request.marker1.up.value;

        // START CHOICE ReservationShape
        if(WIND_DEBUG) ROS_INFO_STREAM("      Choice ReservationShape");
        wind->data.request.reservation.choice = ros->data.request.reservation.choice;

        if(ros->data.request.reservation.choice == 0) {

            wind->data.request.reservation.timeGate.t0.value =
                ros->data.request.reservation.timeGate.t0.value;

            wind->data.request.reservation.timeGate.t1.value =
                ros->data.request.reservation.timeGate.t1.value;

            wind->data.request.reservation.timeGate.s0.value =
                ros->data.request.reservation.timeGate.s0.value;

            wind->data.request.reservation.timeGate.s1.value =
                ros->data.request.reservation.timeGate.s1.value;

            wind->data.request.reservation.timeGate.vmin.value =
                ros->data.request.reservation.timeGate.vmin.value;
        }
        else if(ros->data.request.reservation.choice == 1)  // CHOICE ReservationShape
        {

            wind->data.request.reservation.positionGate.t0.value =
                ros->data.request.reservation.positionGate.t0.value;

            wind->data.request.reservation.positionGate.t1.value =
                ros->data.request.reservation.positionGate.t1.value;

            wind->data.request.reservation.positionGate.s0.value =
                ros->data.request.reservation.positionGate.s0.value;

            wind->data.request.reservation.positionGate.s1.value =
                ros->data.request.reservation.positionGate.s1.value;

            wind->data.request.reservation.positionGate.vmin.value =
                ros->data.request.reservation.positionGate.vmin.value;
        }
        else   // CHOICE ReservationShape
        {

            wind->data.request.reservation.fixedBlock.t0.value =
                ros->data.request.reservation.fixedBlock.t0.value;

            wind->data.request.reservation.fixedBlock.t1.value =
                ros->data.request.reservation.fixedBlock.t1.value;

            wind->data.request.reservation.fixedBlock.s0.value =
                ros->data.request.reservation.fixedBlock.s0.value;

            wind->data.request.reservation.fixedBlock.s1.value =
                ros->data.request.reservation.fixedBlock.s1.value;
        }
        // END CHOICE ReservationShape
    }
    else if(ros->data.choice == 1)  // CHOICE STRPMessageType
    {

        wind->data.accept.requestorID.value =
            ros->data.accept.requestorID.value;

        wind->data.accept.requestID.value =
            ros->data.accept.requestID.value;
    }
    else if(ros->data.choice == 2)  // CHOICE STRPMessageType
    {

        wind->data.reject.requestorID.value =
            ros->data.reject.requestorID.value;

        wind->data.reject.requestID.value =
            ros->data.reject.requestID.value;
    }
    else   // CHOICE STRPMessageType
    {

        wind->data.cancel.requestID.value =
            ros->data.cancel.requestID.value;
    }
    // END CHOICE STRPMessageType
    if(WIND_DEBUG) ROS_INFO_STREAM("###########  End of translation  ###########");
}
