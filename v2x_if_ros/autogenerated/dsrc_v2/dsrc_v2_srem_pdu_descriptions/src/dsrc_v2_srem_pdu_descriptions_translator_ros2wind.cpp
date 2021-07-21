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
 * Module: SREM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(103301) srem(3) version2(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <dsrc_v2_srem_pdu_descriptions_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const dsrc_v2_srem_pdu_descriptions::SREM::ConstPtr& ros, wind::cpp::SREM_PDU_Descriptions::SREM* wind)
{

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;
    wind->srm.timeStampPresent =
        ros->srm.timeStampPresent;
    wind->srm.sequenceNumberPresent =
        ros->srm.sequenceNumberPresent;
    wind->srm.requestsPresent =
        ros->srm.requestsPresent;
    if(ros->srm.timeStampPresent) {

        wind->srm.timeStamp.value =
            ros->srm.timeStamp.value;
    }

    wind->srm.second.value =
        ros->srm.second.value;
    if(ros->srm.sequenceNumberPresent) {

        wind->srm.sequenceNumber.value =
            ros->srm.sequenceNumber.value;
    }
    if(ros->srm.requestsPresent) {

        // Start SEQUENCE OF SignalRequestList
        wind->srm.requests.count =
            ros->srm.requests.count;

        int count_a = ros->srm.requests.count;

        for(int a = 0; a < count_a; a++) {
            wind->srm.requests.elements[a].minutePresent =
                ros->srm.requests.elements[a].minutePresent;
            wind->srm.requests.elements[a].secondPresent =
                ros->srm.requests.elements[a].secondPresent;
            wind->srm.requests.elements[a].durationPresent =
                ros->srm.requests.elements[a].durationPresent;
            wind->srm.requests.elements[a].request.outBoundLanePresent =
                ros->srm.requests.elements[a].request.outBoundLanePresent;
            wind->srm.requests.elements[a].request.id.regionPresent =
                ros->srm.requests.elements[a].request.id.regionPresent;
            if(ros->srm.requests.elements[a].request.id.regionPresent) {

                wind->srm.requests.elements[a].request.id.region.value =
                    ros->srm.requests.elements[a].request.id.region.value;
            }

            wind->srm.requests.elements[a].request.id.id.value =
                ros->srm.requests.elements[a].request.id.id.value;

            wind->srm.requests.elements[a].request.requestID.value =
                ros->srm.requests.elements[a].request.requestID.value;

            wind->srm.requests.elements[a].request.requestType.value =
                ros->srm.requests.elements[a].request.requestType.value;

            // START CHOICE IntersectionAccessPoint
            wind->srm.requests.elements[a].request.inBoundLane.choice = ros->srm.requests.elements[a].request.inBoundLane.choice;

            if(ros->srm.requests.elements[a].request.inBoundLane.choice == 0) {

                wind->srm.requests.elements[a].request.inBoundLane.lane.value =
                    ros->srm.requests.elements[a].request.inBoundLane.lane.value;
            }
            else if(ros->srm.requests.elements[a].request.inBoundLane.choice == 1)  // CHOICE IntersectionAccessPoint
            {

                wind->srm.requests.elements[a].request.inBoundLane.approach.value =
                    ros->srm.requests.elements[a].request.inBoundLane.approach.value;
            }
            else   // CHOICE IntersectionAccessPoint
            {

                wind->srm.requests.elements[a].request.inBoundLane.connection.value =
                    ros->srm.requests.elements[a].request.inBoundLane.connection.value;
            }
            // END CHOICE IntersectionAccessPoint
            if(ros->srm.requests.elements[a].request.outBoundLanePresent) {

                // START CHOICE IntersectionAccessPoint
                wind->srm.requests.elements[a].request.outBoundLane.choice = ros->srm.requests.elements[a].request.outBoundLane.choice;

                if(ros->srm.requests.elements[a].request.outBoundLane.choice == 0) {

                    wind->srm.requests.elements[a].request.outBoundLane.lane.value =
                        ros->srm.requests.elements[a].request.outBoundLane.lane.value;
                }
                else if(ros->srm.requests.elements[a].request.outBoundLane.choice == 1)  // CHOICE IntersectionAccessPoint
                {

                    wind->srm.requests.elements[a].request.outBoundLane.approach.value =
                        ros->srm.requests.elements[a].request.outBoundLane.approach.value;
                }
                else   // CHOICE IntersectionAccessPoint
                {

                    wind->srm.requests.elements[a].request.outBoundLane.connection.value =
                        ros->srm.requests.elements[a].request.outBoundLane.connection.value;
                }
                // END CHOICE IntersectionAccessPoint
            }
            if(ros->srm.requests.elements[a].minutePresent) {

                wind->srm.requests.elements[a].minute.value =
                    ros->srm.requests.elements[a].minute.value;
            }
            if(ros->srm.requests.elements[a].secondPresent) {

                wind->srm.requests.elements[a].second.value =
                    ros->srm.requests.elements[a].second.value;
            }
            if(ros->srm.requests.elements[a].durationPresent) {

                wind->srm.requests.elements[a].duration.value =
                    ros->srm.requests.elements[a].duration.value;
            }
        }
        // End Sequence of SignalRequestList
    }
    wind->srm.requestor.typePresent =
        ros->srm.requestor.typePresent;
    wind->srm.requestor.positionPresent =
        ros->srm.requestor.positionPresent;
    wind->srm.requestor.namePresent =
        ros->srm.requestor.namePresent;
    wind->srm.requestor.routeNamePresent =
        ros->srm.requestor.routeNamePresent;
    wind->srm.requestor.transitStatusPresent =
        ros->srm.requestor.transitStatusPresent;
    wind->srm.requestor.transitOccupancyPresent =
        ros->srm.requestor.transitOccupancyPresent;
    wind->srm.requestor.transitSchedulePresent =
        ros->srm.requestor.transitSchedulePresent;

    // START CHOICE VehicleID
    wind->srm.requestor.id.choice = ros->srm.requestor.id.choice;

    if(ros->srm.requestor.id.choice == 0) {

wind->srm.requestor.id.entityID.count =
    ros->srm.requestor.id.entityID.count;
for(int b = 0; b < 4; b++)
    wind->srm.requestor.id.entityID.value[b] =
        ros->srm.requestor.id.entityID.value[b];
    }
    else   // CHOICE VehicleID
    {

        wind->srm.requestor.id.stationID.value =
            ros->srm.requestor.id.stationID.value;
    }
    // END CHOICE VehicleID
    if(ros->srm.requestor.typePresent) {
        wind->srm.requestor.type.subrolePresent =
            ros->srm.requestor.type.subrolePresent;
        wind->srm.requestor.type.requestPresent =
            ros->srm.requestor.type.requestPresent;
        wind->srm.requestor.type.iso3883Present =
            ros->srm.requestor.type.iso3883Present;
        wind->srm.requestor.type.hpmsTypePresent =
            ros->srm.requestor.type.hpmsTypePresent;

        wind->srm.requestor.type.role.value =
            ros->srm.requestor.type.role.value;
        if(ros->srm.requestor.type.subrolePresent) {

            wind->srm.requestor.type.subrole.value =
                ros->srm.requestor.type.subrole.value;
        }
        if(ros->srm.requestor.type.requestPresent) {

            wind->srm.requestor.type.request.value =
                ros->srm.requestor.type.request.value;
        }
        if(ros->srm.requestor.type.iso3883Present) {

            wind->srm.requestor.type.iso3883.value =
                ros->srm.requestor.type.iso3883.value;
        }
        if(ros->srm.requestor.type.hpmsTypePresent) {

            wind->srm.requestor.type.hpmsType.value =
                ros->srm.requestor.type.hpmsType.value;
        }
    }
    if(ros->srm.requestor.positionPresent) {
        wind->srm.requestor.position.headingPresent =
            ros->srm.requestor.position.headingPresent;
        wind->srm.requestor.position.speedPresent =
            ros->srm.requestor.position.speedPresent;
        wind->srm.requestor.position.position.elevationPresent =
            ros->srm.requestor.position.position.elevationPresent;

        wind->srm.requestor.position.position.lat.value =
            ros->srm.requestor.position.position.lat.value;

        wind->srm.requestor.position.position.long_.value =
            ros->srm.requestor.position.position.long_.value;
        if(ros->srm.requestor.position.position.elevationPresent) {

            wind->srm.requestor.position.position.elevation.value =
                ros->srm.requestor.position.position.elevation.value;
        }
        if(ros->srm.requestor.position.headingPresent) {

            wind->srm.requestor.position.heading.value =
                ros->srm.requestor.position.heading.value;
        }
        if(ros->srm.requestor.position.speedPresent) {

            wind->srm.requestor.position.speed.transmisson.value =
                ros->srm.requestor.position.speed.transmisson.value;

            wind->srm.requestor.position.speed.speed.value =
                ros->srm.requestor.position.speed.speed.value;
        }
    }
    if(ros->srm.requestor.namePresent) {


        for(int c = 0; c < 63; c++) {  // DescriptiveName
            if(c < ros->srm.requestor.name.value.length())
                wind->srm.requestor.name.value[c] = ros->srm.requestor.name.value.c_str()[c];
            else
                wind->srm.requestor.name.value[c] = ' ';
        }
    }
    if(ros->srm.requestor.routeNamePresent) {


        for(int d = 0; d < 63; d++) {  // DescriptiveName
            if(d < ros->srm.requestor.routeName.value.length())
                wind->srm.requestor.routeName.value[d] = ros->srm.requestor.routeName.value.c_str()[d];
            else
                wind->srm.requestor.routeName.value[d] = ' ';
        }
    }
    if(ros->srm.requestor.transitStatusPresent) {
        
        // START BIT STRING TransitVehicleStatus
        for(int e = 0; e < 8; e++) {
            wind->srm.requestor.transitStatus.values[e] = 
                ros->srm.requestor.transitStatus.values[e];
        }
        // END BIT STRING TransitVehicleStatus
        
    }
    if(ros->srm.requestor.transitOccupancyPresent) {

        wind->srm.requestor.transitOccupancy.value =
            ros->srm.requestor.transitOccupancy.value;
    }
    if(ros->srm.requestor.transitSchedulePresent) {

        wind->srm.requestor.transitSchedule.value =
            ros->srm.requestor.transitSchedule.value;
    }
}
