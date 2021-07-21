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

#include <dsrc_v2_srem_pdu_descriptions_translator_wind2ros.h>

void wind::wind_ros::wind2ros(dsrc_v2_srem_pdu_descriptions::SREM* ros, wind::cpp::SREM_PDU_Descriptions::SREM* wind)
{

    ros->header.protocolVersion.value =
        wind->header.protocolVersion.value;

    ros->header.messageID.value =
        wind->header.messageID.value;

    ros->header.stationID.value =
        wind->header.stationID.value;
    ros->srm.timeStampPresent =
        wind->srm.timeStampPresent;
    ros->srm.sequenceNumberPresent =
        wind->srm.sequenceNumberPresent;
    ros->srm.requestsPresent =
        wind->srm.requestsPresent;
    if(ros->srm.timeStampPresent) {

        ros->srm.timeStamp.value =
            wind->srm.timeStamp.value;
    }

    ros->srm.second.value =
        wind->srm.second.value;
    if(ros->srm.sequenceNumberPresent) {

        ros->srm.sequenceNumber.value =
            wind->srm.sequenceNumber.value;
    }
    if(ros->srm.requestsPresent) {

        // Start SEQUENCE OF SignalRequestList
        ros->srm.requests.count =
            wind->srm.requests.count;

        int count_a = ros->srm.requests.count;

        for(int a = 0; a < count_a; a++) {
            dsrc_v2_dsrc::SignalRequestPackage tmp_a;
            ros->srm.requests.elements.push_back(tmp_a);
            ros->srm.requests.elements[a].minutePresent =
                wind->srm.requests.elements[a].minutePresent;
            ros->srm.requests.elements[a].secondPresent =
                wind->srm.requests.elements[a].secondPresent;
            ros->srm.requests.elements[a].durationPresent =
                wind->srm.requests.elements[a].durationPresent;
            ros->srm.requests.elements[a].request.outBoundLanePresent =
                wind->srm.requests.elements[a].request.outBoundLanePresent;
            ros->srm.requests.elements[a].request.id.regionPresent =
                wind->srm.requests.elements[a].request.id.regionPresent;
            if(ros->srm.requests.elements[a].request.id.regionPresent) {

                ros->srm.requests.elements[a].request.id.region.value =
                    wind->srm.requests.elements[a].request.id.region.value;
            }

            ros->srm.requests.elements[a].request.id.id.value =
                wind->srm.requests.elements[a].request.id.id.value;

            ros->srm.requests.elements[a].request.requestID.value =
                wind->srm.requests.elements[a].request.requestID.value;

            ros->srm.requests.elements[a].request.requestType.value =
                wind->srm.requests.elements[a].request.requestType.value;

            // START CHOICE IntersectionAccessPoint
            ros->srm.requests.elements[a].request.inBoundLane.choice = wind->srm.requests.elements[a].request.inBoundLane.choice;

            if(ros->srm.requests.elements[a].request.inBoundLane.choice == 0) {

                ros->srm.requests.elements[a].request.inBoundLane.lane.value =
                    wind->srm.requests.elements[a].request.inBoundLane.lane.value;
            }
            else if(ros->srm.requests.elements[a].request.inBoundLane.choice == 1)  // CHOICE IntersectionAccessPoint
            {

                ros->srm.requests.elements[a].request.inBoundLane.approach.value =
                    wind->srm.requests.elements[a].request.inBoundLane.approach.value;
            }
            else   // CHOICE IntersectionAccessPoint
            {

                ros->srm.requests.elements[a].request.inBoundLane.connection.value =
                    wind->srm.requests.elements[a].request.inBoundLane.connection.value;
            }
            // END CHOICE IntersectionAccessPoint
            if(ros->srm.requests.elements[a].request.outBoundLanePresent) {

                // START CHOICE IntersectionAccessPoint
                ros->srm.requests.elements[a].request.outBoundLane.choice = wind->srm.requests.elements[a].request.outBoundLane.choice;

                if(ros->srm.requests.elements[a].request.outBoundLane.choice == 0) {

                    ros->srm.requests.elements[a].request.outBoundLane.lane.value =
                        wind->srm.requests.elements[a].request.outBoundLane.lane.value;
                }
                else if(ros->srm.requests.elements[a].request.outBoundLane.choice == 1)  // CHOICE IntersectionAccessPoint
                {

                    ros->srm.requests.elements[a].request.outBoundLane.approach.value =
                        wind->srm.requests.elements[a].request.outBoundLane.approach.value;
                }
                else   // CHOICE IntersectionAccessPoint
                {

                    ros->srm.requests.elements[a].request.outBoundLane.connection.value =
                        wind->srm.requests.elements[a].request.outBoundLane.connection.value;
                }
                // END CHOICE IntersectionAccessPoint
            }
            if(ros->srm.requests.elements[a].minutePresent) {

                ros->srm.requests.elements[a].minute.value =
                    wind->srm.requests.elements[a].minute.value;
            }
            if(ros->srm.requests.elements[a].secondPresent) {

                ros->srm.requests.elements[a].second.value =
                    wind->srm.requests.elements[a].second.value;
            }
            if(ros->srm.requests.elements[a].durationPresent) {

                ros->srm.requests.elements[a].duration.value =
                    wind->srm.requests.elements[a].duration.value;
            }
        }
        // End Sequence of SignalRequestList
    }
    ros->srm.requestor.typePresent =
        wind->srm.requestor.typePresent;
    ros->srm.requestor.positionPresent =
        wind->srm.requestor.positionPresent;
    ros->srm.requestor.namePresent =
        wind->srm.requestor.namePresent;
    ros->srm.requestor.routeNamePresent =
        wind->srm.requestor.routeNamePresent;
    ros->srm.requestor.transitStatusPresent =
        wind->srm.requestor.transitStatusPresent;
    ros->srm.requestor.transitOccupancyPresent =
        wind->srm.requestor.transitOccupancyPresent;
    ros->srm.requestor.transitSchedulePresent =
        wind->srm.requestor.transitSchedulePresent;

    // START CHOICE VehicleID
    ros->srm.requestor.id.choice = wind->srm.requestor.id.choice;

    if(ros->srm.requestor.id.choice == 0) {

ros->srm.requestor.id.entityID.count =
    wind->srm.requestor.id.entityID.count;
for(int b = 0; b < 4; b++)
    ros->srm.requestor.id.entityID.value[b] =
        wind->srm.requestor.id.entityID.value[b];
    }
    else   // CHOICE VehicleID
    {

        ros->srm.requestor.id.stationID.value =
            wind->srm.requestor.id.stationID.value;
    }
    // END CHOICE VehicleID
    if(ros->srm.requestor.typePresent) {
        ros->srm.requestor.type.subrolePresent =
            wind->srm.requestor.type.subrolePresent;
        ros->srm.requestor.type.requestPresent =
            wind->srm.requestor.type.requestPresent;
        ros->srm.requestor.type.iso3883Present =
            wind->srm.requestor.type.iso3883Present;
        ros->srm.requestor.type.hpmsTypePresent =
            wind->srm.requestor.type.hpmsTypePresent;

        ros->srm.requestor.type.role.value =
            wind->srm.requestor.type.role.value;
        if(ros->srm.requestor.type.subrolePresent) {

            ros->srm.requestor.type.subrole.value =
                wind->srm.requestor.type.subrole.value;
        }
        if(ros->srm.requestor.type.requestPresent) {

            ros->srm.requestor.type.request.value =
                wind->srm.requestor.type.request.value;
        }
        if(ros->srm.requestor.type.iso3883Present) {

            ros->srm.requestor.type.iso3883.value =
                wind->srm.requestor.type.iso3883.value;
        }
        if(ros->srm.requestor.type.hpmsTypePresent) {

            ros->srm.requestor.type.hpmsType.value =
                wind->srm.requestor.type.hpmsType.value;
        }
    }
    if(ros->srm.requestor.positionPresent) {
        ros->srm.requestor.position.headingPresent =
            wind->srm.requestor.position.headingPresent;
        ros->srm.requestor.position.speedPresent =
            wind->srm.requestor.position.speedPresent;
        ros->srm.requestor.position.position.elevationPresent =
            wind->srm.requestor.position.position.elevationPresent;

        ros->srm.requestor.position.position.lat.value =
            wind->srm.requestor.position.position.lat.value;

        ros->srm.requestor.position.position.long_.value =
            wind->srm.requestor.position.position.long_.value;
        if(ros->srm.requestor.position.position.elevationPresent) {

            ros->srm.requestor.position.position.elevation.value =
                wind->srm.requestor.position.position.elevation.value;
        }
        if(ros->srm.requestor.position.headingPresent) {

            ros->srm.requestor.position.heading.value =
                wind->srm.requestor.position.heading.value;
        }
        if(ros->srm.requestor.position.speedPresent) {

            ros->srm.requestor.position.speed.transmisson.value =
                wind->srm.requestor.position.speed.transmisson.value;

            ros->srm.requestor.position.speed.speed.value =
                wind->srm.requestor.position.speed.speed.value;
        }
    }
    if(ros->srm.requestor.namePresent) {

        ros->srm.requestor.name.value.assign(wind->srm.requestor.name.value);
    }
    if(ros->srm.requestor.routeNamePresent) {

        ros->srm.requestor.routeName.value.assign(wind->srm.requestor.routeName.value);
    }
    if(ros->srm.requestor.transitStatusPresent) {
        
        // START BIT STRING TransitVehicleStatus
        for(int c = 0; c < 8; c++) {
            uint8_t tmp_c;
            ros->srm.requestor.transitStatus.values.push_back(tmp_c);
            ros->srm.requestor.transitStatus.values[c] = 
                wind->srm.requestor.transitStatus.values[c];
        }
        // END BIT STRING TransitVehicleStatus
        
    }
    if(ros->srm.requestor.transitOccupancyPresent) {

        ros->srm.requestor.transitOccupancy.value =
            wind->srm.requestor.transitOccupancy.value;
    }
    if(ros->srm.requestor.transitSchedulePresent) {

        ros->srm.requestor.transitSchedule.value =
            wind->srm.requestor.transitSchedule.value;
    }
}
