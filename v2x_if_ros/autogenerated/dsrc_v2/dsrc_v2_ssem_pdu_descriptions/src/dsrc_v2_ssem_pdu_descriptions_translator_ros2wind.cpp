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
 * Module: SSEM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(103301) ssem(4) version2(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <dsrc_v2_ssem_pdu_descriptions_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const dsrc_v2_ssem_pdu_descriptions::SSEM::ConstPtr& ros, wind::cpp::SSEM_PDU_Descriptions::SSEM* wind)
{

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;
    wind->ssm.timeStampPresent =
        ros->ssm.timeStampPresent;
    wind->ssm.sequenceNumberPresent =
        ros->ssm.sequenceNumberPresent;
    if(ros->ssm.timeStampPresent) {

        wind->ssm.timeStamp.value =
            ros->ssm.timeStamp.value;
    }

    wind->ssm.second.value =
        ros->ssm.second.value;
    if(ros->ssm.sequenceNumberPresent) {

        wind->ssm.sequenceNumber.value =
            ros->ssm.sequenceNumber.value;
    }

    // Start SEQUENCE OF SignalStatusList
    wind->ssm.status.count =
        ros->ssm.status.count;

    int count_a = ros->ssm.status.count;

    for(int a = 0; a < count_a; a++) {

        wind->ssm.status.elements[a].sequenceNumber.value =
            ros->ssm.status.elements[a].sequenceNumber.value;
        wind->ssm.status.elements[a].id.regionPresent =
            ros->ssm.status.elements[a].id.regionPresent;
        if(ros->ssm.status.elements[a].id.regionPresent) {

            wind->ssm.status.elements[a].id.region.value =
                ros->ssm.status.elements[a].id.region.value;
        }

        wind->ssm.status.elements[a].id.id.value =
            ros->ssm.status.elements[a].id.id.value;

        // Start SEQUENCE OF SignalStatusPackageList
        wind->ssm.status.elements[a].sigStatus.count =
            ros->ssm.status.elements[a].sigStatus.count;

        int count_b = ros->ssm.status.elements[a].sigStatus.count;

        for(int b = 0; b < count_b; b++) {
            wind->ssm.status.elements[a].sigStatus.elements[b].requesterPresent =
                ros->ssm.status.elements[a].sigStatus.elements[b].requesterPresent;
            wind->ssm.status.elements[a].sigStatus.elements[b].outboundOnPresent =
                ros->ssm.status.elements[a].sigStatus.elements[b].outboundOnPresent;
            wind->ssm.status.elements[a].sigStatus.elements[b].minutePresent =
                ros->ssm.status.elements[a].sigStatus.elements[b].minutePresent;
            wind->ssm.status.elements[a].sigStatus.elements[b].secondPresent =
                ros->ssm.status.elements[a].sigStatus.elements[b].secondPresent;
            wind->ssm.status.elements[a].sigStatus.elements[b].durationPresent =
                ros->ssm.status.elements[a].sigStatus.elements[b].durationPresent;
            if(ros->ssm.status.elements[a].sigStatus.elements[b].requesterPresent) {
                wind->ssm.status.elements[a].sigStatus.elements[b].requester.rolePresent =
                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.rolePresent;
                wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeDataPresent =
                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeDataPresent;

                // START CHOICE VehicleID
                wind->ssm.status.elements[a].sigStatus.elements[b].requester.id.choice = ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.choice;

                if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.choice == 0) {

wind->ssm.status.elements[a].sigStatus.elements[b].requester.id.entityID.count =
    ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.entityID.count;
for(int c = 0; c < 4; c++)
    wind->ssm.status.elements[a].sigStatus.elements[b].requester.id.entityID.value[c] =
        ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.entityID.value[c];
                }
                else   // CHOICE VehicleID
                {

                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.id.stationID.value =
                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.stationID.value;
                }
                // END CHOICE VehicleID

                wind->ssm.status.elements[a].sigStatus.elements[b].requester.request.value =
                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.request.value;

                wind->ssm.status.elements[a].sigStatus.elements[b].requester.sequenceNumber.value =
                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.sequenceNumber.value;
                if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.rolePresent) {

                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.role.value =
                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.role.value;
                }
                if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeDataPresent) {
                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrolePresent =
                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrolePresent;
                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.requestPresent =
                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.requestPresent;
                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883Present =
                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883Present;
                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsTypePresent =
                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsTypePresent;

                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.role.value =
                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.role.value;
                    if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrolePresent) {

                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrole.value =
                            ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrole.value;
                    }
                    if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.requestPresent) {

                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.request.value =
                            ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.request.value;
                    }
                    if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883Present) {

                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883.value =
                            ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883.value;
                    }
                    if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsTypePresent) {

                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsType.value =
                            ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsType.value;
                    }
                }
            }

            // START CHOICE IntersectionAccessPoint
            wind->ssm.status.elements[a].sigStatus.elements[b].inboundOn.choice = ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.choice;

            if(ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.choice == 0) {

                wind->ssm.status.elements[a].sigStatus.elements[b].inboundOn.lane.value =
                    ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.lane.value;
            }
            else if(ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.choice == 1)  // CHOICE IntersectionAccessPoint
            {

                wind->ssm.status.elements[a].sigStatus.elements[b].inboundOn.approach.value =
                    ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.approach.value;
            }
            else   // CHOICE IntersectionAccessPoint
            {

                wind->ssm.status.elements[a].sigStatus.elements[b].inboundOn.connection.value =
                    ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.connection.value;
            }
            // END CHOICE IntersectionAccessPoint
            if(ros->ssm.status.elements[a].sigStatus.elements[b].outboundOnPresent) {

                // START CHOICE IntersectionAccessPoint
                wind->ssm.status.elements[a].sigStatus.elements[b].outboundOn.choice = ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.choice;

                if(ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.choice == 0) {

                    wind->ssm.status.elements[a].sigStatus.elements[b].outboundOn.lane.value =
                        ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.lane.value;
                }
                else if(ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.choice == 1)  // CHOICE IntersectionAccessPoint
                {

                    wind->ssm.status.elements[a].sigStatus.elements[b].outboundOn.approach.value =
                        ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.approach.value;
                }
                else   // CHOICE IntersectionAccessPoint
                {

                    wind->ssm.status.elements[a].sigStatus.elements[b].outboundOn.connection.value =
                        ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.connection.value;
                }
                // END CHOICE IntersectionAccessPoint
            }
            if(ros->ssm.status.elements[a].sigStatus.elements[b].minutePresent) {

                wind->ssm.status.elements[a].sigStatus.elements[b].minute.value =
                    ros->ssm.status.elements[a].sigStatus.elements[b].minute.value;
            }
            if(ros->ssm.status.elements[a].sigStatus.elements[b].secondPresent) {

                wind->ssm.status.elements[a].sigStatus.elements[b].second.value =
                    ros->ssm.status.elements[a].sigStatus.elements[b].second.value;
            }
            if(ros->ssm.status.elements[a].sigStatus.elements[b].durationPresent) {

                wind->ssm.status.elements[a].sigStatus.elements[b].duration.value =
                    ros->ssm.status.elements[a].sigStatus.elements[b].duration.value;
            }

            wind->ssm.status.elements[a].sigStatus.elements[b].status.value =
                ros->ssm.status.elements[a].sigStatus.elements[b].status.value;
        }
        // End Sequence of SignalStatusPackageList
    }
    // End Sequence of SignalStatusList
}
