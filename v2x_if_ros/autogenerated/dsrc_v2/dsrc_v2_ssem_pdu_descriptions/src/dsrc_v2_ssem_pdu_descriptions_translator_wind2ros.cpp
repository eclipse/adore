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

#include <dsrc_v2_ssem_pdu_descriptions_translator_wind2ros.h>

void wind::wind_ros::wind2ros(dsrc_v2_ssem_pdu_descriptions::SSEM* ros, wind::cpp::SSEM_PDU_Descriptions::SSEM* wind)
{

    ros->header.protocolVersion.value =
        wind->header.protocolVersion.value;

    ros->header.messageID.value =
        wind->header.messageID.value;

    ros->header.stationID.value =
        wind->header.stationID.value;
    ros->ssm.timeStampPresent =
        wind->ssm.timeStampPresent;
    ros->ssm.sequenceNumberPresent =
        wind->ssm.sequenceNumberPresent;
    if(ros->ssm.timeStampPresent) {

        ros->ssm.timeStamp.value =
            wind->ssm.timeStamp.value;
    }

    ros->ssm.second.value =
        wind->ssm.second.value;
    if(ros->ssm.sequenceNumberPresent) {

        ros->ssm.sequenceNumber.value =
            wind->ssm.sequenceNumber.value;
    }

    // Start SEQUENCE OF SignalStatusList
    ros->ssm.status.count =
        wind->ssm.status.count;

    int count_a = ros->ssm.status.count;

    for(int a = 0; a < count_a; a++) {
        dsrc_v2_dsrc::SignalStatus tmp_a;
        ros->ssm.status.elements.push_back(tmp_a);

        ros->ssm.status.elements[a].sequenceNumber.value =
            wind->ssm.status.elements[a].sequenceNumber.value;
        ros->ssm.status.elements[a].id.regionPresent =
            wind->ssm.status.elements[a].id.regionPresent;
        if(ros->ssm.status.elements[a].id.regionPresent) {

            ros->ssm.status.elements[a].id.region.value =
                wind->ssm.status.elements[a].id.region.value;
        }

        ros->ssm.status.elements[a].id.id.value =
            wind->ssm.status.elements[a].id.id.value;

        // Start SEQUENCE OF SignalStatusPackageList
        ros->ssm.status.elements[a].sigStatus.count =
            wind->ssm.status.elements[a].sigStatus.count;

        int count_b = ros->ssm.status.elements[a].sigStatus.count;

        for(int b = 0; b < count_b; b++) {
            dsrc_v2_dsrc::SignalStatusPackage tmp_b;
            ros->ssm.status.elements[a].sigStatus.elements.push_back(tmp_b);
            ros->ssm.status.elements[a].sigStatus.elements[b].requesterPresent =
                wind->ssm.status.elements[a].sigStatus.elements[b].requesterPresent;
            ros->ssm.status.elements[a].sigStatus.elements[b].outboundOnPresent =
                wind->ssm.status.elements[a].sigStatus.elements[b].outboundOnPresent;
            ros->ssm.status.elements[a].sigStatus.elements[b].minutePresent =
                wind->ssm.status.elements[a].sigStatus.elements[b].minutePresent;
            ros->ssm.status.elements[a].sigStatus.elements[b].secondPresent =
                wind->ssm.status.elements[a].sigStatus.elements[b].secondPresent;
            ros->ssm.status.elements[a].sigStatus.elements[b].durationPresent =
                wind->ssm.status.elements[a].sigStatus.elements[b].durationPresent;
            if(ros->ssm.status.elements[a].sigStatus.elements[b].requesterPresent) {
                ros->ssm.status.elements[a].sigStatus.elements[b].requester.rolePresent =
                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.rolePresent;
                ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeDataPresent =
                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeDataPresent;

                // START CHOICE VehicleID
                ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.choice = wind->ssm.status.elements[a].sigStatus.elements[b].requester.id.choice;

                if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.choice == 0) {

ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.entityID.count =
    wind->ssm.status.elements[a].sigStatus.elements[b].requester.id.entityID.count;
for(int c = 0; c < 4; c++)
    ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.entityID.value[c] =
        wind->ssm.status.elements[a].sigStatus.elements[b].requester.id.entityID.value[c];
                }
                else   // CHOICE VehicleID
                {

                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.id.stationID.value =
                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.id.stationID.value;
                }
                // END CHOICE VehicleID

                ros->ssm.status.elements[a].sigStatus.elements[b].requester.request.value =
                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.request.value;

                ros->ssm.status.elements[a].sigStatus.elements[b].requester.sequenceNumber.value =
                    wind->ssm.status.elements[a].sigStatus.elements[b].requester.sequenceNumber.value;
                if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.rolePresent) {

                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.role.value =
                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.role.value;
                }
                if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeDataPresent) {
                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrolePresent =
                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrolePresent;
                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.requestPresent =
                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.requestPresent;
                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883Present =
                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883Present;
                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsTypePresent =
                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsTypePresent;

                    ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.role.value =
                        wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.role.value;
                    if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrolePresent) {

                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrole.value =
                            wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.subrole.value;
                    }
                    if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.requestPresent) {

                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.request.value =
                            wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.request.value;
                    }
                    if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883Present) {

                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883.value =
                            wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.iso3883.value;
                    }
                    if(ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsTypePresent) {

                        ros->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsType.value =
                            wind->ssm.status.elements[a].sigStatus.elements[b].requester.typeData.hpmsType.value;
                    }
                }
            }

            // START CHOICE IntersectionAccessPoint
            ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.choice = wind->ssm.status.elements[a].sigStatus.elements[b].inboundOn.choice;

            if(ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.choice == 0) {

                ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.lane.value =
                    wind->ssm.status.elements[a].sigStatus.elements[b].inboundOn.lane.value;
            }
            else if(ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.choice == 1)  // CHOICE IntersectionAccessPoint
            {

                ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.approach.value =
                    wind->ssm.status.elements[a].sigStatus.elements[b].inboundOn.approach.value;
            }
            else   // CHOICE IntersectionAccessPoint
            {

                ros->ssm.status.elements[a].sigStatus.elements[b].inboundOn.connection.value =
                    wind->ssm.status.elements[a].sigStatus.elements[b].inboundOn.connection.value;
            }
            // END CHOICE IntersectionAccessPoint
            if(ros->ssm.status.elements[a].sigStatus.elements[b].outboundOnPresent) {

                // START CHOICE IntersectionAccessPoint
                ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.choice = wind->ssm.status.elements[a].sigStatus.elements[b].outboundOn.choice;

                if(ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.choice == 0) {

                    ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.lane.value =
                        wind->ssm.status.elements[a].sigStatus.elements[b].outboundOn.lane.value;
                }
                else if(ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.choice == 1)  // CHOICE IntersectionAccessPoint
                {

                    ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.approach.value =
                        wind->ssm.status.elements[a].sigStatus.elements[b].outboundOn.approach.value;
                }
                else   // CHOICE IntersectionAccessPoint
                {

                    ros->ssm.status.elements[a].sigStatus.elements[b].outboundOn.connection.value =
                        wind->ssm.status.elements[a].sigStatus.elements[b].outboundOn.connection.value;
                }
                // END CHOICE IntersectionAccessPoint
            }
            if(ros->ssm.status.elements[a].sigStatus.elements[b].minutePresent) {

                ros->ssm.status.elements[a].sigStatus.elements[b].minute.value =
                    wind->ssm.status.elements[a].sigStatus.elements[b].minute.value;
            }
            if(ros->ssm.status.elements[a].sigStatus.elements[b].secondPresent) {

                ros->ssm.status.elements[a].sigStatus.elements[b].second.value =
                    wind->ssm.status.elements[a].sigStatus.elements[b].second.value;
            }
            if(ros->ssm.status.elements[a].sigStatus.elements[b].durationPresent) {

                ros->ssm.status.elements[a].sigStatus.elements[b].duration.value =
                    wind->ssm.status.elements[a].sigStatus.elements[b].duration.value;
            }

            ros->ssm.status.elements[a].sigStatus.elements[b].status.value =
                wind->ssm.status.elements[a].sigStatus.elements[b].status.value;
        }
        // End Sequence of SignalStatusPackageList
    }
    // End Sequence of SignalStatusList
}
