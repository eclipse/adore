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
 * Module: RTCMEM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(103301) rtcmem(5) version1(1)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <dsrc_v2_rtcmem_pdu_descriptions_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const dsrc_v2_rtcmem_pdu_descriptions::RTCMEM::ConstPtr& ros, wind::cpp::RTCMEM_PDU_Descriptions::RTCMEM* wind)
{

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;
    wind->rtcmc.timeStampPresent =
        ros->rtcmc.timeStampPresent;
    wind->rtcmc.anchorPointPresent =
        ros->rtcmc.anchorPointPresent;
    wind->rtcmc.rtcmHeaderPresent =
        ros->rtcmc.rtcmHeaderPresent;

    wind->rtcmc.msgCnt.value =
        ros->rtcmc.msgCnt.value;

    wind->rtcmc.rev.value =
        ros->rtcmc.rev.value;
    if(ros->rtcmc.timeStampPresent) {

        wind->rtcmc.timeStamp.value =
            ros->rtcmc.timeStamp.value;
    }
    if(ros->rtcmc.anchorPointPresent) {
        wind->rtcmc.anchorPoint.utcTimePresent =
            ros->rtcmc.anchorPoint.utcTimePresent;
        wind->rtcmc.anchorPoint.elevationPresent =
            ros->rtcmc.anchorPoint.elevationPresent;
        wind->rtcmc.anchorPoint.headingPresent =
            ros->rtcmc.anchorPoint.headingPresent;
        wind->rtcmc.anchorPoint.speedPresent =
            ros->rtcmc.anchorPoint.speedPresent;
        wind->rtcmc.anchorPoint.posAccuracyPresent =
            ros->rtcmc.anchorPoint.posAccuracyPresent;
        wind->rtcmc.anchorPoint.timeConfidencePresent =
            ros->rtcmc.anchorPoint.timeConfidencePresent;
        wind->rtcmc.anchorPoint.posConfidencePresent =
            ros->rtcmc.anchorPoint.posConfidencePresent;
        wind->rtcmc.anchorPoint.speedConfidencePresent =
            ros->rtcmc.anchorPoint.speedConfidencePresent;
        if(ros->rtcmc.anchorPoint.utcTimePresent) {
            wind->rtcmc.anchorPoint.utcTime.yearPresent =
                ros->rtcmc.anchorPoint.utcTime.yearPresent;
            wind->rtcmc.anchorPoint.utcTime.monthPresent =
                ros->rtcmc.anchorPoint.utcTime.monthPresent;
            wind->rtcmc.anchorPoint.utcTime.dayPresent =
                ros->rtcmc.anchorPoint.utcTime.dayPresent;
            wind->rtcmc.anchorPoint.utcTime.hourPresent =
                ros->rtcmc.anchorPoint.utcTime.hourPresent;
            wind->rtcmc.anchorPoint.utcTime.minutePresent =
                ros->rtcmc.anchorPoint.utcTime.minutePresent;
            wind->rtcmc.anchorPoint.utcTime.secondPresent =
                ros->rtcmc.anchorPoint.utcTime.secondPresent;
            wind->rtcmc.anchorPoint.utcTime.offsetPresent =
                ros->rtcmc.anchorPoint.utcTime.offsetPresent;
            if(ros->rtcmc.anchorPoint.utcTime.yearPresent) {

                wind->rtcmc.anchorPoint.utcTime.year.value =
                    ros->rtcmc.anchorPoint.utcTime.year.value;
            }
            if(ros->rtcmc.anchorPoint.utcTime.monthPresent) {

                wind->rtcmc.anchorPoint.utcTime.month.value =
                    ros->rtcmc.anchorPoint.utcTime.month.value;
            }
            if(ros->rtcmc.anchorPoint.utcTime.dayPresent) {

                wind->rtcmc.anchorPoint.utcTime.day.value =
                    ros->rtcmc.anchorPoint.utcTime.day.value;
            }
            if(ros->rtcmc.anchorPoint.utcTime.hourPresent) {

                wind->rtcmc.anchorPoint.utcTime.hour.value =
                    ros->rtcmc.anchorPoint.utcTime.hour.value;
            }
            if(ros->rtcmc.anchorPoint.utcTime.minutePresent) {

                wind->rtcmc.anchorPoint.utcTime.minute.value =
                    ros->rtcmc.anchorPoint.utcTime.minute.value;
            }
            if(ros->rtcmc.anchorPoint.utcTime.secondPresent) {

                wind->rtcmc.anchorPoint.utcTime.second.value =
                    ros->rtcmc.anchorPoint.utcTime.second.value;
            }
            if(ros->rtcmc.anchorPoint.utcTime.offsetPresent) {

                wind->rtcmc.anchorPoint.utcTime.offset.value =
                    ros->rtcmc.anchorPoint.utcTime.offset.value;
            }
        }

        wind->rtcmc.anchorPoint.long_.value =
            ros->rtcmc.anchorPoint.long_.value;

        wind->rtcmc.anchorPoint.lat.value =
            ros->rtcmc.anchorPoint.lat.value;
        if(ros->rtcmc.anchorPoint.elevationPresent) {

            wind->rtcmc.anchorPoint.elevation.value =
                ros->rtcmc.anchorPoint.elevation.value;
        }
        if(ros->rtcmc.anchorPoint.headingPresent) {

            wind->rtcmc.anchorPoint.heading.value =
                ros->rtcmc.anchorPoint.heading.value;
        }
        if(ros->rtcmc.anchorPoint.speedPresent) {

            wind->rtcmc.anchorPoint.speed.transmisson.value =
                ros->rtcmc.anchorPoint.speed.transmisson.value;

            wind->rtcmc.anchorPoint.speed.speed.value =
                ros->rtcmc.anchorPoint.speed.speed.value;
        }
        if(ros->rtcmc.anchorPoint.posAccuracyPresent) {

            wind->rtcmc.anchorPoint.posAccuracy.semiMajor.value =
                ros->rtcmc.anchorPoint.posAccuracy.semiMajor.value;

            wind->rtcmc.anchorPoint.posAccuracy.semiMinor.value =
                ros->rtcmc.anchorPoint.posAccuracy.semiMinor.value;

            wind->rtcmc.anchorPoint.posAccuracy.orientation.value =
                ros->rtcmc.anchorPoint.posAccuracy.orientation.value;
        }
        if(ros->rtcmc.anchorPoint.timeConfidencePresent) {

            wind->rtcmc.anchorPoint.timeConfidence.value =
                ros->rtcmc.anchorPoint.timeConfidence.value;
        }
        if(ros->rtcmc.anchorPoint.posConfidencePresent) {

            wind->rtcmc.anchorPoint.posConfidence.pos.value =
                ros->rtcmc.anchorPoint.posConfidence.pos.value;

            wind->rtcmc.anchorPoint.posConfidence.elevation.value =
                ros->rtcmc.anchorPoint.posConfidence.elevation.value;
        }
        if(ros->rtcmc.anchorPoint.speedConfidencePresent) {

            wind->rtcmc.anchorPoint.speedConfidence.heading.value =
                ros->rtcmc.anchorPoint.speedConfidence.heading.value;

            wind->rtcmc.anchorPoint.speedConfidence.speed.value =
                ros->rtcmc.anchorPoint.speedConfidence.speed.value;

            wind->rtcmc.anchorPoint.speedConfidence.throttle.value =
                ros->rtcmc.anchorPoint.speedConfidence.throttle.value;
        }
    }
    if(ros->rtcmc.rtcmHeaderPresent) {
        
        // START BIT STRING GNSSstatus
        for(int a = 0; a < 7; a++) {
            wind->rtcmc.rtcmHeader.status.values[a] = 
                ros->rtcmc.rtcmHeader.status.values[a];
        }
        // END BIT STRING GNSSstatus
        

        wind->rtcmc.rtcmHeader.offsetSet.antOffsetX.value =
            ros->rtcmc.rtcmHeader.offsetSet.antOffsetX.value;

        wind->rtcmc.rtcmHeader.offsetSet.antOffsetY.value =
            ros->rtcmc.rtcmHeader.offsetSet.antOffsetY.value;

        wind->rtcmc.rtcmHeader.offsetSet.antOffsetZ.value =
            ros->rtcmc.rtcmHeader.offsetSet.antOffsetZ.value;
    }

    // Start SEQUENCE OF RTCMmessageList
    wind->rtcmc.msgs.count =
        ros->rtcmc.msgs.count;

    int count_b = ros->rtcmc.msgs.count;

    for(int b = 0; b < count_b; b++) {

wind->rtcmc.msgs.elements[b].count =
    ros->rtcmc.msgs.elements[b].count;
for(int c = 0; c < 1023; c++)
    wind->rtcmc.msgs.elements[b].value[c] =
        ros->rtcmc.msgs.elements[b].value[c];
    }
    // End Sequence of RTCMmessageList
}
