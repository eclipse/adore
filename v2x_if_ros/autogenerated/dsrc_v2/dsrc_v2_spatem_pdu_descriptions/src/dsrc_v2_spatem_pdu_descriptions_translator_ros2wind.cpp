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
 * Module: SPATEM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(103301) spatem(0) version2(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <dsrc_v2_spatem_pdu_descriptions_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const dsrc_v2_spatem_pdu_descriptions::SPATEM::ConstPtr& ros, wind::cpp::SPATEM_PDU_Descriptions::SPATEM* wind)
{

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;
    wind->spat.timeStampPresent =
        ros->spat.timeStampPresent;
    wind->spat.namePresent =
        ros->spat.namePresent;
    if(ros->spat.timeStampPresent) {

        wind->spat.timeStamp.value =
            ros->spat.timeStamp.value;
    }
    if(ros->spat.namePresent) {


        for(int a = 0; a < 63; a++) {  // DescriptiveName
            if(a < ros->spat.name.value.length())
                wind->spat.name.value[a] = ros->spat.name.value.c_str()[a];
            else
                wind->spat.name.value[a] = ' ';
        }
    }

    // Start SEQUENCE OF IntersectionStateList
    wind->spat.intersections.count =
        ros->spat.intersections.count;

    int count_b = ros->spat.intersections.count;

    for(int b = 0; b < count_b; b++) {
        wind->spat.intersections.elements[b].namePresent =
            ros->spat.intersections.elements[b].namePresent;
        wind->spat.intersections.elements[b].moyPresent =
            ros->spat.intersections.elements[b].moyPresent;
        wind->spat.intersections.elements[b].timeStampPresent =
            ros->spat.intersections.elements[b].timeStampPresent;
        wind->spat.intersections.elements[b].enabledLanesPresent =
            ros->spat.intersections.elements[b].enabledLanesPresent;
        wind->spat.intersections.elements[b].maneuverAssistListPresent =
            ros->spat.intersections.elements[b].maneuverAssistListPresent;
        if(ros->spat.intersections.elements[b].namePresent) {


            for(int c = 0; c < 63; c++) {  // DescriptiveName
                if(c < ros->spat.intersections.elements[b].name.value.length())
                    wind->spat.intersections.elements[b].name.value[c] = ros->spat.intersections.elements[b].name.value.c_str()[c];
                else
                    wind->spat.intersections.elements[b].name.value[c] = ' ';
            }
        }
        wind->spat.intersections.elements[b].id.regionPresent =
            ros->spat.intersections.elements[b].id.regionPresent;
        if(ros->spat.intersections.elements[b].id.regionPresent) {

            wind->spat.intersections.elements[b].id.region.value =
                ros->spat.intersections.elements[b].id.region.value;
        }

        wind->spat.intersections.elements[b].id.id.value =
            ros->spat.intersections.elements[b].id.id.value;

        wind->spat.intersections.elements[b].revision.value =
            ros->spat.intersections.elements[b].revision.value;
        
        // START BIT STRING IntersectionStatusObject
        for(int d = 0; d < 16; d++) {
            wind->spat.intersections.elements[b].status.values[d] = 
                ros->spat.intersections.elements[b].status.values[d];
        }
        // END BIT STRING IntersectionStatusObject
        
        if(ros->spat.intersections.elements[b].moyPresent) {

            wind->spat.intersections.elements[b].moy.value =
                ros->spat.intersections.elements[b].moy.value;
        }
        if(ros->spat.intersections.elements[b].timeStampPresent) {

            wind->spat.intersections.elements[b].timeStamp.value =
                ros->spat.intersections.elements[b].timeStamp.value;
        }
        if(ros->spat.intersections.elements[b].enabledLanesPresent) {

            // Start SEQUENCE OF EnabledLaneList
            wind->spat.intersections.elements[b].enabledLanes.count =
                ros->spat.intersections.elements[b].enabledLanes.count;

            int count_e = ros->spat.intersections.elements[b].enabledLanes.count;

            for(int e = 0; e < count_e; e++) {

                wind->spat.intersections.elements[b].enabledLanes.elements[e].value =
                    ros->spat.intersections.elements[b].enabledLanes.elements[e].value;
            }
            // End Sequence of EnabledLaneList
        }

        // Start SEQUENCE OF MovementList
        wind->spat.intersections.elements[b].states.count =
            ros->spat.intersections.elements[b].states.count;

        int count_f = ros->spat.intersections.elements[b].states.count;

        for(int f = 0; f < count_f; f++) {
            wind->spat.intersections.elements[b].states.elements[f].movementNamePresent =
                ros->spat.intersections.elements[b].states.elements[f].movementNamePresent;
            wind->spat.intersections.elements[b].states.elements[f].maneuverAssistListPresent =
                ros->spat.intersections.elements[b].states.elements[f].maneuverAssistListPresent;
            if(ros->spat.intersections.elements[b].states.elements[f].movementNamePresent) {


                for(int g = 0; g < 63; g++) {  // DescriptiveName
                    if(g < ros->spat.intersections.elements[b].states.elements[f].movementName.value.length())
                        wind->spat.intersections.elements[b].states.elements[f].movementName.value[g] = ros->spat.intersections.elements[b].states.elements[f].movementName.value.c_str()[g];
                    else
                        wind->spat.intersections.elements[b].states.elements[f].movementName.value[g] = ' ';
                }
            }

            wind->spat.intersections.elements[b].states.elements[f].signalGroup.value =
                ros->spat.intersections.elements[b].states.elements[f].signalGroup.value;

            // Start SEQUENCE OF MovementEventList
            wind->spat.intersections.elements[b].states.elements[f].state_time_speed.count =
                ros->spat.intersections.elements[b].states.elements[f].state_time_speed.count;

            int count_h = ros->spat.intersections.elements[b].states.elements[f].state_time_speed.count;

            for(int h = 0; h < count_h; h++) {
                wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timingPresent =
                    ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timingPresent;
                wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speedsPresent =
                    ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speedsPresent;

                wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].eventState.value =
                    ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].eventState.value;
                if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timingPresent) {
                    wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.startTimePresent =
                        ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.startTimePresent;
                    wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.maxEndTimePresent =
                        ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.maxEndTimePresent;
                    wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.likelyTimePresent =
                        ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.likelyTimePresent;
                    wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.confidencePresent =
                        ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.confidencePresent;
                    wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.nextTimePresent =
                        ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.nextTimePresent;
                    if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.startTimePresent) {

                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.startTime.value =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.startTime.value;
                    }

                    wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.minEndTime.value =
                        ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.minEndTime.value;
                    if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.maxEndTimePresent) {

                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.maxEndTime.value =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.maxEndTime.value;
                    }
                    if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.likelyTimePresent) {

                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.likelyTime.value =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.likelyTime.value;
                    }
                    if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.confidencePresent) {

                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.confidence.value =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.confidence.value;
                    }
                    if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.nextTimePresent) {

                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.nextTime.value =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].timing.nextTime.value;
                    }
                }
                if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speedsPresent) {

                    // Start SEQUENCE OF AdvisorySpeedList
                    wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.count =
                        ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.count;

                    int count_i = ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.count;

                    for(int i = 0; i < count_i; i++) {
                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].speedPresent =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].speedPresent;
                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].confidencePresent =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].confidencePresent;
                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].distancePresent =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].distancePresent;
                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].class_Present =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].class_Present;

                        wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].type.value =
                            ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].type.value;
                        if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].speedPresent) {

                            wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].speed.value =
                                ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].speed.value;
                        }
                        if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].confidencePresent) {

                            wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].confidence.value =
                                ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].confidence.value;
                        }
                        if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].distancePresent) {

                            wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].distance.value =
                                ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].distance.value;
                        }
                        if(ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].class_Present) {

                            wind->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].class_.value =
                                ros->spat.intersections.elements[b].states.elements[f].state_time_speed.elements[h].speeds.elements[i].class_.value;
                        }
                    }
                    // End Sequence of AdvisorySpeedList
                }
            }
            // End Sequence of MovementEventList
            if(ros->spat.intersections.elements[b].states.elements[f].maneuverAssistListPresent) {

                // Start SEQUENCE OF ManeuverAssistList
                wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.count =
                    ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.count;

                int count_j = ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.count;

                for(int j = 0; j < count_j; j++) {
                    wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].queueLengthPresent =
                        ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].queueLengthPresent;
                    wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].availableStorageLengthPresent =
                        ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].availableStorageLengthPresent;
                    wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].waitOnStopPresent =
                        ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].waitOnStopPresent;
                    wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].pedBicycleDetectPresent =
                        ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].pedBicycleDetectPresent;

                    wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].connectionID.value =
                        ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].connectionID.value;
                    if(ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].queueLengthPresent) {

                        wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].queueLength.value =
                            ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].queueLength.value;
                    }
                    if(ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].availableStorageLengthPresent) {

                        wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].availableStorageLength.value =
                            ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].availableStorageLength.value;
                    }
                    if(ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].waitOnStopPresent) {

                        wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].waitOnStop.value =
                            ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].waitOnStop.value;
                    }
                    if(ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].pedBicycleDetectPresent) {

                        wind->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].pedBicycleDetect.value =
                            ros->spat.intersections.elements[b].states.elements[f].maneuverAssistList.elements[j].pedBicycleDetect.value;
                    }
                }
                // End Sequence of ManeuverAssistList
            }
        }
        // End Sequence of MovementList
        if(ros->spat.intersections.elements[b].maneuverAssistListPresent) {

            // Start SEQUENCE OF ManeuverAssistList
            wind->spat.intersections.elements[b].maneuverAssistList.count =
                ros->spat.intersections.elements[b].maneuverAssistList.count;

            int count_k = ros->spat.intersections.elements[b].maneuverAssistList.count;

            for(int k = 0; k < count_k; k++) {
                wind->spat.intersections.elements[b].maneuverAssistList.elements[k].queueLengthPresent =
                    ros->spat.intersections.elements[b].maneuverAssistList.elements[k].queueLengthPresent;
                wind->spat.intersections.elements[b].maneuverAssistList.elements[k].availableStorageLengthPresent =
                    ros->spat.intersections.elements[b].maneuverAssistList.elements[k].availableStorageLengthPresent;
                wind->spat.intersections.elements[b].maneuverAssistList.elements[k].waitOnStopPresent =
                    ros->spat.intersections.elements[b].maneuverAssistList.elements[k].waitOnStopPresent;
                wind->spat.intersections.elements[b].maneuverAssistList.elements[k].pedBicycleDetectPresent =
                    ros->spat.intersections.elements[b].maneuverAssistList.elements[k].pedBicycleDetectPresent;

                wind->spat.intersections.elements[b].maneuverAssistList.elements[k].connectionID.value =
                    ros->spat.intersections.elements[b].maneuverAssistList.elements[k].connectionID.value;
                if(ros->spat.intersections.elements[b].maneuverAssistList.elements[k].queueLengthPresent) {

                    wind->spat.intersections.elements[b].maneuverAssistList.elements[k].queueLength.value =
                        ros->spat.intersections.elements[b].maneuverAssistList.elements[k].queueLength.value;
                }
                if(ros->spat.intersections.elements[b].maneuverAssistList.elements[k].availableStorageLengthPresent) {

                    wind->spat.intersections.elements[b].maneuverAssistList.elements[k].availableStorageLength.value =
                        ros->spat.intersections.elements[b].maneuverAssistList.elements[k].availableStorageLength.value;
                }
                if(ros->spat.intersections.elements[b].maneuverAssistList.elements[k].waitOnStopPresent) {

                    wind->spat.intersections.elements[b].maneuverAssistList.elements[k].waitOnStop.value =
                        ros->spat.intersections.elements[b].maneuverAssistList.elements[k].waitOnStop.value;
                }
                if(ros->spat.intersections.elements[b].maneuverAssistList.elements[k].pedBicycleDetectPresent) {

                    wind->spat.intersections.elements[b].maneuverAssistList.elements[k].pedBicycleDetect.value =
                        ros->spat.intersections.elements[b].maneuverAssistList.elements[k].pedBicycleDetect.value;
                }
            }
            // End Sequence of ManeuverAssistList
        }
    }
    // End Sequence of IntersectionStateList
}
