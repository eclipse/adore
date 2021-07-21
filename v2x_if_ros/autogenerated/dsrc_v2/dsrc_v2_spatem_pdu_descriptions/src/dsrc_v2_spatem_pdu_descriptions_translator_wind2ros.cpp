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

#include <dsrc_v2_spatem_pdu_descriptions_translator_wind2ros.h>

void wind::wind_ros::wind2ros(dsrc_v2_spatem_pdu_descriptions::SPATEM* ros, wind::cpp::SPATEM_PDU_Descriptions::SPATEM* wind)
{

    ros->header.protocolVersion.value =
        wind->header.protocolVersion.value;

    ros->header.messageID.value =
        wind->header.messageID.value;

    ros->header.stationID.value =
        wind->header.stationID.value;
    ros->spat.timeStampPresent =
        wind->spat.timeStampPresent;
    ros->spat.namePresent =
        wind->spat.namePresent;
    if(ros->spat.timeStampPresent) {

        ros->spat.timeStamp.value =
            wind->spat.timeStamp.value;
    }
    if(ros->spat.namePresent) {

        ros->spat.name.value.assign(wind->spat.name.value);
    }

    // Start SEQUENCE OF IntersectionStateList
    ros->spat.intersections.count =
        wind->spat.intersections.count;

    int count_a = ros->spat.intersections.count;

    for(int a = 0; a < count_a; a++) {
        dsrc_v2_dsrc::IntersectionState tmp_a;
        ros->spat.intersections.elements.push_back(tmp_a);
        ros->spat.intersections.elements[a].namePresent =
            wind->spat.intersections.elements[a].namePresent;
        ros->spat.intersections.elements[a].moyPresent =
            wind->spat.intersections.elements[a].moyPresent;
        ros->spat.intersections.elements[a].timeStampPresent =
            wind->spat.intersections.elements[a].timeStampPresent;
        ros->spat.intersections.elements[a].enabledLanesPresent =
            wind->spat.intersections.elements[a].enabledLanesPresent;
        ros->spat.intersections.elements[a].maneuverAssistListPresent =
            wind->spat.intersections.elements[a].maneuverAssistListPresent;
        if(ros->spat.intersections.elements[a].namePresent) {

            ros->spat.intersections.elements[a].name.value.assign(wind->spat.intersections.elements[a].name.value);
        }
        ros->spat.intersections.elements[a].id.regionPresent =
            wind->spat.intersections.elements[a].id.regionPresent;
        if(ros->spat.intersections.elements[a].id.regionPresent) {

            ros->spat.intersections.elements[a].id.region.value =
                wind->spat.intersections.elements[a].id.region.value;
        }

        ros->spat.intersections.elements[a].id.id.value =
            wind->spat.intersections.elements[a].id.id.value;

        ros->spat.intersections.elements[a].revision.value =
            wind->spat.intersections.elements[a].revision.value;
        
        // START BIT STRING IntersectionStatusObject
        for(int b = 0; b < 16; b++) {
            uint8_t tmp_b;
            ros->spat.intersections.elements[a].status.values.push_back(tmp_b);
            ros->spat.intersections.elements[a].status.values[b] = 
                wind->spat.intersections.elements[a].status.values[b];
        }
        // END BIT STRING IntersectionStatusObject
        
        if(ros->spat.intersections.elements[a].moyPresent) {

            ros->spat.intersections.elements[a].moy.value =
                wind->spat.intersections.elements[a].moy.value;
        }
        if(ros->spat.intersections.elements[a].timeStampPresent) {

            ros->spat.intersections.elements[a].timeStamp.value =
                wind->spat.intersections.elements[a].timeStamp.value;
        }
        if(ros->spat.intersections.elements[a].enabledLanesPresent) {

            // Start SEQUENCE OF EnabledLaneList
            ros->spat.intersections.elements[a].enabledLanes.count =
                wind->spat.intersections.elements[a].enabledLanes.count;

            int count_c = ros->spat.intersections.elements[a].enabledLanes.count;

            for(int c = 0; c < count_c; c++) {
                dsrc_v2_dsrc::LaneID tmp_c;
                ros->spat.intersections.elements[a].enabledLanes.elements.push_back(tmp_c);

                ros->spat.intersections.elements[a].enabledLanes.elements[c].value =
                    wind->spat.intersections.elements[a].enabledLanes.elements[c].value;
            }
            // End Sequence of EnabledLaneList
        }

        // Start SEQUENCE OF MovementList
        ros->spat.intersections.elements[a].states.count =
            wind->spat.intersections.elements[a].states.count;

        int count_d = ros->spat.intersections.elements[a].states.count;

        for(int d = 0; d < count_d; d++) {
            dsrc_v2_dsrc::MovementState tmp_d;
            ros->spat.intersections.elements[a].states.elements.push_back(tmp_d);
            ros->spat.intersections.elements[a].states.elements[d].movementNamePresent =
                wind->spat.intersections.elements[a].states.elements[d].movementNamePresent;
            ros->spat.intersections.elements[a].states.elements[d].maneuverAssistListPresent =
                wind->spat.intersections.elements[a].states.elements[d].maneuverAssistListPresent;
            if(ros->spat.intersections.elements[a].states.elements[d].movementNamePresent) {

                ros->spat.intersections.elements[a].states.elements[d].movementName.value.assign(wind->spat.intersections.elements[a].states.elements[d].movementName.value);
            }

            ros->spat.intersections.elements[a].states.elements[d].signalGroup.value =
                wind->spat.intersections.elements[a].states.elements[d].signalGroup.value;

            // Start SEQUENCE OF MovementEventList
            ros->spat.intersections.elements[a].states.elements[d].state_time_speed.count =
                wind->spat.intersections.elements[a].states.elements[d].state_time_speed.count;

            int count_e = ros->spat.intersections.elements[a].states.elements[d].state_time_speed.count;

            for(int e = 0; e < count_e; e++) {
                dsrc_v2_dsrc::MovementEvent tmp_e;
                ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements.push_back(tmp_e);
                ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timingPresent =
                    wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timingPresent;
                ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speedsPresent =
                    wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speedsPresent;

                ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].eventState.value =
                    wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].eventState.value;
                if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timingPresent) {
                    ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.startTimePresent =
                        wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.startTimePresent;
                    ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.maxEndTimePresent =
                        wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.maxEndTimePresent;
                    ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.likelyTimePresent =
                        wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.likelyTimePresent;
                    ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.confidencePresent =
                        wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.confidencePresent;
                    ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.nextTimePresent =
                        wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.nextTimePresent;
                    if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.startTimePresent) {

                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.startTime.value =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.startTime.value;
                    }

                    ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.minEndTime.value =
                        wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.minEndTime.value;
                    if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.maxEndTimePresent) {

                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.maxEndTime.value =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.maxEndTime.value;
                    }
                    if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.likelyTimePresent) {

                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.likelyTime.value =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.likelyTime.value;
                    }
                    if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.confidencePresent) {

                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.confidence.value =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.confidence.value;
                    }
                    if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.nextTimePresent) {

                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.nextTime.value =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].timing.nextTime.value;
                    }
                }
                if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speedsPresent) {

                    // Start SEQUENCE OF AdvisorySpeedList
                    ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.count =
                        wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.count;

                    int count_f = ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.count;

                    for(int f = 0; f < count_f; f++) {
                        dsrc_v2_dsrc::AdvisorySpeed tmp_f;
                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements.push_back(tmp_f);
                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].speedPresent =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].speedPresent;
                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].confidencePresent =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].confidencePresent;
                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].distancePresent =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].distancePresent;
                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].class_Present =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].class_Present;

                        ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].type.value =
                            wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].type.value;
                        if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].speedPresent) {

                            ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].speed.value =
                                wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].speed.value;
                        }
                        if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].confidencePresent) {

                            ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].confidence.value =
                                wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].confidence.value;
                        }
                        if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].distancePresent) {

                            ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].distance.value =
                                wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].distance.value;
                        }
                        if(ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].class_Present) {

                            ros->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].class_.value =
                                wind->spat.intersections.elements[a].states.elements[d].state_time_speed.elements[e].speeds.elements[f].class_.value;
                        }
                    }
                    // End Sequence of AdvisorySpeedList
                }
            }
            // End Sequence of MovementEventList
            if(ros->spat.intersections.elements[a].states.elements[d].maneuverAssistListPresent) {

                // Start SEQUENCE OF ManeuverAssistList
                ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.count =
                    wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.count;

                int count_g = ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.count;

                for(int g = 0; g < count_g; g++) {
                    dsrc_v2_dsrc::ConnectionManeuverAssist tmp_g;
                    ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements.push_back(tmp_g);
                    ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].queueLengthPresent =
                        wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].queueLengthPresent;
                    ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].availableStorageLengthPresent =
                        wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].availableStorageLengthPresent;
                    ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].waitOnStopPresent =
                        wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].waitOnStopPresent;
                    ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].pedBicycleDetectPresent =
                        wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].pedBicycleDetectPresent;

                    ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].connectionID.value =
                        wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].connectionID.value;
                    if(ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].queueLengthPresent) {

                        ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].queueLength.value =
                            wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].queueLength.value;
                    }
                    if(ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].availableStorageLengthPresent) {

                        ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].availableStorageLength.value =
                            wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].availableStorageLength.value;
                    }
                    if(ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].waitOnStopPresent) {

                        ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].waitOnStop.value =
                            wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].waitOnStop.value;
                    }
                    if(ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].pedBicycleDetectPresent) {

                        ros->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].pedBicycleDetect.value =
                            wind->spat.intersections.elements[a].states.elements[d].maneuverAssistList.elements[g].pedBicycleDetect.value;
                    }
                }
                // End Sequence of ManeuverAssistList
            }
        }
        // End Sequence of MovementList
        if(ros->spat.intersections.elements[a].maneuverAssistListPresent) {

            // Start SEQUENCE OF ManeuverAssistList
            ros->spat.intersections.elements[a].maneuverAssistList.count =
                wind->spat.intersections.elements[a].maneuverAssistList.count;

            int count_h = ros->spat.intersections.elements[a].maneuverAssistList.count;

            for(int h = 0; h < count_h; h++) {
                dsrc_v2_dsrc::ConnectionManeuverAssist tmp_h;
                ros->spat.intersections.elements[a].maneuverAssistList.elements.push_back(tmp_h);
                ros->spat.intersections.elements[a].maneuverAssistList.elements[h].queueLengthPresent =
                    wind->spat.intersections.elements[a].maneuverAssistList.elements[h].queueLengthPresent;
                ros->spat.intersections.elements[a].maneuverAssistList.elements[h].availableStorageLengthPresent =
                    wind->spat.intersections.elements[a].maneuverAssistList.elements[h].availableStorageLengthPresent;
                ros->spat.intersections.elements[a].maneuverAssistList.elements[h].waitOnStopPresent =
                    wind->spat.intersections.elements[a].maneuverAssistList.elements[h].waitOnStopPresent;
                ros->spat.intersections.elements[a].maneuverAssistList.elements[h].pedBicycleDetectPresent =
                    wind->spat.intersections.elements[a].maneuverAssistList.elements[h].pedBicycleDetectPresent;

                ros->spat.intersections.elements[a].maneuverAssistList.elements[h].connectionID.value =
                    wind->spat.intersections.elements[a].maneuverAssistList.elements[h].connectionID.value;
                if(ros->spat.intersections.elements[a].maneuverAssistList.elements[h].queueLengthPresent) {

                    ros->spat.intersections.elements[a].maneuverAssistList.elements[h].queueLength.value =
                        wind->spat.intersections.elements[a].maneuverAssistList.elements[h].queueLength.value;
                }
                if(ros->spat.intersections.elements[a].maneuverAssistList.elements[h].availableStorageLengthPresent) {

                    ros->spat.intersections.elements[a].maneuverAssistList.elements[h].availableStorageLength.value =
                        wind->spat.intersections.elements[a].maneuverAssistList.elements[h].availableStorageLength.value;
                }
                if(ros->spat.intersections.elements[a].maneuverAssistList.elements[h].waitOnStopPresent) {

                    ros->spat.intersections.elements[a].maneuverAssistList.elements[h].waitOnStop.value =
                        wind->spat.intersections.elements[a].maneuverAssistList.elements[h].waitOnStop.value;
                }
                if(ros->spat.intersections.elements[a].maneuverAssistList.elements[h].pedBicycleDetectPresent) {

                    ros->spat.intersections.elements[a].maneuverAssistList.elements[h].pedBicycleDetect.value =
                        wind->spat.intersections.elements[a].maneuverAssistList.elements[h].pedBicycleDetect.value;
                }
            }
            // End Sequence of ManeuverAssistList
        }
    }
    // End Sequence of IntersectionStateList
}
