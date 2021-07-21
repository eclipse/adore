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
 * Module: MAPEM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(103301) mapem(1) version2(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <dsrc_v2_mapem_pdu_descriptions_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const dsrc_v2_mapem_pdu_descriptions::MAPEM::ConstPtr& ros, wind::cpp::MAPEM_PDU_Descriptions::MAPEM* wind)
{

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;
    wind->map.timeStampPresent =
        ros->map.timeStampPresent;
    wind->map.layerTypePresent =
        ros->map.layerTypePresent;
    wind->map.layerIDPresent =
        ros->map.layerIDPresent;
    wind->map.intersectionsPresent =
        ros->map.intersectionsPresent;
    wind->map.restrictionListPresent =
        ros->map.restrictionListPresent;
    if(ros->map.timeStampPresent) {

        wind->map.timeStamp.value =
            ros->map.timeStamp.value;
    }

    wind->map.msgIssueRevision.value =
        ros->map.msgIssueRevision.value;
    if(ros->map.layerTypePresent) {

        wind->map.layerType.value =
            ros->map.layerType.value;
    }
    if(ros->map.layerIDPresent) {

        wind->map.layerID.value =
            ros->map.layerID.value;
    }
    if(ros->map.intersectionsPresent) {

        // Start SEQUENCE OF IntersectionGeometryList
        wind->map.intersections.count =
            ros->map.intersections.count;

        int count_a = ros->map.intersections.count;

        for(int a = 0; a < count_a; a++) {
            wind->map.intersections.elements[a].namePresent =
                ros->map.intersections.elements[a].namePresent;
            wind->map.intersections.elements[a].laneWidthPresent =
                ros->map.intersections.elements[a].laneWidthPresent;
            wind->map.intersections.elements[a].speedLimitsPresent =
                ros->map.intersections.elements[a].speedLimitsPresent;
            if(ros->map.intersections.elements[a].namePresent) {


                for(int b = 0; b < 63; b++) {  // DescriptiveName
                    if(b < ros->map.intersections.elements[a].name.value.length())
                        wind->map.intersections.elements[a].name.value[b] = ros->map.intersections.elements[a].name.value.c_str()[b];
                    else
                        wind->map.intersections.elements[a].name.value[b] = ' ';
                }
            }
            wind->map.intersections.elements[a].id.regionPresent =
                ros->map.intersections.elements[a].id.regionPresent;
            if(ros->map.intersections.elements[a].id.regionPresent) {

                wind->map.intersections.elements[a].id.region.value =
                    ros->map.intersections.elements[a].id.region.value;
            }

            wind->map.intersections.elements[a].id.id.value =
                ros->map.intersections.elements[a].id.id.value;

            wind->map.intersections.elements[a].revision.value =
                ros->map.intersections.elements[a].revision.value;
            wind->map.intersections.elements[a].refPoint.elevationPresent =
                ros->map.intersections.elements[a].refPoint.elevationPresent;

            wind->map.intersections.elements[a].refPoint.lat.value =
                ros->map.intersections.elements[a].refPoint.lat.value;

            wind->map.intersections.elements[a].refPoint.long_.value =
                ros->map.intersections.elements[a].refPoint.long_.value;
            if(ros->map.intersections.elements[a].refPoint.elevationPresent) {

                wind->map.intersections.elements[a].refPoint.elevation.value =
                    ros->map.intersections.elements[a].refPoint.elevation.value;
            }
            if(ros->map.intersections.elements[a].laneWidthPresent) {

                wind->map.intersections.elements[a].laneWidth.value =
                    ros->map.intersections.elements[a].laneWidth.value;
            }
            if(ros->map.intersections.elements[a].speedLimitsPresent) {

                // Start SEQUENCE OF SpeedLimitList
                wind->map.intersections.elements[a].speedLimits.count =
                    ros->map.intersections.elements[a].speedLimits.count;

                int count_c = ros->map.intersections.elements[a].speedLimits.count;

                for(int c = 0; c < count_c; c++) {

                    wind->map.intersections.elements[a].speedLimits.elements[c].type.value =
                        ros->map.intersections.elements[a].speedLimits.elements[c].type.value;

                    wind->map.intersections.elements[a].speedLimits.elements[c].speed.value =
                        ros->map.intersections.elements[a].speedLimits.elements[c].speed.value;
                }
                // End Sequence of SpeedLimitList
            }

            // Start SEQUENCE OF LaneList
            wind->map.intersections.elements[a].laneSet.count =
                ros->map.intersections.elements[a].laneSet.count;

            int count_d = ros->map.intersections.elements[a].laneSet.count;

            for(int d = 0; d < count_d; d++) {
                wind->map.intersections.elements[a].laneSet.elements[d].namePresent =
                    ros->map.intersections.elements[a].laneSet.elements[d].namePresent;
                wind->map.intersections.elements[a].laneSet.elements[d].ingressApproachPresent =
                    ros->map.intersections.elements[a].laneSet.elements[d].ingressApproachPresent;
                wind->map.intersections.elements[a].laneSet.elements[d].egressApproachPresent =
                    ros->map.intersections.elements[a].laneSet.elements[d].egressApproachPresent;
                wind->map.intersections.elements[a].laneSet.elements[d].maneuversPresent =
                    ros->map.intersections.elements[a].laneSet.elements[d].maneuversPresent;
                wind->map.intersections.elements[a].laneSet.elements[d].connectsToPresent =
                    ros->map.intersections.elements[a].laneSet.elements[d].connectsToPresent;
                wind->map.intersections.elements[a].laneSet.elements[d].overlaysPresent =
                    ros->map.intersections.elements[a].laneSet.elements[d].overlaysPresent;

                wind->map.intersections.elements[a].laneSet.elements[d].laneID.value =
                    ros->map.intersections.elements[a].laneSet.elements[d].laneID.value;
                if(ros->map.intersections.elements[a].laneSet.elements[d].namePresent) {


                    for(int e = 0; e < 63; e++) {  // DescriptiveName
                        if(e < ros->map.intersections.elements[a].laneSet.elements[d].name.value.length())
                            wind->map.intersections.elements[a].laneSet.elements[d].name.value[e] = ros->map.intersections.elements[a].laneSet.elements[d].name.value.c_str()[e];
                        else
                            wind->map.intersections.elements[a].laneSet.elements[d].name.value[e] = ' ';
                    }
                }
                if(ros->map.intersections.elements[a].laneSet.elements[d].ingressApproachPresent) {

                    wind->map.intersections.elements[a].laneSet.elements[d].ingressApproach.value =
                        ros->map.intersections.elements[a].laneSet.elements[d].ingressApproach.value;
                }
                if(ros->map.intersections.elements[a].laneSet.elements[d].egressApproachPresent) {

                    wind->map.intersections.elements[a].laneSet.elements[d].egressApproach.value =
                        ros->map.intersections.elements[a].laneSet.elements[d].egressApproach.value;
                }
                
                // START BIT STRING LaneDirection
                for(int f = 0; f < 2; f++) {
                    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.directionalUse.values[f] = 
                        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.directionalUse.values[f];
                }
                // END BIT STRING LaneDirection
                
                
                // START BIT STRING LaneSharing
                for(int g = 0; g < 10; g++) {
                    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.sharedWith.values[g] = 
                        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.sharedWith.values[g];
                }
                // END BIT STRING LaneSharing
                

                // START CHOICE LaneTypeAttributes
                wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.choice = ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.choice;

                if(ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.choice == 0) {

// START BIT STRING LaneAttributes_Vehicle
for(int h = 0; h < 8; h++) {
    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.vehicle.values[h] = 
        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.vehicle.values[h];
}
// END BIT STRING LaneAttributes_Vehicle

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.choice == 1)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Crosswalk
for(int i = 0; i < 16; i++) {
    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.crosswalk.values[i] = 
        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.crosswalk.values[i];
}
// END BIT STRING LaneAttributes_Crosswalk

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.choice == 2)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Bike
for(int j = 0; j < 16; j++) {
    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.bikeLane.values[j] = 
        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.bikeLane.values[j];
}
// END BIT STRING LaneAttributes_Bike

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.choice == 3)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Sidewalk
for(int k = 0; k < 16; k++) {
    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.sidewalk.values[k] = 
        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.sidewalk.values[k];
}
// END BIT STRING LaneAttributes_Sidewalk

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.choice == 4)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Barrier
for(int l = 0; l < 16; l++) {
    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.median.values[l] = 
        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.median.values[l];
}
// END BIT STRING LaneAttributes_Barrier

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.choice == 5)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Striping
for(int m = 0; m < 16; m++) {
    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.striping.values[m] = 
        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.striping.values[m];
}
// END BIT STRING LaneAttributes_Striping

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.choice == 6)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_TrackedVehicle
for(int n = 0; n < 16; n++) {
    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.trackedVehicle.values[n] = 
        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.trackedVehicle.values[n];
}
// END BIT STRING LaneAttributes_TrackedVehicle

                }
                else   // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Parking
for(int o = 0; o < 16; o++) {
    wind->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.parking.values[o] = 
        ros->map.intersections.elements[a].laneSet.elements[d].laneAttributes.laneType.parking.values[o];
}
// END BIT STRING LaneAttributes_Parking

                }
                // END CHOICE LaneTypeAttributes
                if(ros->map.intersections.elements[a].laneSet.elements[d].maneuversPresent) {
                    
                    // START BIT STRING AllowedManeuvers
                    for(int p = 0; p < 12; p++) {
                        wind->map.intersections.elements[a].laneSet.elements[d].maneuvers.values[p] = 
                            ros->map.intersections.elements[a].laneSet.elements[d].maneuvers.values[p];
                    }
                    // END BIT STRING AllowedManeuvers
                    
                }

                // START CHOICE NodeListXY
                wind->map.intersections.elements[a].laneSet.elements[d].nodeList.choice = ros->map.intersections.elements[a].laneSet.elements[d].nodeList.choice;

                if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.choice == 0) {

                    // Start SEQUENCE OF NodeSetXY
                    wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.count =
                        ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.count;

                    int count_q = ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.count;

                    for(int q = 0; q < count_q; q++) {

                        // START CHOICE NodeOffsetPointXY
                        wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.choice = ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.choice;

                        if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.choice == 0) {

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY1.x.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY1.x.value;

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY1.y.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY1.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.choice == 1)  // CHOICE NodeOffsetPointXY
                        {

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY2.x.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY2.x.value;

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY2.y.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY2.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.choice == 2)  // CHOICE NodeOffsetPointXY
                        {

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY3.x.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY3.x.value;

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY3.y.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY3.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.choice == 3)  // CHOICE NodeOffsetPointXY
                        {

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY4.x.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY4.x.value;

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY4.y.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY4.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.choice == 4)  // CHOICE NodeOffsetPointXY
                        {

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY5.x.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY5.x.value;

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY5.y.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY5.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.choice == 5)  // CHOICE NodeOffsetPointXY
                        {

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY6.x.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY6.x.value;

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY6.y.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_XY6.y.value;
                        }
                        else   // CHOICE NodeOffsetPointXY
                        {

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_LatLon.lon.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_LatLon.lon.value;

                            wind->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_LatLon.lat.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].nodeList.nodes.elements[q].delta.node_LatLon.lat.value;
                        }
                        // END CHOICE NodeOffsetPointXY
                    }
                    // End Sequence of NodeSetXY
                }
                else   // CHOICE NodeListXY
                {
                    wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.rotateXYPresent =
                        ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.rotateXYPresent;
                    wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleXaxisPresent =
                        ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleXaxisPresent;
                    wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleYaxisPresent =
                        ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleYaxisPresent;

                    wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.referenceLaneId.value =
                        ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.referenceLaneId.value;

                    // START CHOICE ComputedLane_offsetXaxis
                    wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetXaxis.choice = ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetXaxis.choice;

                    if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetXaxis.choice == 0) {

                        wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetXaxis.small.value =
                            ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetXaxis.small.value;
                    }
                    else   // CHOICE ComputedLane_offsetXaxis
                    {

                        wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetXaxis.large.value =
                            ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetXaxis.large.value;
                    }
                    // END CHOICE ComputedLane_offsetXaxis

                    // START CHOICE ComputedLane_offsetYaxis
                    wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetYaxis.choice = ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetYaxis.choice;

                    if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetYaxis.choice == 0) {

                        wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetYaxis.small.value =
                            ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetYaxis.small.value;
                    }
                    else   // CHOICE ComputedLane_offsetYaxis
                    {

                        wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetYaxis.large.value =
                            ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.offsetYaxis.large.value;
                    }
                    // END CHOICE ComputedLane_offsetYaxis
                    if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.rotateXYPresent) {

                        wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.rotateXY.value =
                            ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.rotateXY.value;
                    }
                    if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleXaxisPresent) {

                        wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleXaxis.value =
                            ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleXaxis.value;
                    }
                    if(ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleYaxisPresent) {

                        wind->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleYaxis.value =
                            ros->map.intersections.elements[a].laneSet.elements[d].nodeList.computed.scaleYaxis.value;
                    }
                }
                // END CHOICE NodeListXY
                if(ros->map.intersections.elements[a].laneSet.elements[d].connectsToPresent) {

                    // Start SEQUENCE OF ConnectsToList
                    wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.count =
                        ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.count;

                    int count_r = ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.count;

                    for(int r = 0; r < count_r; r++) {
                        wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersectionPresent =
                            ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersectionPresent;
                        wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].signalGroupPresent =
                            ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].signalGroupPresent;
                        wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].userClassPresent =
                            ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].userClassPresent;
                        wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectionIDPresent =
                            ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectionIDPresent;
                        wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectingLane.maneuverPresent =
                            ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectingLane.maneuverPresent;

                        wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectingLane.lane.value =
                            ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectingLane.lane.value;
                        if(ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectingLane.maneuverPresent) {
                            
                            // START BIT STRING AllowedManeuvers
                            for(int s = 0; s < 12; s++) {
                                wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectingLane.maneuver.values[s] = 
                                    ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectingLane.maneuver.values[s];
                            }
                            // END BIT STRING AllowedManeuvers
                            
                        }
                        if(ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersectionPresent) {
                            wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersection.regionPresent =
                                ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersection.regionPresent;
                            if(ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersection.regionPresent) {

                                wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersection.region.value =
                                    ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersection.region.value;
                            }

                            wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersection.id.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].remoteIntersection.id.value;
                        }
                        if(ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].signalGroupPresent) {

                            wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].signalGroup.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].signalGroup.value;
                        }
                        if(ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].userClassPresent) {

                            wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].userClass.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].userClass.value;
                        }
                        if(ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectionIDPresent) {

                            wind->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectionID.value =
                                ros->map.intersections.elements[a].laneSet.elements[d].connectsTo.elements[r].connectionID.value;
                        }
                    }
                    // End Sequence of ConnectsToList
                }
                if(ros->map.intersections.elements[a].laneSet.elements[d].overlaysPresent) {

                    // Start SEQUENCE OF OverlayLaneList
                    wind->map.intersections.elements[a].laneSet.elements[d].overlays.count =
                        ros->map.intersections.elements[a].laneSet.elements[d].overlays.count;

                    int count_t = ros->map.intersections.elements[a].laneSet.elements[d].overlays.count;

                    for(int t = 0; t < count_t; t++) {

                        wind->map.intersections.elements[a].laneSet.elements[d].overlays.elements[t].value =
                            ros->map.intersections.elements[a].laneSet.elements[d].overlays.elements[t].value;
                    }
                    // End Sequence of OverlayLaneList
                }
            }
            // End Sequence of LaneList
        }
        // End Sequence of IntersectionGeometryList
    }
    if(ros->map.restrictionListPresent) {

        // Start SEQUENCE OF RestrictionClassList
        wind->map.restrictionList.count =
            ros->map.restrictionList.count;

        int count_u = ros->map.restrictionList.count;

        for(int u = 0; u < count_u; u++) {

            wind->map.restrictionList.elements[u].id.value =
                ros->map.restrictionList.elements[u].id.value;

            // Start SEQUENCE OF RestrictionUserTypeList
            wind->map.restrictionList.elements[u].users.count =
                ros->map.restrictionList.elements[u].users.count;

            int count_v = ros->map.restrictionList.elements[u].users.count;

            for(int v = 0; v < count_v; v++) {

                // START CHOICE RestrictionUserType
                wind->map.restrictionList.elements[u].users.elements[v].choice = ros->map.restrictionList.elements[u].users.elements[v].choice;

                if(ros->map.restrictionList.elements[u].users.elements[v].choice == 0) {

                    wind->map.restrictionList.elements[u].users.elements[v].basicType.value =
                        ros->map.restrictionList.elements[u].users.elements[v].basicType.value;
                }
                // END CHOICE RestrictionUserType
            }
            // End Sequence of RestrictionUserTypeList
        }
        // End Sequence of RestrictionClassList
    }
}
