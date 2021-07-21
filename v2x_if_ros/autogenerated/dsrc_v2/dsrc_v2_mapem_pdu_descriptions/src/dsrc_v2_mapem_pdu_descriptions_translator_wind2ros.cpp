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

#include <dsrc_v2_mapem_pdu_descriptions_translator_wind2ros.h>

void wind::wind_ros::wind2ros(dsrc_v2_mapem_pdu_descriptions::MAPEM* ros, wind::cpp::MAPEM_PDU_Descriptions::MAPEM* wind)
{

    ros->header.protocolVersion.value =
        wind->header.protocolVersion.value;

    ros->header.messageID.value =
        wind->header.messageID.value;

    ros->header.stationID.value =
        wind->header.stationID.value;
    ros->map.timeStampPresent =
        wind->map.timeStampPresent;
    ros->map.layerTypePresent =
        wind->map.layerTypePresent;
    ros->map.layerIDPresent =
        wind->map.layerIDPresent;
    ros->map.intersectionsPresent =
        wind->map.intersectionsPresent;
    ros->map.restrictionListPresent =
        wind->map.restrictionListPresent;
    if(ros->map.timeStampPresent) {

        ros->map.timeStamp.value =
            wind->map.timeStamp.value;
    }

    ros->map.msgIssueRevision.value =
        wind->map.msgIssueRevision.value;
    if(ros->map.layerTypePresent) {

        ros->map.layerType.value =
            wind->map.layerType.value;
    }
    if(ros->map.layerIDPresent) {

        ros->map.layerID.value =
            wind->map.layerID.value;
    }
    if(ros->map.intersectionsPresent) {

        // Start SEQUENCE OF IntersectionGeometryList
        ros->map.intersections.count =
            wind->map.intersections.count;

        int count_a = ros->map.intersections.count;

        for(int a = 0; a < count_a; a++) {
            dsrc_v2_dsrc::IntersectionGeometry tmp_a;
            ros->map.intersections.elements.push_back(tmp_a);
            ros->map.intersections.elements[a].namePresent =
                wind->map.intersections.elements[a].namePresent;
            ros->map.intersections.elements[a].laneWidthPresent =
                wind->map.intersections.elements[a].laneWidthPresent;
            ros->map.intersections.elements[a].speedLimitsPresent =
                wind->map.intersections.elements[a].speedLimitsPresent;
            if(ros->map.intersections.elements[a].namePresent) {

                ros->map.intersections.elements[a].name.value.assign(wind->map.intersections.elements[a].name.value);
            }
            ros->map.intersections.elements[a].id.regionPresent =
                wind->map.intersections.elements[a].id.regionPresent;
            if(ros->map.intersections.elements[a].id.regionPresent) {

                ros->map.intersections.elements[a].id.region.value =
                    wind->map.intersections.elements[a].id.region.value;
            }

            ros->map.intersections.elements[a].id.id.value =
                wind->map.intersections.elements[a].id.id.value;

            ros->map.intersections.elements[a].revision.value =
                wind->map.intersections.elements[a].revision.value;
            ros->map.intersections.elements[a].refPoint.elevationPresent =
                wind->map.intersections.elements[a].refPoint.elevationPresent;

            ros->map.intersections.elements[a].refPoint.lat.value =
                wind->map.intersections.elements[a].refPoint.lat.value;

            ros->map.intersections.elements[a].refPoint.long_.value =
                wind->map.intersections.elements[a].refPoint.long_.value;
            if(ros->map.intersections.elements[a].refPoint.elevationPresent) {

                ros->map.intersections.elements[a].refPoint.elevation.value =
                    wind->map.intersections.elements[a].refPoint.elevation.value;
            }
            if(ros->map.intersections.elements[a].laneWidthPresent) {

                ros->map.intersections.elements[a].laneWidth.value =
                    wind->map.intersections.elements[a].laneWidth.value;
            }
            if(ros->map.intersections.elements[a].speedLimitsPresent) {

                // Start SEQUENCE OF SpeedLimitList
                ros->map.intersections.elements[a].speedLimits.count =
                    wind->map.intersections.elements[a].speedLimits.count;

                int count_b = ros->map.intersections.elements[a].speedLimits.count;

                for(int b = 0; b < count_b; b++) {
                    dsrc_v2_dsrc::RegulatorySpeedLimit tmp_b;
                    ros->map.intersections.elements[a].speedLimits.elements.push_back(tmp_b);

                    ros->map.intersections.elements[a].speedLimits.elements[b].type.value =
                        wind->map.intersections.elements[a].speedLimits.elements[b].type.value;

                    ros->map.intersections.elements[a].speedLimits.elements[b].speed.value =
                        wind->map.intersections.elements[a].speedLimits.elements[b].speed.value;
                }
                // End Sequence of SpeedLimitList
            }

            // Start SEQUENCE OF LaneList
            ros->map.intersections.elements[a].laneSet.count =
                wind->map.intersections.elements[a].laneSet.count;

            int count_c = ros->map.intersections.elements[a].laneSet.count;

            for(int c = 0; c < count_c; c++) {
                dsrc_v2_dsrc::GenericLane tmp_c;
                ros->map.intersections.elements[a].laneSet.elements.push_back(tmp_c);
                ros->map.intersections.elements[a].laneSet.elements[c].namePresent =
                    wind->map.intersections.elements[a].laneSet.elements[c].namePresent;
                ros->map.intersections.elements[a].laneSet.elements[c].ingressApproachPresent =
                    wind->map.intersections.elements[a].laneSet.elements[c].ingressApproachPresent;
                ros->map.intersections.elements[a].laneSet.elements[c].egressApproachPresent =
                    wind->map.intersections.elements[a].laneSet.elements[c].egressApproachPresent;
                ros->map.intersections.elements[a].laneSet.elements[c].maneuversPresent =
                    wind->map.intersections.elements[a].laneSet.elements[c].maneuversPresent;
                ros->map.intersections.elements[a].laneSet.elements[c].connectsToPresent =
                    wind->map.intersections.elements[a].laneSet.elements[c].connectsToPresent;
                ros->map.intersections.elements[a].laneSet.elements[c].overlaysPresent =
                    wind->map.intersections.elements[a].laneSet.elements[c].overlaysPresent;

                ros->map.intersections.elements[a].laneSet.elements[c].laneID.value =
                    wind->map.intersections.elements[a].laneSet.elements[c].laneID.value;
                if(ros->map.intersections.elements[a].laneSet.elements[c].namePresent) {

                    ros->map.intersections.elements[a].laneSet.elements[c].name.value.assign(wind->map.intersections.elements[a].laneSet.elements[c].name.value);
                }
                if(ros->map.intersections.elements[a].laneSet.elements[c].ingressApproachPresent) {

                    ros->map.intersections.elements[a].laneSet.elements[c].ingressApproach.value =
                        wind->map.intersections.elements[a].laneSet.elements[c].ingressApproach.value;
                }
                if(ros->map.intersections.elements[a].laneSet.elements[c].egressApproachPresent) {

                    ros->map.intersections.elements[a].laneSet.elements[c].egressApproach.value =
                        wind->map.intersections.elements[a].laneSet.elements[c].egressApproach.value;
                }
                
                // START BIT STRING LaneDirection
                for(int d = 0; d < 2; d++) {
                    uint8_t tmp_d;
                    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.directionalUse.values.push_back(tmp_d);
                    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.directionalUse.values[d] = 
                        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.directionalUse.values[d];
                }
                // END BIT STRING LaneDirection
                
                
                // START BIT STRING LaneSharing
                for(int e = 0; e < 10; e++) {
                    uint8_t tmp_e;
                    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.sharedWith.values.push_back(tmp_e);
                    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.sharedWith.values[e] = 
                        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.sharedWith.values[e];
                }
                // END BIT STRING LaneSharing
                

                // START CHOICE LaneTypeAttributes
                ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.choice = wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.choice;

                if(ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.choice == 0) {

// START BIT STRING LaneAttributes_Vehicle
for(int f = 0; f < 8; f++) {
    uint8_t tmp_f;
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.vehicle.values.push_back(tmp_f);
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.vehicle.values[f] = 
        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.vehicle.values[f];
}
// END BIT STRING LaneAttributes_Vehicle

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.choice == 1)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Crosswalk
for(int g = 0; g < 16; g++) {
    uint8_t tmp_g;
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.crosswalk.values.push_back(tmp_g);
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.crosswalk.values[g] = 
        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.crosswalk.values[g];
}
// END BIT STRING LaneAttributes_Crosswalk

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.choice == 2)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Bike
for(int h = 0; h < 16; h++) {
    uint8_t tmp_h;
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.bikeLane.values.push_back(tmp_h);
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.bikeLane.values[h] = 
        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.bikeLane.values[h];
}
// END BIT STRING LaneAttributes_Bike

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.choice == 3)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Sidewalk
for(int i = 0; i < 16; i++) {
    uint8_t tmp_i;
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.sidewalk.values.push_back(tmp_i);
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.sidewalk.values[i] = 
        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.sidewalk.values[i];
}
// END BIT STRING LaneAttributes_Sidewalk

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.choice == 4)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Barrier
for(int j = 0; j < 16; j++) {
    uint8_t tmp_j;
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.median.values.push_back(tmp_j);
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.median.values[j] = 
        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.median.values[j];
}
// END BIT STRING LaneAttributes_Barrier

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.choice == 5)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Striping
for(int k = 0; k < 16; k++) {
    uint8_t tmp_k;
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.striping.values.push_back(tmp_k);
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.striping.values[k] = 
        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.striping.values[k];
}
// END BIT STRING LaneAttributes_Striping

                }
                else if(ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.choice == 6)  // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_TrackedVehicle
for(int l = 0; l < 16; l++) {
    uint8_t tmp_l;
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.trackedVehicle.values.push_back(tmp_l);
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.trackedVehicle.values[l] = 
        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.trackedVehicle.values[l];
}
// END BIT STRING LaneAttributes_TrackedVehicle

                }
                else   // CHOICE LaneTypeAttributes
                {

// START BIT STRING LaneAttributes_Parking
for(int m = 0; m < 16; m++) {
    uint8_t tmp_m;
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.parking.values.push_back(tmp_m);
    ros->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.parking.values[m] = 
        wind->map.intersections.elements[a].laneSet.elements[c].laneAttributes.laneType.parking.values[m];
}
// END BIT STRING LaneAttributes_Parking

                }
                // END CHOICE LaneTypeAttributes
                if(ros->map.intersections.elements[a].laneSet.elements[c].maneuversPresent) {
                    
                    // START BIT STRING AllowedManeuvers
                    for(int n = 0; n < 12; n++) {
                        uint8_t tmp_n;
                        ros->map.intersections.elements[a].laneSet.elements[c].maneuvers.values.push_back(tmp_n);
                        ros->map.intersections.elements[a].laneSet.elements[c].maneuvers.values[n] = 
                            wind->map.intersections.elements[a].laneSet.elements[c].maneuvers.values[n];
                    }
                    // END BIT STRING AllowedManeuvers
                    
                }

                // START CHOICE NodeListXY
                ros->map.intersections.elements[a].laneSet.elements[c].nodeList.choice = wind->map.intersections.elements[a].laneSet.elements[c].nodeList.choice;

                if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.choice == 0) {

                    // Start SEQUENCE OF NodeSetXY
                    ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.count =
                        wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.count;

                    int count_o = ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.count;

                    for(int o = 0; o < count_o; o++) {
                        dsrc_v2_dsrc::NodeXY tmp_o;
                        ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements.push_back(tmp_o);

                        // START CHOICE NodeOffsetPointXY
                        ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.choice = wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.choice;

                        if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.choice == 0) {

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY1.x.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY1.x.value;

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY1.y.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY1.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.choice == 1)  // CHOICE NodeOffsetPointXY
                        {

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY2.x.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY2.x.value;

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY2.y.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY2.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.choice == 2)  // CHOICE NodeOffsetPointXY
                        {

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY3.x.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY3.x.value;

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY3.y.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY3.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.choice == 3)  // CHOICE NodeOffsetPointXY
                        {

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY4.x.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY4.x.value;

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY4.y.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY4.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.choice == 4)  // CHOICE NodeOffsetPointXY
                        {

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY5.x.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY5.x.value;

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY5.y.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY5.y.value;
                        }
                        else if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.choice == 5)  // CHOICE NodeOffsetPointXY
                        {

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY6.x.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY6.x.value;

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY6.y.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_XY6.y.value;
                        }
                        else   // CHOICE NodeOffsetPointXY
                        {

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_LatLon.lon.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_LatLon.lon.value;

                            ros->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_LatLon.lat.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].nodeList.nodes.elements[o].delta.node_LatLon.lat.value;
                        }
                        // END CHOICE NodeOffsetPointXY
                    }
                    // End Sequence of NodeSetXY
                }
                else   // CHOICE NodeListXY
                {
                    ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.rotateXYPresent =
                        wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.rotateXYPresent;
                    ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleXaxisPresent =
                        wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleXaxisPresent;
                    ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleYaxisPresent =
                        wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleYaxisPresent;

                    ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.referenceLaneId.value =
                        wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.referenceLaneId.value;

                    // START CHOICE ComputedLane_offsetXaxis
                    ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetXaxis.choice = wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetXaxis.choice;

                    if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetXaxis.choice == 0) {

                        ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetXaxis.small.value =
                            wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetXaxis.small.value;
                    }
                    else   // CHOICE ComputedLane_offsetXaxis
                    {

                        ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetXaxis.large.value =
                            wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetXaxis.large.value;
                    }
                    // END CHOICE ComputedLane_offsetXaxis

                    // START CHOICE ComputedLane_offsetYaxis
                    ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetYaxis.choice = wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetYaxis.choice;

                    if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetYaxis.choice == 0) {

                        ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetYaxis.small.value =
                            wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetYaxis.small.value;
                    }
                    else   // CHOICE ComputedLane_offsetYaxis
                    {

                        ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetYaxis.large.value =
                            wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.offsetYaxis.large.value;
                    }
                    // END CHOICE ComputedLane_offsetYaxis
                    if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.rotateXYPresent) {

                        ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.rotateXY.value =
                            wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.rotateXY.value;
                    }
                    if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleXaxisPresent) {

                        ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleXaxis.value =
                            wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleXaxis.value;
                    }
                    if(ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleYaxisPresent) {

                        ros->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleYaxis.value =
                            wind->map.intersections.elements[a].laneSet.elements[c].nodeList.computed.scaleYaxis.value;
                    }
                }
                // END CHOICE NodeListXY
                if(ros->map.intersections.elements[a].laneSet.elements[c].connectsToPresent) {

                    // Start SEQUENCE OF ConnectsToList
                    ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.count =
                        wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.count;

                    int count_p = ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.count;

                    for(int p = 0; p < count_p; p++) {
                        dsrc_v2_dsrc::Connection tmp_p;
                        ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements.push_back(tmp_p);
                        ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersectionPresent =
                            wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersectionPresent;
                        ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].signalGroupPresent =
                            wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].signalGroupPresent;
                        ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].userClassPresent =
                            wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].userClassPresent;
                        ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectionIDPresent =
                            wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectionIDPresent;
                        ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectingLane.maneuverPresent =
                            wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectingLane.maneuverPresent;

                        ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectingLane.lane.value =
                            wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectingLane.lane.value;
                        if(ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectingLane.maneuverPresent) {
                            
                            // START BIT STRING AllowedManeuvers
                            for(int q = 0; q < 12; q++) {
                                uint8_t tmp_q;
                                ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectingLane.maneuver.values.push_back(tmp_q);
                                ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectingLane.maneuver.values[q] = 
                                    wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectingLane.maneuver.values[q];
                            }
                            // END BIT STRING AllowedManeuvers
                            
                        }
                        if(ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersectionPresent) {
                            ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersection.regionPresent =
                                wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersection.regionPresent;
                            if(ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersection.regionPresent) {

                                ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersection.region.value =
                                    wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersection.region.value;
                            }

                            ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersection.id.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].remoteIntersection.id.value;
                        }
                        if(ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].signalGroupPresent) {

                            ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].signalGroup.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].signalGroup.value;
                        }
                        if(ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].userClassPresent) {

                            ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].userClass.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].userClass.value;
                        }
                        if(ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectionIDPresent) {

                            ros->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectionID.value =
                                wind->map.intersections.elements[a].laneSet.elements[c].connectsTo.elements[p].connectionID.value;
                        }
                    }
                    // End Sequence of ConnectsToList
                }
                if(ros->map.intersections.elements[a].laneSet.elements[c].overlaysPresent) {

                    // Start SEQUENCE OF OverlayLaneList
                    ros->map.intersections.elements[a].laneSet.elements[c].overlays.count =
                        wind->map.intersections.elements[a].laneSet.elements[c].overlays.count;

                    int count_r = ros->map.intersections.elements[a].laneSet.elements[c].overlays.count;

                    for(int r = 0; r < count_r; r++) {
                        dsrc_v2_dsrc::LaneID tmp_r;
                        ros->map.intersections.elements[a].laneSet.elements[c].overlays.elements.push_back(tmp_r);

                        ros->map.intersections.elements[a].laneSet.elements[c].overlays.elements[r].value =
                            wind->map.intersections.elements[a].laneSet.elements[c].overlays.elements[r].value;
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
        ros->map.restrictionList.count =
            wind->map.restrictionList.count;

        int count_s = ros->map.restrictionList.count;

        for(int s = 0; s < count_s; s++) {
            dsrc_v2_dsrc::RestrictionClassAssignment tmp_s;
            ros->map.restrictionList.elements.push_back(tmp_s);

            ros->map.restrictionList.elements[s].id.value =
                wind->map.restrictionList.elements[s].id.value;

            // Start SEQUENCE OF RestrictionUserTypeList
            ros->map.restrictionList.elements[s].users.count =
                wind->map.restrictionList.elements[s].users.count;

            int count_t = ros->map.restrictionList.elements[s].users.count;

            for(int t = 0; t < count_t; t++) {
                dsrc_v2_dsrc::RestrictionUserType tmp_t;
                ros->map.restrictionList.elements[s].users.elements.push_back(tmp_t);

                // START CHOICE RestrictionUserType
                ros->map.restrictionList.elements[s].users.elements[t].choice = wind->map.restrictionList.elements[s].users.elements[t].choice;

                if(ros->map.restrictionList.elements[s].users.elements[t].choice == 0) {

                    ros->map.restrictionList.elements[s].users.elements[t].basicType.value =
                        wind->map.restrictionList.elements[s].users.elements[t].basicType.value;
                }
                // END CHOICE RestrictionUserType
            }
            // End Sequence of RestrictionUserTypeList
        }
        // End Sequence of RestrictionClassList
    }
}
