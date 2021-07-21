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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:cpm_v1_19:1.3
 * 
 * Module: CPM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) ts(103324) cpm(1) version1(1)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <cpm_v1_19_cpm_pdu_descriptions_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const cpm_v1_19_cpm_pdu_descriptions::CPM::ConstPtr& ros, wind::cpp::CPM_PDU_Descriptions::CPM* wind)
{

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;

    wind->cpm.generationDeltaTime.value =
        ros->cpm.generationDeltaTime.value;
    wind->cpm.cpmParameters.perceptionDataPresent =
        ros->cpm.cpmParameters.perceptionDataPresent;
    wind->cpm.cpmParameters.managementContainer.messageSegmentInfoPresent =
        ros->cpm.cpmParameters.managementContainer.messageSegmentInfoPresent;

    wind->cpm.cpmParameters.managementContainer.stationType.value =
        ros->cpm.cpmParameters.managementContainer.stationType.value;
    if(ros->cpm.cpmParameters.managementContainer.messageSegmentInfoPresent) {

        wind->cpm.cpmParameters.managementContainer.messageSegmentInfo.totalMsgSegments.value =
            ros->cpm.cpmParameters.managementContainer.messageSegmentInfo.totalMsgSegments.value;

        wind->cpm.cpmParameters.managementContainer.messageSegmentInfo.thisSegmentNum.value =
            ros->cpm.cpmParameters.managementContainer.messageSegmentInfo.thisSegmentNum.value;
    }

    wind->cpm.cpmParameters.managementContainer.referencePosition.latitude.value =
        ros->cpm.cpmParameters.managementContainer.referencePosition.latitude.value;

    wind->cpm.cpmParameters.managementContainer.referencePosition.longitude.value =
        ros->cpm.cpmParameters.managementContainer.referencePosition.longitude.value;

    wind->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence.value =
        ros->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence.value;

    wind->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence.value =
        ros->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence.value;

    wind->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation.value =
        ros->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation.value;

    wind->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeValue.value =
        ros->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeValue.value;

    wind->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeConfidence.value =
        ros->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeConfidence.value;
    if(ros->cpm.cpmParameters.perceptionDataPresent) {

        // Start SEQUENCE OF CpmParameters_perceptionData
        wind->cpm.cpmParameters.perceptionData.count =
            ros->cpm.cpmParameters.perceptionData.count;

        int count_a = ros->cpm.cpmParameters.perceptionData.count;

        for(int a = 0; a < count_a; a++) {

            // START CHOICE CpmPerceptionDataContainer
            wind->cpm.cpmParameters.perceptionData.elements[a].choice = ros->cpm.cpmParameters.perceptionData.elements[a].choice;

            if(ros->cpm.cpmParameters.perceptionData.elements[a].choice == 0) {

                // Start SEQUENCE OF SensorInformationContainer
                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.count =
                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.count;

                int count_b = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.count;

                for(int b = 0; b < count_b; b++) {
                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].freeSpaceConfidencePresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].freeSpaceConfidencePresent;

                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].sensorID.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].sensorID.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].type.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].type.value;

                    // START CHOICE DetectionArea
                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.choice;

                    if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.choice == 0) {
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.zSensorOffsetPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.zSensorOffsetPresent;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.refPointId.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.refPointId.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.xSensorOffset.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.xSensorOffset.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.ySensorOffset.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.ySensorOffset.value;
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.zSensorOffsetPresent) {

                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.zSensorOffset.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.zSensorOffset.value;
                        }

                        // Start SEQUENCE OF VehicleSensorPropertyList
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.count =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.count;

                        int count_c = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.count;

                        for(int c = 0; c < count_c; c++) {
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleStartPresent =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleStartPresent;
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleEndPresent =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleEndPresent;

                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].range.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].range.value;

                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].horizontalOpeningAngleStart.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].horizontalOpeningAngleStart.value;

                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].horizontalOpeningAngleEnd.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].horizontalOpeningAngleEnd.value;
                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleStartPresent) {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleStart.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleStart.value;
                            }
                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleEndPresent) {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleEnd.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.vehicleSensor.vehicleSensorPropertyList.elements[c].verticalOpeningAngleEnd.value;
                            }
                        }
                        // End Sequence of VehicleSensorPropertyList
                    }
                    else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.choice == 1)  // CHOICE DetectionArea
                    {
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleStartPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleStartPresent;
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleEndPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleEndPresent;
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffsetPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffsetPresent;
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorHeightPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorHeightPresent;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.range.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.range.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.stationaryHorizontalOpeningAngleStart.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.stationaryHorizontalOpeningAngleStart.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.stationaryHorizontalOpeningAngleEnd.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.stationaryHorizontalOpeningAngleEnd.value;
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleStartPresent) {

                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleStart.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleStart.value;
                        }
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleEndPresent) {

                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleEnd.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.verticalOpeningAngleEnd.value;
                        }
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffsetPresent) {
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZPresent =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZPresent;

                            // START CHOICE NodeOffsetPointXY
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.choice;

                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.choice == 0) {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY1.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY1.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY1.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY1.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.choice == 1)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY2.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY2.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY2.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY2.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.choice == 2)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY3.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY3.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY3.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY3.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.choice == 3)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY4.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY4.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY4.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY4.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.choice == 4)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY5.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY5.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY5.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY5.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.choice == 5)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY6.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY6.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY6.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_XY6.y.value;
                            }
                            else   // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_LatLon.lon.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_LatLon.lon.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_LatLon.lat.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointxy.node_LatLon.lat.value;
                            }
                            // END CHOICE NodeOffsetPointXY
                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZPresent) {

                                // START CHOICE NodeOffsetPointZ
                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.choice;

                                if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.choice == 0) {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z1.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z1.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.choice == 1)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z2.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z2.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.choice == 2)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z3.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z3.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.choice == 3)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z4.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z4.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.choice == 4)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z5.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z5.value;
                                }
                                else   // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z6.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorPositionOffset.nodeOffsetPointZ.node_Z6.value;
                                }
                                // END CHOICE NodeOffsetPointZ
                            }
                        }
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorHeightPresent) {

                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorHeight.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRadial.sensorHeight.value;
                        }
                    }
                    else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.choice == 2)  // CHOICE DetectionArea
                    {

                        // Start SEQUENCE OF PolyPointList
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.count =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.count;

                        int count_d = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.count;

                        for(int d = 0; d < count_d; d++) {
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZPresent =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZPresent;

                            // START CHOICE NodeOffsetPointXY
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.choice;

                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.choice == 0) {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY1.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY1.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY1.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY1.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.choice == 1)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY2.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY2.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY2.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY2.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.choice == 2)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY3.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY3.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY3.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY3.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.choice == 3)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY4.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY4.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY4.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY4.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.choice == 4)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY5.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY5.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY5.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY5.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.choice == 5)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY6.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY6.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY6.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_XY6.y.value;
                            }
                            else   // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_LatLon.lon.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_LatLon.lon.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_LatLon.lat.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointxy.node_LatLon.lat.value;
                            }
                            // END CHOICE NodeOffsetPointXY
                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZPresent) {

                                // START CHOICE NodeOffsetPointZ
                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.choice;

                                if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.choice == 0) {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z1.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z1.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.choice == 1)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z2.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z2.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.choice == 2)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z3.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z3.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.choice == 3)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z4.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z4.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.choice == 4)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z5.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z5.value;
                                }
                                else   // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z6.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorPolygon.polyPointList.elements[d].nodeOffsetPointZ.node_Z6.value;
                                }
                                // END CHOICE NodeOffsetPointZ
                            }
                        }
                        // End Sequence of PolyPointList
                    }
                    else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.choice == 3)  // CHOICE DetectionArea
                    {
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPointPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPointPresent;
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPointPresent) {
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZPresent =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZPresent;

                            // START CHOICE NodeOffsetPointXY
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.choice;

                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.choice == 0) {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY1.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY1.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY1.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY1.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.choice == 1)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY2.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY2.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY2.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY2.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.choice == 2)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY3.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY3.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY3.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY3.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.choice == 3)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY4.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY4.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY4.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY4.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.choice == 4)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY5.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY5.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY5.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY5.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.choice == 5)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY6.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY6.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY6.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_XY6.y.value;
                            }
                            else   // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lon.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lon.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lat.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lat.value;
                            }
                            // END CHOICE NodeOffsetPointXY
                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZPresent) {

                                // START CHOICE NodeOffsetPointZ
                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.choice;

                                if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.choice == 0) {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z1.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z1.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.choice == 1)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z2.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z2.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.choice == 2)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z3.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z3.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.choice == 3)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z4.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z4.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.choice == 4)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z5.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z5.value;
                                }
                                else   // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z6.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.nodeCenterPoint.nodeOffsetPointZ.node_Z6.value;
                                }
                                // END CHOICE NodeOffsetPointZ
                            }
                        }

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.radius.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorCircular.radius.value;
                    }
                    else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.choice == 4)  // CHOICE DetectionArea
                    {
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPointPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPointPresent;
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiHeightPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiHeightPresent;
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPointPresent) {
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZPresent =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZPresent;

                            // START CHOICE NodeOffsetPointXY
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.choice;

                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.choice == 0) {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY1.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY1.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY1.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY1.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.choice == 1)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY2.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY2.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY2.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY2.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.choice == 2)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY3.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY3.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY3.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY3.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.choice == 3)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY4.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY4.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY4.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY4.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.choice == 4)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY5.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY5.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY5.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY5.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.choice == 5)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY6.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY6.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY6.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_XY6.y.value;
                            }
                            else   // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lon.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lon.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lat.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lat.value;
                            }
                            // END CHOICE NodeOffsetPointXY
                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZPresent) {

                                // START CHOICE NodeOffsetPointZ
                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.choice;

                                if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.choice == 0) {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z1.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z1.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.choice == 1)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z2.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z2.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.choice == 2)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z3.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z3.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.choice == 3)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z4.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z4.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.choice == 4)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z5.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z5.value;
                                }
                                else   // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z6.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.nodeCenterPoint.nodeOffsetPointZ.node_Z6.value;
                                }
                                // END CHOICE NodeOffsetPointZ
                            }
                        }

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiMinorRangeLength.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiMinorRangeLength.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiMajorRangeLength.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiMajorRangeLength.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiMajorRangeOrientation.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiMajorRangeOrientation.value;
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiHeightPresent) {

                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiHeight.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorEllipse.semiHeight.value;
                        }
                    }
                    else   // CHOICE DetectionArea
                    {
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPointPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPointPresent;
                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiHeightPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiHeightPresent;
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPointPresent) {
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZPresent =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZPresent;

                            // START CHOICE NodeOffsetPointXY
                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.choice;

                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.choice == 0) {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY1.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY1.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY1.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY1.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.choice == 1)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY2.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY2.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY2.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY2.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.choice == 2)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY3.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY3.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY3.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY3.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.choice == 3)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY4.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY4.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY4.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY4.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.choice == 4)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY5.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY5.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY5.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY5.y.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.choice == 5)  // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY6.x.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY6.x.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY6.y.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_XY6.y.value;
                            }
                            else   // CHOICE NodeOffsetPointXY
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lon.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lon.value;

                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lat.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointxy.node_LatLon.lat.value;
                            }
                            // END CHOICE NodeOffsetPointXY
                            if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZPresent) {

                                // START CHOICE NodeOffsetPointZ
                                wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.choice = ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.choice;

                                if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.choice == 0) {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z1.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z1.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.choice == 1)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z2.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z2.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.choice == 2)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z3.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z3.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.choice == 3)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z4.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z4.value;
                                }
                                else if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.choice == 4)  // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z5.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z5.value;
                                }
                                else   // CHOICE NodeOffsetPointZ
                                {

                                    wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z6.value =
                                        ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.nodeCenterPoint.nodeOffsetPointZ.node_Z6.value;
                                }
                                // END CHOICE NodeOffsetPointZ
                            }
                        }

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiMajorRangeLength.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiMajorRangeLength.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiMinorRangeLength.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiMinorRangeLength.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiMajorRangeOrientation.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiMajorRangeOrientation.value;
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiHeightPresent) {

                            wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiHeight.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].detectionArea.stationarySensorRectangle.semiHeight.value;
                        }
                    }
                    // END CHOICE DetectionArea
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].freeSpaceConfidencePresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].freeSpaceConfidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].sensorInformationContainer.elements[b].freeSpaceConfidence.value;
                    }
                }
                // End Sequence of SensorInformationContainer
            }
            else   // CHOICE CpmPerceptionDataContainer
            {

                wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.numberOfPerceivedObjects.value =
                    ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.numberOfPerceivedObjects.value;

                // Start SEQUENCE OF PerceivedObjectContainer_perceivedObjects
                wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.count =
                    ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.count;

                int count_e = ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.count;

                for(int e = 0; e < count_e; e++) {
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zDistancePresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zDistancePresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zSpeedPresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zSpeedPresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xAccelerationPresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xAccelerationPresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yAccelerationPresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yAccelerationPresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zAccelerationPresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zAccelerationPresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yawAnglePresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yawAnglePresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension1Present =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension1Present;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension2Present =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension2Present;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].verticalObjectDimensionPresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].verticalObjectDimensionPresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectAgePresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectAgePresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].sensorIDListPresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].sensorIDListPresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].dynamicStatusPresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].dynamicStatusPresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classificationPresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classificationPresent;
                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPositionPresent =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPositionPresent;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectID.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectID.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].timeOfMeasurement.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].timeOfMeasurement.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectConfidence.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectConfidence.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xDistance.value.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xDistance.value.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xDistance.confidence.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xDistance.confidence.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yDistance.value.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yDistance.value.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yDistance.confidence.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yDistance.confidence.value;
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zDistancePresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zDistance.value.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zDistance.value.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zDistance.confidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zDistance.confidence.value;
                    }

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xSpeed.value.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xSpeed.value.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xSpeed.confidence.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xSpeed.confidence.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].ySpeed.value.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].ySpeed.value.value;

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].ySpeed.confidence.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].ySpeed.confidence.value;
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zSpeedPresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zSpeed.value.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zSpeed.value.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zSpeed.confidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zSpeed.confidence.value;
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xAccelerationPresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xAcceleration.longitudinalAccelerationValue.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xAcceleration.longitudinalAccelerationValue.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xAcceleration.longitudinalAccelerationConfidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].xAcceleration.longitudinalAccelerationConfidence.value;
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yAccelerationPresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yAcceleration.lateralAccelerationValue.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yAcceleration.lateralAccelerationValue.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yAcceleration.lateralAccelerationConfidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yAcceleration.lateralAccelerationConfidence.value;
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zAccelerationPresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zAcceleration.verticalAccelerationValue.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zAcceleration.verticalAccelerationValue.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zAcceleration.verticalAccelerationConfidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].zAcceleration.verticalAccelerationConfidence.value;
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yawAnglePresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yawAngle.value.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yawAngle.value.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yawAngle.confidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].yawAngle.confidence.value;
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension1Present) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension1.value.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension1.value.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension1.confidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension1.confidence.value;
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension2Present) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension2.value.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension2.value.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension2.confidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].planarObjectDimension2.confidence.value;
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].verticalObjectDimensionPresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].verticalObjectDimension.value.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].verticalObjectDimension.value.value;

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].verticalObjectDimension.confidence.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].verticalObjectDimension.confidence.value;
                    }

                    wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectRefPoint.value =
                        ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectRefPoint.value;
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectAgePresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectAge.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].objectAge.value;
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].sensorIDListPresent) {

                        // Start SEQUENCE OF SensorIdList
                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].sensorIDList.count =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].sensorIDList.count;

                        int count_f = ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].sensorIDList.count;

                        for(int f = 0; f < count_f; f++) {

                            wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].sensorIDList.elements[f].value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].sensorIDList.elements[f].value;
                        }
                        // End Sequence of SensorIdList
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].dynamicStatusPresent) {

                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].dynamicStatus.value =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].dynamicStatus.value;
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classificationPresent) {

                        // Start SEQUENCE OF ObjectClassDescription
                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.count =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.count;

                        int count_g = ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.count;

                        for(int g = 0; g < count_g; g++) {

                            // START CHOICE ObjectClass
                            wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.choice = ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.choice;

                            if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.choice == 0) {

                                wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.unknownSubclass.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.unknownSubclass.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.choice == 1)  // CHOICE ObjectClass
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.vehicleSubclass.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.vehicleSubclass.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.choice == 2)  // CHOICE ObjectClass
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.personSubclass.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.personSubclass.value;
                            }
                            else if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.choice == 3)  // CHOICE ObjectClass
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.animalSubclass.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.animalSubclass.value;
                            }
                            else   // CHOICE ObjectClass
                            {

                                wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.otherSubclass.value =
                                    ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].objectClass.otherSubclass.value;
                            }
                            // END CHOICE ObjectClass

                            wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].confidence.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].classification.elements[g].confidence.value;
                        }
                        // End Sequence of ObjectClassDescription
                    }
                    if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPositionPresent) {
                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.laneIDPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.laneIDPresent;
                        wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.longitudinalLanePositionPresent =
                            ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.longitudinalLanePositionPresent;
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.laneIDPresent) {

                            wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.laneID.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.laneID.value;
                        }
                        if(ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.longitudinalLanePositionPresent) {

                            wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.longitudinalLanePosition.longitudinalLanePositionValue.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.longitudinalLanePosition.longitudinalLanePositionValue.value;

                            wind->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.longitudinalLanePosition.longitudinalLanePositionConfidence.value =
                                ros->cpm.cpmParameters.perceptionData.elements[a].perceivedObjectContainer.perceivedObjects.elements[e].matchedPosition.longitudinalLanePosition.longitudinalLanePositionConfidence.value;
                        }
                    }
                }
                // End Sequence of PerceivedObjectContainer_perceivedObjects
            }
            // END CHOICE CpmPerceptionDataContainer
        }
        // End Sequence of CpmParameters_perceptionData
    }
}
