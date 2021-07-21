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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:cam_v2:1.4
 * 
 * Module: CAM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) cam(2) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <cam_v2_cam_pdu_descriptions_translator_wind2ros.h>

void wind::wind_ros::wind2ros(cam_v2_cam_pdu_descriptions::CAM* ros, wind::cpp::CAM_PDU_Descriptions::CAM* wind)
{

    ros->header.protocolVersion.value =
        wind->header.protocolVersion.value;

    ros->header.messageID.value =
        wind->header.messageID.value;

    ros->header.stationID.value =
        wind->header.stationID.value;

    ros->cam.generationDeltaTime.value =
        wind->cam.generationDeltaTime.value;
    ros->cam.camParameters.lowFrequencyContainerPresent =
        wind->cam.camParameters.lowFrequencyContainerPresent;
    ros->cam.camParameters.specialVehicleContainerPresent =
        wind->cam.camParameters.specialVehicleContainerPresent;

    ros->cam.camParameters.basicContainer.stationType.value =
        wind->cam.camParameters.basicContainer.stationType.value;

    ros->cam.camParameters.basicContainer.referencePosition.latitude.value =
        wind->cam.camParameters.basicContainer.referencePosition.latitude.value;

    ros->cam.camParameters.basicContainer.referencePosition.longitude.value =
        wind->cam.camParameters.basicContainer.referencePosition.longitude.value;

    ros->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence.value =
        wind->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence.value;

    ros->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence.value =
        wind->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence.value;

    ros->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation.value =
        wind->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation.value;

    ros->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue.value =
        wind->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue.value;

    ros->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence.value =
        wind->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence.value;

    // START CHOICE HighFrequencyContainer
    ros->cam.camParameters.highFrequencyContainer.choice = wind->cam.camParameters.highFrequencyContainer.choice;

    if(ros->cam.camParameters.highFrequencyContainer.choice == 0) {
        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.accelerationControlPresent =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.accelerationControlPresent;
        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lanePositionPresent =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lanePositionPresent;
        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.steeringWheelAnglePresent =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.steeringWheelAnglePresent;
        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAccelerationPresent =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAccelerationPresent;
        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.verticalAccelerationPresent =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.verticalAccelerationPresent;
        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.performanceClassPresent =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.performanceClassPresent;
        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZonePresent =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZonePresent;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading.headingValue.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading.headingValue.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading.headingConfidence.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading.headingConfidence.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed.speedValue.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed.speedValue.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed.speedConfidence.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed.speedConfidence.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.driveDirection.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.driveDirection.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleWidth.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.vehicleWidth.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.curvature.curvatureValue.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.curvature.curvatureValue.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.curvature.curvatureConfidence.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.curvature.curvatureConfidence.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.curvatureCalculationMode.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.curvatureCalculationMode.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.yawRate.yawRateValue.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.yawRate.yawRateValue.value;

        ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence.value =
            wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence.value;
        if(ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.accelerationControlPresent) {
            
            // START BIT STRING AccelerationControl
            for(int a = 0; a < 7; a++) {
                uint8_t tmp_a;
                ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.accelerationControl.values.push_back(tmp_a);
                ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.accelerationControl.values[a] = 
                    wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.accelerationControl.values[a];
            }
            // END BIT STRING AccelerationControl
            
        }
        if(ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lanePositionPresent) {

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lanePosition.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lanePosition.value;
        }
        if(ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.steeringWheelAnglePresent) {

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.steeringWheelAngle.steeringWheelAngleValue.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.steeringWheelAngle.steeringWheelAngleValue.value;

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.steeringWheelAngle.steeringWheelAngleConfidence.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.steeringWheelAngle.steeringWheelAngleConfidence.value;
        }
        if(ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAccelerationPresent) {

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAcceleration.lateralAccelerationValue.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAcceleration.lateralAccelerationValue.value;

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAcceleration.lateralAccelerationConfidence.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.lateralAcceleration.lateralAccelerationConfidence.value;
        }
        if(ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.verticalAccelerationPresent) {

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.verticalAcceleration.verticalAccelerationValue.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.verticalAcceleration.verticalAccelerationValue.value;

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.verticalAcceleration.verticalAccelerationConfidence.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.verticalAcceleration.verticalAccelerationConfidence.value;
        }
        if(ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.performanceClassPresent) {

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.performanceClass.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.performanceClass.value;
        }
        if(ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZonePresent) {
            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZone.cenDsrcTollingZoneIDPresent =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZone.cenDsrcTollingZoneIDPresent;

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZone.protectedZoneLatitude.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZone.protectedZoneLatitude.value;

            ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZone.protectedZoneLongitude.value =
                wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZone.protectedZoneLongitude.value;
            if(ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZone.cenDsrcTollingZoneIDPresent) {

                ros->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZone.cenDsrcTollingZoneID.value =
                    wind->cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.cenDsrcTollingZone.cenDsrcTollingZoneID.value;
            }
        }
    }
    else   // CHOICE HighFrequencyContainer
    {
        ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSUPresent =
            wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSUPresent;
        if(ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSUPresent) {

            // Start SEQUENCE OF ProtectedCommunicationZonesRSU
            ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.count =
                wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.count;

            int count_b = ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.count;

            for(int b = 0; b < count_b; b++) {
                its_container_v2_its_container::ProtectedCommunicationZone tmp_b;
                ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements.push_back(tmp_b);
                ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].expiryTimePresent =
                    wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].expiryTimePresent;
                ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneRadiusPresent =
                    wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneRadiusPresent;
                ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneIDPresent =
                    wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneIDPresent;

                ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneType.value =
                    wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneType.value;
                if(ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].expiryTimePresent) {

                    ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].expiryTime.value =
                        wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].expiryTime.value;
                }

                ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneLatitude.value =
                    wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneLatitude.value;

                ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneLongitude.value =
                    wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneLongitude.value;
                if(ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneRadiusPresent) {

                    ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneRadius.value =
                        wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneRadius.value;
                }
                if(ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneIDPresent) {

                    ros->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneID.value =
                        wind->cam.camParameters.highFrequencyContainer.rsuContainerHighFrequency.protectedCommunicationZonesRSU.elements[b].protectedZoneID.value;
                }
            }
            // End Sequence of ProtectedCommunicationZonesRSU
        }
    }
    // END CHOICE HighFrequencyContainer
    if(ros->cam.camParameters.lowFrequencyContainerPresent) {

        // START CHOICE LowFrequencyContainer
        ros->cam.camParameters.lowFrequencyContainer.choice = wind->cam.camParameters.lowFrequencyContainer.choice;

        if(ros->cam.camParameters.lowFrequencyContainer.choice == 0) {

            ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.vehicleRole.value =
                wind->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.vehicleRole.value;
            
            // START BIT STRING ExteriorLights
            for(int c = 0; c < 8; c++) {
                uint8_t tmp_c;
                ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.exteriorLights.values.push_back(tmp_c);
                ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.exteriorLights.values[c] = 
                    wind->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.exteriorLights.values[c];
            }
            // END BIT STRING ExteriorLights
            

            // Start SEQUENCE OF PathHistory
            ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.count =
                wind->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.count;

            int count_d = ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.count;

            for(int d = 0; d < count_d; d++) {
                its_container_v2_its_container::PathPoint tmp_d;
                ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements.push_back(tmp_d);
                ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathDeltaTimePresent =
                    wind->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathDeltaTimePresent;

                ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathPosition.deltaLatitude.value =
                    wind->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathPosition.deltaLatitude.value;

                ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathPosition.deltaLongitude.value =
                    wind->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathPosition.deltaLongitude.value;

                ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathPosition.deltaAltitude.value =
                    wind->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathPosition.deltaAltitude.value;
                if(ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathDeltaTimePresent) {

                    ros->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathDeltaTime.value =
                        wind->cam.camParameters.lowFrequencyContainer.basicVehicleContainerLowFrequency.pathHistory.elements[d].pathDeltaTime.value;
                }
            }
            // End Sequence of PathHistory
        }
        // END CHOICE LowFrequencyContainer
    }
    if(ros->cam.camParameters.specialVehicleContainerPresent) {

        // START CHOICE SpecialVehicleContainer
        ros->cam.camParameters.specialVehicleContainer.choice = wind->cam.camParameters.specialVehicleContainer.choice;

        if(ros->cam.camParameters.specialVehicleContainer.choice == 0) {
            ros->cam.camParameters.specialVehicleContainer.publicTransportContainer.ptActivationPresent =
                wind->cam.camParameters.specialVehicleContainer.publicTransportContainer.ptActivationPresent;

            ros->cam.camParameters.specialVehicleContainer.publicTransportContainer.embarkationStatus.value =
                wind->cam.camParameters.specialVehicleContainer.publicTransportContainer.embarkationStatus.value;
            if(ros->cam.camParameters.specialVehicleContainer.publicTransportContainer.ptActivationPresent) {

                ros->cam.camParameters.specialVehicleContainer.publicTransportContainer.ptActivation.ptActivationType.value =
                    wind->cam.camParameters.specialVehicleContainer.publicTransportContainer.ptActivation.ptActivationType.value;
                
                ros->cam.camParameters.specialVehicleContainer.publicTransportContainer.ptActivation.ptActivationData.count =
                    wind->cam.camParameters.specialVehicleContainer.publicTransportContainer.ptActivation.ptActivationData.count;
                for(int e = 0; e < 20; e++)
                    ros->cam.camParameters.specialVehicleContainer.publicTransportContainer.ptActivation.ptActivationData.value[e] =
                        wind->cam.camParameters.specialVehicleContainer.publicTransportContainer.ptActivation.ptActivationData.value[e];
            }
        }
        else if(ros->cam.camParameters.specialVehicleContainer.choice == 1)  // CHOICE SpecialVehicleContainer
        {
            
            // START BIT STRING SpecialTransportType
            for(int f = 0; f < 4; f++) {
                uint8_t tmp_f;
                ros->cam.camParameters.specialVehicleContainer.specialTransportContainer.specialTransportType.values.push_back(tmp_f);
                ros->cam.camParameters.specialVehicleContainer.specialTransportContainer.specialTransportType.values[f] = 
                    wind->cam.camParameters.specialVehicleContainer.specialTransportContainer.specialTransportType.values[f];
            }
            // END BIT STRING SpecialTransportType
            
            
            // START BIT STRING LightBarSirenInUse
            for(int g = 0; g < 2; g++) {
                uint8_t tmp_g;
                ros->cam.camParameters.specialVehicleContainer.specialTransportContainer.lightBarSirenInUse.values.push_back(tmp_g);
                ros->cam.camParameters.specialVehicleContainer.specialTransportContainer.lightBarSirenInUse.values[g] = 
                    wind->cam.camParameters.specialVehicleContainer.specialTransportContainer.lightBarSirenInUse.values[g];
            }
            // END BIT STRING LightBarSirenInUse
            
        }
        else if(ros->cam.camParameters.specialVehicleContainer.choice == 2)  // CHOICE SpecialVehicleContainer
        {

            ros->cam.camParameters.specialVehicleContainer.dangerousGoodsContainer.dangerousGoodsBasic.value =
                wind->cam.camParameters.specialVehicleContainer.dangerousGoodsContainer.dangerousGoodsBasic.value;
        }
        else if(ros->cam.camParameters.specialVehicleContainer.choice == 3)  // CHOICE SpecialVehicleContainer
        {
            ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.roadworksSubCauseCodePresent =
                wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.roadworksSubCauseCodePresent;
            ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanesPresent =
                wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanesPresent;
            if(ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.roadworksSubCauseCodePresent) {

                ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.roadworksSubCauseCode.value =
                    wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.roadworksSubCauseCode.value;
            }
            
            // START BIT STRING LightBarSirenInUse
            for(int h = 0; h < 2; h++) {
                uint8_t tmp_h;
                ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.lightBarSirenInUse.values.push_back(tmp_h);
                ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.lightBarSirenInUse.values[h] = 
                    wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.lightBarSirenInUse.values[h];
            }
            // END BIT STRING LightBarSirenInUse
            
            if(ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanesPresent) {
                ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.innerhardShoulderStatusPresent =
                    wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.innerhardShoulderStatusPresent;
                ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.outerhardShoulderStatusPresent =
                    wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.outerhardShoulderStatusPresent;
                ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.drivingLaneStatusPresent =
                    wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.drivingLaneStatusPresent;
                if(ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.innerhardShoulderStatusPresent) {

                    ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.innerhardShoulderStatus.value =
                        wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.innerhardShoulderStatus.value;
                }
                if(ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.outerhardShoulderStatusPresent) {

                    ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.outerhardShoulderStatus.value =
                        wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.outerhardShoulderStatus.value;
                }
                if(ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.drivingLaneStatusPresent) {
                    
                    // START BIT STRING DrivingLaneStatus
                    for(int i = 0; i < 13; i++) {
                        uint8_t tmp_i;
                        ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.drivingLaneStatus.values.push_back(tmp_i);
                        ros->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.drivingLaneStatus.values[i] = 
                            wind->cam.camParameters.specialVehicleContainer.roadWorksContainerBasic.closedLanes.drivingLaneStatus.values[i];
                    }
                    // END BIT STRING DrivingLaneStatus
                    
                }
            }
        }
        else if(ros->cam.camParameters.specialVehicleContainer.choice == 4)  // CHOICE SpecialVehicleContainer
        {
            
            // START BIT STRING LightBarSirenInUse
            for(int j = 0; j < 2; j++) {
                uint8_t tmp_j;
                ros->cam.camParameters.specialVehicleContainer.rescueContainer.lightBarSirenInUse.values.push_back(tmp_j);
                ros->cam.camParameters.specialVehicleContainer.rescueContainer.lightBarSirenInUse.values[j] = 
                    wind->cam.camParameters.specialVehicleContainer.rescueContainer.lightBarSirenInUse.values[j];
            }
            // END BIT STRING LightBarSirenInUse
            
        }
        else if(ros->cam.camParameters.specialVehicleContainer.choice == 5)  // CHOICE SpecialVehicleContainer
        {
            ros->cam.camParameters.specialVehicleContainer.emergencyContainer.incidentIndicationPresent =
                wind->cam.camParameters.specialVehicleContainer.emergencyContainer.incidentIndicationPresent;
            ros->cam.camParameters.specialVehicleContainer.emergencyContainer.emergencyPriorityPresent =
                wind->cam.camParameters.specialVehicleContainer.emergencyContainer.emergencyPriorityPresent;
            
            // START BIT STRING LightBarSirenInUse
            for(int k = 0; k < 2; k++) {
                uint8_t tmp_k;
                ros->cam.camParameters.specialVehicleContainer.emergencyContainer.lightBarSirenInUse.values.push_back(tmp_k);
                ros->cam.camParameters.specialVehicleContainer.emergencyContainer.lightBarSirenInUse.values[k] = 
                    wind->cam.camParameters.specialVehicleContainer.emergencyContainer.lightBarSirenInUse.values[k];
            }
            // END BIT STRING LightBarSirenInUse
            
            if(ros->cam.camParameters.specialVehicleContainer.emergencyContainer.incidentIndicationPresent) {

                ros->cam.camParameters.specialVehicleContainer.emergencyContainer.incidentIndication.causeCode.value =
                    wind->cam.camParameters.specialVehicleContainer.emergencyContainer.incidentIndication.causeCode.value;

                ros->cam.camParameters.specialVehicleContainer.emergencyContainer.incidentIndication.subCauseCode.value =
                    wind->cam.camParameters.specialVehicleContainer.emergencyContainer.incidentIndication.subCauseCode.value;
            }
            if(ros->cam.camParameters.specialVehicleContainer.emergencyContainer.emergencyPriorityPresent) {
                
                // START BIT STRING EmergencyPriority
                for(int l = 0; l < 2; l++) {
                    uint8_t tmp_l;
                    ros->cam.camParameters.specialVehicleContainer.emergencyContainer.emergencyPriority.values.push_back(tmp_l);
                    ros->cam.camParameters.specialVehicleContainer.emergencyContainer.emergencyPriority.values[l] = 
                        wind->cam.camParameters.specialVehicleContainer.emergencyContainer.emergencyPriority.values[l];
                }
                // END BIT STRING EmergencyPriority
                
            }
        }
        else   // CHOICE SpecialVehicleContainer
        {
            ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.incidentIndicationPresent =
                wind->cam.camParameters.specialVehicleContainer.safetyCarContainer.incidentIndicationPresent;
            ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.trafficRulePresent =
                wind->cam.camParameters.specialVehicleContainer.safetyCarContainer.trafficRulePresent;
            ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.speedLimitPresent =
                wind->cam.camParameters.specialVehicleContainer.safetyCarContainer.speedLimitPresent;
            
            // START BIT STRING LightBarSirenInUse
            for(int m = 0; m < 2; m++) {
                uint8_t tmp_m;
                ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.lightBarSirenInUse.values.push_back(tmp_m);
                ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.lightBarSirenInUse.values[m] = 
                    wind->cam.camParameters.specialVehicleContainer.safetyCarContainer.lightBarSirenInUse.values[m];
            }
            // END BIT STRING LightBarSirenInUse
            
            if(ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.incidentIndicationPresent) {

                ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.incidentIndication.causeCode.value =
                    wind->cam.camParameters.specialVehicleContainer.safetyCarContainer.incidentIndication.causeCode.value;

                ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.incidentIndication.subCauseCode.value =
                    wind->cam.camParameters.specialVehicleContainer.safetyCarContainer.incidentIndication.subCauseCode.value;
            }
            if(ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.trafficRulePresent) {

                ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.trafficRule.value =
                    wind->cam.camParameters.specialVehicleContainer.safetyCarContainer.trafficRule.value;
            }
            if(ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.speedLimitPresent) {

                ros->cam.camParameters.specialVehicleContainer.safetyCarContainer.speedLimit.value =
                    wind->cam.camParameters.specialVehicleContainer.safetyCarContainer.speedLimit.value;
            }
        }
        // END CHOICE SpecialVehicleContainer
    }
}
