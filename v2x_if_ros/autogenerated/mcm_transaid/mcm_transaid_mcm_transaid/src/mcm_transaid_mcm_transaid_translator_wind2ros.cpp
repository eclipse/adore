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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_transaid:3.0
 * 
 * Module: MCM_TransAID {version(3)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <mcm_transaid_mcm_transaid_translator_wind2ros.h>

void wind::wind_ros::wind2ros(mcm_transaid_mcm_transaid::MCM* ros, wind::cpp::MCM_TransAID::MCM* wind)
{

    ros->header.protocolVersion.value =
        wind->header.protocolVersion.value;

    ros->header.messageID.value =
        wind->header.messageID.value;

    ros->header.stationID.value =
        wind->header.stationID.value;

    ros->maneuverCoordination.generationDeltaTime.value =
        wind->maneuverCoordination.generationDeltaTime.value;

    ros->maneuverCoordination.mcmParameters.basicContainer.stationType.value =
        wind->maneuverCoordination.mcmParameters.basicContainer.stationType.value;

    ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.latitude.value =
        wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.latitude.value;

    ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.longitude.value =
        wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.longitude.value;

    ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence.value =
        wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence.value;

    ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence.value =
        wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence.value;

    ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation.value =
        wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation.value;

    ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.altitude.altitudeValue.value =
        wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.altitude.altitudeValue.value;

    ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.altitude.altitudeConfidence.value =
        wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.altitude.altitudeConfidence.value;

    // START CHOICE ManeuverContainer
    ros->maneuverCoordination.mcmParameters.maneuverContainer.choice = wind->maneuverCoordination.mcmParameters.maneuverContainer.choice;

    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.choice == 0) {
        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectoryPresent =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectoryPresent;
        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToCPresent =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToCPresent;
        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevelPresent =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevelPresent;
        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRMPresent =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRMPresent;
        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseListPresent =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseListPresent;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.toleratedDistanceAheadCmps.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.toleratedDistanceAheadCmps.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.toleratedDistanceBehindCmps.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.toleratedDistanceBehindCmps.value;

        // Start SEQUENCE OF PlannedTrajectory
        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.count =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.count;

        int count_a = ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.count;

        for(int a = 0; a < count_a; a++) {
            mcm_transaid_mcm_transaid::TrajectoryPoint tmp_a;
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements.push_back(tmp_a);
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeedPresent =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeedPresent;
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAccelerationPresent =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAccelerationPresent;

            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaXCm.value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaXCm.value;

            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaYCm.value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaYCm.value;

            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaTimeMs.value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaTimeMs.value;
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeedPresent) {

                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeed.value =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeed.value;
            }
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAccelerationPresent) {

                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAcceleration.value =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAcceleration.value;
            }
        }
        // End Sequence of PlannedTrajectory
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectoryPresent) {

            // Start SEQUENCE OF DesiredTrajectory
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.count =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.count;

            int count_b = ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.count;

            for(int b = 0; b < count_b; b++) {
                mcm_transaid_mcm_transaid::TrajectoryPoint tmp_b;
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements.push_back(tmp_b);
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeedPresent =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeedPresent;
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAccelerationPresent =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAccelerationPresent;

                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaXCm.value =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaXCm.value;

                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaYCm.value =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaYCm.value;

                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaTimeMs.value =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaTimeMs.value;
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeedPresent) {

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeed.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeed.value;
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAccelerationPresent) {

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAcceleration.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAcceleration.value;
                }
            }
            // End Sequence of DesiredTrajectory
        }

        // Start SEQUENCE OF RespectedDesiredTrajectoriesList
        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.count =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.count;

        int count_c = ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.count;

        for(int c = 0; c < count_c; c++) {
            mcm_transaid_mcm_transaid::RespectedDesiredTrajectory tmp_c;
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.elements.push_back(tmp_c);

            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.elements[c].value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.elements[c].value;
        }
        // End Sequence of RespectedDesiredTrajectoriesList
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToCPresent) {

            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToC.minute.value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToC.minute.value;

            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToC.millisecond.value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToC.millisecond.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevelPresent) {

            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevel.value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevel.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRMPresent) {

            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRM.value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRM.value;
        }

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingValue.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingValue.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingConfidence.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingConfidence.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.speed.speedValue.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.speed.speedValue.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.speed.speedConfidence.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.speed.speedConfidence.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.longitudinalAcceleration.longitudinalAccelerationValue.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.longitudinalAcceleration.longitudinalAccelerationValue.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.longitudinalAcceleration.longitudinalAccelerationConfidence.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.longitudinalAcceleration.longitudinalAccelerationConfidence.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lateralAcceleration.lateralAccelerationValue.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lateralAcceleration.lateralAccelerationValue.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lateralAcceleration.lateralAccelerationConfidence.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lateralAcceleration.lateralAccelerationConfidence.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.verticalAcceleration.verticalAccelerationValue.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.verticalAcceleration.verticalAccelerationValue.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.verticalAcceleration.verticalAccelerationConfidence.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.verticalAcceleration.verticalAccelerationConfidence.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.yawRate.yawRateValue.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.yawRate.yawRateValue.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.yawRate.yawRateConfidence.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.yawRate.yawRateConfidence.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvature.curvatureValue.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvature.curvatureValue.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvature.curvatureConfidence.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvature.curvatureConfidence.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvatureCalculationMode.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvatureCalculationMode.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.driveDirection.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.driveDirection.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lanePosition.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lanePosition.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.steeringWheelAngle.steeringWheelAngleValue.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.steeringWheelAngle.steeringWheelAngleValue.value;

        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.steeringWheelAngle.steeringWheelAngleConfidence.value =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.steeringWheelAngle.steeringWheelAngleConfidence.value;
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseListPresent) {

            // Start SEQUENCE OF AdviceResponseList
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.count =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.count;

            int count_d = ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.count;

            for(int d = 0; d < count_d; d++) {
                mcm_transaid_mcm_transaid::AdviceResponse tmp_d;
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements.push_back(tmp_d);

                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements[d].adviceID.value =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements[d].adviceID.value;
                
                // START BIT STRING AdviceFollowed
                for(int e = 0; e < 2; e++) {
                    uint8_t tmp_e;
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements[d].adviceFollowed.values.push_back(tmp_e);
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements[d].adviceFollowed.values[e] = 
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements[d].adviceFollowed.values[e];
                }
                // END BIT STRING AdviceFollowed
                
            }
            // End Sequence of AdviceResponseList
        }
    }
    else   // CHOICE ManeuverContainer
    {
        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceIDPresent =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceIDPresent;
        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceIDPresent =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceIDPresent;
        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceListPresent =
            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceListPresent;
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceIDPresent) {
            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.regionPresent =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.regionPresent;
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.regionPresent) {

                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.region.value =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.region.value;
            }

            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.id.value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.id.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceIDPresent) {
            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.regionPresent =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.regionPresent;
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.regionPresent) {

                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.region.value =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.region.value;
            }

            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.id.value =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.id.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceListPresent) {

            // Start SEQUENCE OF VehicleAdviceList
            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.count =
                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.count;

            int count_f = ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.count;

            for(int f = 0; f < count_f; f++) {
                mcm_transaid_mcm_transaid::VehicleAdvice tmp_f;
                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements.push_back(tmp_f);
                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvicePresent =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvicePresent;
                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvicePresent =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvicePresent;
                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvicePresent =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvicePresent;

                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].targetStationID.value =
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].targetStationID.value;
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvicePresent) {
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeedPresent =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeedPresent;
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehiclePresent =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehiclePresent;
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehiclePresent =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehiclePresent;
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToCPresent =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToCPresent;

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.adviceID.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.adviceID.value;

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneAdviceReason.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneAdviceReason.value;

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangePosition.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangePosition.value;

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeMoment.minute.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeMoment.minute.value;

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeMoment.millisecond.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeMoment.millisecond.value;
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeedPresent) {

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeed.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeed.value;
                    }
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehiclePresent) {

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehicle.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehicle.value;
                    }
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehiclePresent) {

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehicle.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehicle.value;
                    }

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.targetLane.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.targetLane.value;
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToCPresent) {

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToC.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToC.value;
                    }
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvicePresent) {

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.adviceID.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.adviceID.value;

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.adviceLaneID.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.adviceLaneID.value;

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.advicePosition.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.advicePosition.value;

                    // START CHOICE DesiredBehaviour
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.choice = wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.choice;

                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.choice == 0) {

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.targetGap.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.targetGap.value;
                    }
                    else   // CHOICE DesiredBehaviour
                    {

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.targetSpeed.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.targetSpeed.value;
                    }
                    // END CHOICE DesiredBehaviour
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvicePresent) {
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransitionPresent =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransitionPresent;
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransitionPresent =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransitionPresent;
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransitionPresent =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransitionPresent;

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.adviceID.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.adviceID.value;

                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.tocAdviceReason.value =
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.tocAdviceReason.value;
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransitionPresent) {

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransition.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransition.value;
                    }
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransitionPresent) {

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransition.minute.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransition.minute.value;

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransition.millisecond.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransition.millisecond.value;
                    }
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransitionPresent) {

                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransition.value =
                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransition.value;
                    }
                }
            }
            // End Sequence of VehicleAdviceList
        }
    }
    // END CHOICE ManeuverContainer
}
