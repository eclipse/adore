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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_dmove:3.0
 * 
 * Module: MCM_DMove {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) mcm(1) version(3)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <mcm_dmove_mcm_dmove_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const mcm_dmove_mcm_dmove::MCM::ConstPtr& ros, wind::cpp::MCM_DMove::MCM* wind)
{

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;

    wind->maneuverCoordination.generationDeltaTime.value =
        ros->maneuverCoordination.generationDeltaTime.value;

    wind->maneuverCoordination.mcmParameters.basicContainer.stationType.value =
        ros->maneuverCoordination.mcmParameters.basicContainer.stationType.value;

    wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.latitude.value =
        ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.latitude.value;

    wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.longitude.value =
        ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.longitude.value;

    wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence.value =
        ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence.value;

    wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence.value =
        ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence.value;

    wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation.value =
        ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation.value;

    wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.altitude.altitudeValue.value =
        ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.altitude.altitudeValue.value;

    wind->maneuverCoordination.mcmParameters.basicContainer.referencePosition.altitude.altitudeConfidence.value =
        ros->maneuverCoordination.mcmParameters.basicContainer.referencePosition.altitude.altitudeConfidence.value;

    // START CHOICE ManeuverContainer
    wind->maneuverCoordination.mcmParameters.maneuverContainer.choice = ros->maneuverCoordination.mcmParameters.maneuverContainer.choice;

    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.choice == 0) {
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectoryPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectoryPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToCPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToCPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevelPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevelPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRMPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRMPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseListPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseListPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinSpeedPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinSpeedPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxSpeedPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxSpeedPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinLongitudinalAccelerationPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinLongitudinalAccelerationPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLongitudinalAccelerationPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLongitudinalAccelerationPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLateralAccelerationPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLateralAccelerationPresent;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceAheadCmps.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceAheadCmps.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceBehindCmps.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceBehindCmps.value;
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinSpeedPresent) {

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinSpeed.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinSpeed.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxSpeedPresent) {

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxSpeed.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxSpeed.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinLongitudinalAccelerationPresent) {

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinLongitudinalAcceleration.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMinLongitudinalAcceleration.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLongitudinalAccelerationPresent) {

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLongitudinalAcceleration.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLongitudinalAcceleration.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLateralAccelerationPresent) {

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLateralAcceleration.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedMaxLateralAcceleration.value;
        }

        // Start SEQUENCE OF PlannedTrajectory
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.count =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.count;

        int count_a = ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.count;

        for(int a = 0; a < count_a; a++) {
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].headingValuePresent =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].headingValuePresent;
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationLeftCmPresent =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationLeftCmPresent;
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationRightCmPresent =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationRightCmPresent;
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationBeforeCsPresent =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationBeforeCsPresent;
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationAfterCsPresent =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationAfterCsPresent;
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeedPresent =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeedPresent;
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAccelerationPresent =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAccelerationPresent;

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaXCm.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaXCm.value;

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaYCm.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaYCm.value;

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaTimeCs.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].deltaTimeCs.value;
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].headingValuePresent) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].headingValue.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].headingValue.value;
            }
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationLeftCmPresent) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationLeftCm.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationLeftCm.value;
            }
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationRightCmPresent) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationRightCm.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleLateralDeviationRightCm.value;
            }
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationBeforeCsPresent) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationBeforeCs.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationBeforeCs.value;
            }
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationAfterCsPresent) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationAfterCs.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].possibleTimeDeviationAfterCs.value;
            }
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeedPresent) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeed.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].absSpeed.value;
            }
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAccelerationPresent) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAcceleration.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[a].longitudinalAcceleration.value;
            }
        }
        // End Sequence of PlannedTrajectory
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectoryPresent) {

            // Start SEQUENCE OF DesiredTrajectory
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.count =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.count;

            int count_b = ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.count;

            for(int b = 0; b < count_b; b++) {
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].headingValuePresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].headingValuePresent;
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationLeftCmPresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationLeftCmPresent;
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationRightCmPresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationRightCmPresent;
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationBeforeCsPresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationBeforeCsPresent;
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationAfterCsPresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationAfterCsPresent;
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeedPresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeedPresent;
                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAccelerationPresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAccelerationPresent;

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaXCm.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaXCm.value;

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaYCm.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaYCm.value;

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaTimeCs.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].deltaTimeCs.value;
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].headingValuePresent) {

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].headingValue.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].headingValue.value;
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationLeftCmPresent) {

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationLeftCm.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationLeftCm.value;
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationRightCmPresent) {

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationRightCm.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleLateralDeviationRightCm.value;
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationBeforeCsPresent) {

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationBeforeCs.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationBeforeCs.value;
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationAfterCsPresent) {

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationAfterCs.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].possibleTimeDeviationAfterCs.value;
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeedPresent) {

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeed.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].absSpeed.value;
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAccelerationPresent) {

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAcceleration.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.desiredTrajectory.elements[b].longitudinalAcceleration.value;
                }
            }
            // End Sequence of DesiredTrajectory
        }

        // Start SEQUENCE OF RespectedDesiredTrajectoriesList
        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.count =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.count;

        int count_c = ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.count;

        for(int c = 0; c < count_c; c++) {

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.elements[c].value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.respectedDesiredTrajectoriesList.elements[c].value;
        }
        // End Sequence of RespectedDesiredTrajectoriesList
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToCPresent) {

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToC.minute.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToC.minute.value;

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToC.millisecond.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfToC.millisecond.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevelPresent) {

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevel.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevel.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRMPresent) {

            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRM.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.triggerTimeOfMRM.value;
        }

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingValue.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingValue.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingConfidence.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingConfidence.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.speed.speedValue.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.speed.speedValue.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.speed.speedConfidence.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.speed.speedConfidence.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleLength.vehicleLengthValue.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleLength.vehicleLengthValue.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleLength.vehicleLengthConfidenceIndication.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleLength.vehicleLengthConfidenceIndication.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleWidth.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleWidth.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.longitudinalAcceleration.longitudinalAccelerationValue.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.longitudinalAcceleration.longitudinalAccelerationValue.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.longitudinalAcceleration.longitudinalAccelerationConfidence.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.longitudinalAcceleration.longitudinalAccelerationConfidence.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lateralAcceleration.lateralAccelerationValue.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lateralAcceleration.lateralAccelerationValue.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lateralAcceleration.lateralAccelerationConfidence.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lateralAcceleration.lateralAccelerationConfidence.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.verticalAcceleration.verticalAccelerationValue.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.verticalAcceleration.verticalAccelerationValue.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.verticalAcceleration.verticalAccelerationConfidence.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.verticalAcceleration.verticalAccelerationConfidence.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.yawRate.yawRateValue.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.yawRate.yawRateValue.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.yawRate.yawRateConfidence.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.yawRate.yawRateConfidence.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvature.curvatureValue.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvature.curvatureValue.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvature.curvatureConfidence.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvature.curvatureConfidence.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvatureCalculationMode.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.curvatureCalculationMode.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.driveDirection.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.driveDirection.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lanePosition.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lanePosition.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.steeringWheelAngle.steeringWheelAngleValue.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.steeringWheelAngle.steeringWheelAngleValue.value;

        wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.steeringWheelAngle.steeringWheelAngleConfidence.value =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.steeringWheelAngle.steeringWheelAngleConfidence.value;
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseListPresent) {

            // Start SEQUENCE OF AdviceResponseList
            wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.count =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.count;

            int count_d = ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.count;

            for(int d = 0; d < count_d; d++) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements[d].adviceID.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements[d].adviceID.value;
                
                // START BIT STRING AdviceFollowed
                for(int e = 0; e < 2; e++) {
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements[d].adviceFollowed.values[e] = 
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.adviceResponseList.elements[d].adviceFollowed.values[e];
                }
                // END BIT STRING AdviceFollowed
                
            }
            // End Sequence of AdviceResponseList
        }
    }
    else   // CHOICE ManeuverContainer
    {
        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceIDPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceIDPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceIDPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceIDPresent;
        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceListPresent =
            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceListPresent;
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceIDPresent) {
            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.regionPresent =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.regionPresent;
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.regionPresent) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.region.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.region.value;
            }

            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.id.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.intersectionReferenceID.id.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceIDPresent) {
            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.regionPresent =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.regionPresent;
            if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.regionPresent) {

                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.region.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.region.value;
            }

            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.id.value =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.roadSegmentReferenceID.id.value;
        }
        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceListPresent) {

            // Start SEQUENCE OF VehicleAdviceList
            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.count =
                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.count;

            int count_f = ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.count;

            for(int f = 0; f < count_f; f++) {
                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvicePresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvicePresent;
                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvicePresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvicePresent;
                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvicePresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvicePresent;
                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvicePresent =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvicePresent;

                wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].targetStationID.value =
                    ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].targetStationID.value;
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvicePresent) {
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeedPresent =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeedPresent;
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehiclePresent =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehiclePresent;
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehiclePresent =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehiclePresent;
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToCPresent =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToCPresent;

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.adviceID.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.adviceID.value;

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneAdviceReason.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneAdviceReason.value;

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangePosition.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangePosition.value;

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeMoment.minute.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeMoment.minute.value;

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeMoment.millisecond.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeMoment.millisecond.value;
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeedPresent) {

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeed.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.laneChangeSpeed.value;
                    }
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehiclePresent) {

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehicle.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.leadingVehicle.value;
                    }
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehiclePresent) {

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehicle.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.followingVehicle.value;
                    }

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.targetLane.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.targetLane.value;
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToCPresent) {

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToC.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].laneAdvice.triggeringPointOfToC.value;
                    }
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvicePresent) {

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.adviceID.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.adviceID.value;

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.adviceLaneID.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.adviceLaneID.value;

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.advicePosition.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.advicePosition.value;

                    // START CHOICE DesiredBehaviour
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.choice = ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.choice;

                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.choice == 0) {

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.targetGap.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.targetGap.value;
                    }
                    else   // CHOICE DesiredBehaviour
                    {

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.targetSpeed.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].carFollowingAdvice.desiredBehaviour.targetSpeed.value;
                    }
                    // END CHOICE DesiredBehaviour
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvicePresent) {
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransitionPresent =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransitionPresent;
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransitionPresent =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransitionPresent;
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransitionPresent =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransitionPresent;

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.adviceID.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.adviceID.value;

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.tocAdviceReason.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.tocAdviceReason.value;
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransitionPresent) {

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransition.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfStartTransition.value;
                    }
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransitionPresent) {

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransition.minute.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransition.minute.value;

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransition.millisecond.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.timeOfTriggerTransition.millisecond.value;
                    }
                    if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransitionPresent) {

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransition.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].tocAdvice.placeOfEndTransition.value;
                    }
                }
                if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvicePresent) {

                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.adviceID.value =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.adviceID.value;

                    // Start SEQUENCE OF TrajectoryAdvice_trajectory
                    wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.count =
                        ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.count;

                    int count_g = ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.count;

                    for(int g = 0; g < count_g; g++) {
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].headingValuePresent =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].headingValuePresent;
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationLeftCmPresent =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationLeftCmPresent;
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationRightCmPresent =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationRightCmPresent;
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationBeforeCsPresent =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationBeforeCsPresent;
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationAfterCsPresent =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationAfterCsPresent;
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].absSpeedPresent =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].absSpeedPresent;
                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].longitudinalAccelerationPresent =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].longitudinalAccelerationPresent;

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].deltaXCm.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].deltaXCm.value;

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].deltaYCm.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].deltaYCm.value;

                        wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].deltaTimeCs.value =
                            ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].deltaTimeCs.value;
                        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].headingValuePresent) {

                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].headingValue.value =
                                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].headingValue.value;
                        }
                        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationLeftCmPresent) {

                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationLeftCm.value =
                                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationLeftCm.value;
                        }
                        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationRightCmPresent) {

                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationRightCm.value =
                                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleLateralDeviationRightCm.value;
                        }
                        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationBeforeCsPresent) {

                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationBeforeCs.value =
                                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationBeforeCs.value;
                        }
                        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationAfterCsPresent) {

                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationAfterCs.value =
                                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].possibleTimeDeviationAfterCs.value;
                        }
                        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].absSpeedPresent) {

                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].absSpeed.value =
                                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].absSpeed.value;
                        }
                        if(ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].longitudinalAccelerationPresent) {

                            wind->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].longitudinalAcceleration.value =
                                ros->maneuverCoordination.mcmParameters.maneuverContainer.rsuManeuver.vehicleAdviceList.elements[f].trajectoryAdvice.trajectory.elements[g].longitudinalAcceleration.value;
                        }
                    }
                    // End Sequence of TrajectoryAdvice_trajectory
                }
            }
            // End Sequence of VehicleAdviceList
        }
    }
    // END CHOICE ManeuverContainer
}
