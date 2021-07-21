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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:denm_v2:1.5
 * 
 * Module: DENM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) denm(1) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <denm_v2_denm_pdu_descriptions_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const denm_v2_denm_pdu_descriptions::DENM::ConstPtr& ros, wind::cpp::DENM_PDU_Descriptions::DENM* wind)
{

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;
    wind->denm.situationPresent =
        ros->denm.situationPresent;
    wind->denm.locationPresent =
        ros->denm.locationPresent;
    wind->denm.alacartePresent =
        ros->denm.alacartePresent;
    wind->denm.management.terminationPresent =
        ros->denm.management.terminationPresent;
    wind->denm.management.relevanceDistancePresent =
        ros->denm.management.relevanceDistancePresent;
    wind->denm.management.relevanceTrafficDirectionPresent =
        ros->denm.management.relevanceTrafficDirectionPresent;
    wind->denm.management.transmissionIntervalPresent =
        ros->denm.management.transmissionIntervalPresent;

    wind->denm.management.actionID.originatingStationID.value =
        ros->denm.management.actionID.originatingStationID.value;

    wind->denm.management.actionID.sequenceNumber.value =
        ros->denm.management.actionID.sequenceNumber.value;

    wind->denm.management.detectionTime.value =
        ros->denm.management.detectionTime.value;

    wind->denm.management.referenceTime.value =
        ros->denm.management.referenceTime.value;
    if(ros->denm.management.terminationPresent) {

        wind->denm.management.termination.value =
            ros->denm.management.termination.value;
    }

    wind->denm.management.eventPosition.latitude.value =
        ros->denm.management.eventPosition.latitude.value;

    wind->denm.management.eventPosition.longitude.value =
        ros->denm.management.eventPosition.longitude.value;

    wind->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence.value =
        ros->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence.value;

    wind->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence.value =
        ros->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence.value;

    wind->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation.value =
        ros->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation.value;

    wind->denm.management.eventPosition.altitude.altitudeValue.value =
        ros->denm.management.eventPosition.altitude.altitudeValue.value;

    wind->denm.management.eventPosition.altitude.altitudeConfidence.value =
        ros->denm.management.eventPosition.altitude.altitudeConfidence.value;
    if(ros->denm.management.relevanceDistancePresent) {

        wind->denm.management.relevanceDistance.value =
            ros->denm.management.relevanceDistance.value;
    }
    if(ros->denm.management.relevanceTrafficDirectionPresent) {

        wind->denm.management.relevanceTrafficDirection.value =
            ros->denm.management.relevanceTrafficDirection.value;
    }

    wind->denm.management.validityDuration.value =
        ros->denm.management.validityDuration.value;
    if(ros->denm.management.transmissionIntervalPresent) {

        wind->denm.management.transmissionInterval.value =
            ros->denm.management.transmissionInterval.value;
    }

    wind->denm.management.stationType.value =
        ros->denm.management.stationType.value;
    if(ros->denm.situationPresent) {
        wind->denm.situation.linkedCausePresent =
            ros->denm.situation.linkedCausePresent;
        wind->denm.situation.eventHistoryPresent =
            ros->denm.situation.eventHistoryPresent;

        wind->denm.situation.informationQuality.value =
            ros->denm.situation.informationQuality.value;

        wind->denm.situation.eventType.causeCode.value =
            ros->denm.situation.eventType.causeCode.value;

        wind->denm.situation.eventType.subCauseCode.value =
            ros->denm.situation.eventType.subCauseCode.value;
        if(ros->denm.situation.linkedCausePresent) {

            wind->denm.situation.linkedCause.causeCode.value =
                ros->denm.situation.linkedCause.causeCode.value;

            wind->denm.situation.linkedCause.subCauseCode.value =
                ros->denm.situation.linkedCause.subCauseCode.value;
        }
        if(ros->denm.situation.eventHistoryPresent) {

            // Start SEQUENCE OF EventHistory
            wind->denm.situation.eventHistory.count =
                ros->denm.situation.eventHistory.count;

            int count_a = ros->denm.situation.eventHistory.count;

            for(int a = 0; a < count_a; a++) {
                wind->denm.situation.eventHistory.elements[a].eventDeltaTimePresent =
                    ros->denm.situation.eventHistory.elements[a].eventDeltaTimePresent;

                wind->denm.situation.eventHistory.elements[a].eventPosition.deltaLatitude.value =
                    ros->denm.situation.eventHistory.elements[a].eventPosition.deltaLatitude.value;

                wind->denm.situation.eventHistory.elements[a].eventPosition.deltaLongitude.value =
                    ros->denm.situation.eventHistory.elements[a].eventPosition.deltaLongitude.value;

                wind->denm.situation.eventHistory.elements[a].eventPosition.deltaAltitude.value =
                    ros->denm.situation.eventHistory.elements[a].eventPosition.deltaAltitude.value;
                if(ros->denm.situation.eventHistory.elements[a].eventDeltaTimePresent) {

                    wind->denm.situation.eventHistory.elements[a].eventDeltaTime.value =
                        ros->denm.situation.eventHistory.elements[a].eventDeltaTime.value;
                }

                wind->denm.situation.eventHistory.elements[a].informationQuality.value =
                    ros->denm.situation.eventHistory.elements[a].informationQuality.value;
            }
            // End Sequence of EventHistory
        }
    }
    if(ros->denm.locationPresent) {
        wind->denm.location.eventSpeedPresent =
            ros->denm.location.eventSpeedPresent;
        wind->denm.location.eventPositionHeadingPresent =
            ros->denm.location.eventPositionHeadingPresent;
        wind->denm.location.roadTypePresent =
            ros->denm.location.roadTypePresent;
        if(ros->denm.location.eventSpeedPresent) {

            wind->denm.location.eventSpeed.speedValue.value =
                ros->denm.location.eventSpeed.speedValue.value;

            wind->denm.location.eventSpeed.speedConfidence.value =
                ros->denm.location.eventSpeed.speedConfidence.value;
        }
        if(ros->denm.location.eventPositionHeadingPresent) {

            wind->denm.location.eventPositionHeading.headingValue.value =
                ros->denm.location.eventPositionHeading.headingValue.value;

            wind->denm.location.eventPositionHeading.headingConfidence.value =
                ros->denm.location.eventPositionHeading.headingConfidence.value;
        }

        // Start SEQUENCE OF Traces
        wind->denm.location.traces.count =
            ros->denm.location.traces.count;

        int count_b = ros->denm.location.traces.count;

        for(int b = 0; b < count_b; b++) {

            // Start SEQUENCE OF PathHistory
            wind->denm.location.traces.elements[b].count =
                ros->denm.location.traces.elements[b].count;

            int count_c = ros->denm.location.traces.elements[b].count;

            for(int c = 0; c < count_c; c++) {
                wind->denm.location.traces.elements[b].elements[c].pathDeltaTimePresent =
                    ros->denm.location.traces.elements[b].elements[c].pathDeltaTimePresent;

                wind->denm.location.traces.elements[b].elements[c].pathPosition.deltaLatitude.value =
                    ros->denm.location.traces.elements[b].elements[c].pathPosition.deltaLatitude.value;

                wind->denm.location.traces.elements[b].elements[c].pathPosition.deltaLongitude.value =
                    ros->denm.location.traces.elements[b].elements[c].pathPosition.deltaLongitude.value;

                wind->denm.location.traces.elements[b].elements[c].pathPosition.deltaAltitude.value =
                    ros->denm.location.traces.elements[b].elements[c].pathPosition.deltaAltitude.value;
                if(ros->denm.location.traces.elements[b].elements[c].pathDeltaTimePresent) {

                    wind->denm.location.traces.elements[b].elements[c].pathDeltaTime.value =
                        ros->denm.location.traces.elements[b].elements[c].pathDeltaTime.value;
                }
            }
            // End Sequence of PathHistory
        }
        // End Sequence of Traces
        if(ros->denm.location.roadTypePresent) {

            wind->denm.location.roadType.value =
                ros->denm.location.roadType.value;
        }
    }
    if(ros->denm.alacartePresent) {
        wind->denm.alacarte.lanePositionPresent =
            ros->denm.alacarte.lanePositionPresent;
        wind->denm.alacarte.impactReductionPresent =
            ros->denm.alacarte.impactReductionPresent;
        wind->denm.alacarte.externalTemperaturePresent =
            ros->denm.alacarte.externalTemperaturePresent;
        wind->denm.alacarte.roadWorksPresent =
            ros->denm.alacarte.roadWorksPresent;
        wind->denm.alacarte.positioningSolutionPresent =
            ros->denm.alacarte.positioningSolutionPresent;
        wind->denm.alacarte.stationaryVehiclePresent =
            ros->denm.alacarte.stationaryVehiclePresent;
        if(ros->denm.alacarte.lanePositionPresent) {

            wind->denm.alacarte.lanePosition.value =
                ros->denm.alacarte.lanePosition.value;
        }
        if(ros->denm.alacarte.impactReductionPresent) {

            wind->denm.alacarte.impactReduction.heightLonCarrLeft.value =
                ros->denm.alacarte.impactReduction.heightLonCarrLeft.value;

            wind->denm.alacarte.impactReduction.heightLonCarrRight.value =
                ros->denm.alacarte.impactReduction.heightLonCarrRight.value;

            wind->denm.alacarte.impactReduction.posLonCarrLeft.value =
                ros->denm.alacarte.impactReduction.posLonCarrLeft.value;

            wind->denm.alacarte.impactReduction.posLonCarrRight.value =
                ros->denm.alacarte.impactReduction.posLonCarrRight.value;

            // Start SEQUENCE OF PositionOfPillars
            wind->denm.alacarte.impactReduction.positionOfPillars.count =
                ros->denm.alacarte.impactReduction.positionOfPillars.count;

            int count_d = ros->denm.alacarte.impactReduction.positionOfPillars.count;

            for(int d = 0; d < count_d; d++) {

                wind->denm.alacarte.impactReduction.positionOfPillars.elements[d].value =
                    ros->denm.alacarte.impactReduction.positionOfPillars.elements[d].value;
            }
            // End Sequence of PositionOfPillars

            wind->denm.alacarte.impactReduction.posCentMass.value =
                ros->denm.alacarte.impactReduction.posCentMass.value;

            wind->denm.alacarte.impactReduction.wheelBaseVehicle.value =
                ros->denm.alacarte.impactReduction.wheelBaseVehicle.value;

            wind->denm.alacarte.impactReduction.turningRadius.value =
                ros->denm.alacarte.impactReduction.turningRadius.value;

            wind->denm.alacarte.impactReduction.posFrontAx.value =
                ros->denm.alacarte.impactReduction.posFrontAx.value;
            
            // START BIT STRING PositionOfOccupants
            for(int e = 0; e < 20; e++) {
                wind->denm.alacarte.impactReduction.positionOfOccupants.values[e] = 
                    ros->denm.alacarte.impactReduction.positionOfOccupants.values[e];
            }
            // END BIT STRING PositionOfOccupants
            

            wind->denm.alacarte.impactReduction.vehicleMass.value =
                ros->denm.alacarte.impactReduction.vehicleMass.value;

            wind->denm.alacarte.impactReduction.requestResponseIndication.value =
                ros->denm.alacarte.impactReduction.requestResponseIndication.value;
        }
        if(ros->denm.alacarte.externalTemperaturePresent) {

            wind->denm.alacarte.externalTemperature.value =
                ros->denm.alacarte.externalTemperature.value;
        }
        if(ros->denm.alacarte.roadWorksPresent) {
            wind->denm.alacarte.roadWorks.lightBarSirenInUsePresent =
                ros->denm.alacarte.roadWorks.lightBarSirenInUsePresent;
            wind->denm.alacarte.roadWorks.closedLanesPresent =
                ros->denm.alacarte.roadWorks.closedLanesPresent;
            wind->denm.alacarte.roadWorks.restrictionPresent =
                ros->denm.alacarte.roadWorks.restrictionPresent;
            wind->denm.alacarte.roadWorks.speedLimitPresent =
                ros->denm.alacarte.roadWorks.speedLimitPresent;
            wind->denm.alacarte.roadWorks.incidentIndicationPresent =
                ros->denm.alacarte.roadWorks.incidentIndicationPresent;
            wind->denm.alacarte.roadWorks.recommendedPathPresent =
                ros->denm.alacarte.roadWorks.recommendedPathPresent;
            wind->denm.alacarte.roadWorks.startingPointSpeedLimitPresent =
                ros->denm.alacarte.roadWorks.startingPointSpeedLimitPresent;
            wind->denm.alacarte.roadWorks.trafficFlowRulePresent =
                ros->denm.alacarte.roadWorks.trafficFlowRulePresent;
            wind->denm.alacarte.roadWorks.referenceDenmsPresent =
                ros->denm.alacarte.roadWorks.referenceDenmsPresent;
            if(ros->denm.alacarte.roadWorks.lightBarSirenInUsePresent) {
                
                // START BIT STRING LightBarSirenInUse
                for(int f = 0; f < 2; f++) {
                    wind->denm.alacarte.roadWorks.lightBarSirenInUse.values[f] = 
                        ros->denm.alacarte.roadWorks.lightBarSirenInUse.values[f];
                }
                // END BIT STRING LightBarSirenInUse
                
            }
            if(ros->denm.alacarte.roadWorks.closedLanesPresent) {
                wind->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatusPresent =
                    ros->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatusPresent;
                wind->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatusPresent =
                    ros->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatusPresent;
                wind->denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent =
                    ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent;
                if(ros->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatusPresent) {

                    wind->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatus.value =
                        ros->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatus.value;
                }
                if(ros->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatusPresent) {

                    wind->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatus.value =
                        ros->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatus.value;
                }
                if(ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent) {
                    
                    // START BIT STRING DrivingLaneStatus
                    for(int g = 0; g < 13; g++) {
                        wind->denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values[g] = 
                            ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values[g];
                    }
                    // END BIT STRING DrivingLaneStatus
                    
                }
            }
            if(ros->denm.alacarte.roadWorks.restrictionPresent) {

                // Start SEQUENCE OF RestrictedTypes
                wind->denm.alacarte.roadWorks.restriction.count =
                    ros->denm.alacarte.roadWorks.restriction.count;

                int count_h = ros->denm.alacarte.roadWorks.restriction.count;

                for(int h = 0; h < count_h; h++) {

                    wind->denm.alacarte.roadWorks.restriction.elements[h].value =
                        ros->denm.alacarte.roadWorks.restriction.elements[h].value;
                }
                // End Sequence of RestrictedTypes
            }
            if(ros->denm.alacarte.roadWorks.speedLimitPresent) {

                wind->denm.alacarte.roadWorks.speedLimit.value =
                    ros->denm.alacarte.roadWorks.speedLimit.value;
            }
            if(ros->denm.alacarte.roadWorks.incidentIndicationPresent) {

                wind->denm.alacarte.roadWorks.incidentIndication.causeCode.value =
                    ros->denm.alacarte.roadWorks.incidentIndication.causeCode.value;

                wind->denm.alacarte.roadWorks.incidentIndication.subCauseCode.value =
                    ros->denm.alacarte.roadWorks.incidentIndication.subCauseCode.value;
            }
            if(ros->denm.alacarte.roadWorks.recommendedPathPresent) {

                // Start SEQUENCE OF ItineraryPath
                wind->denm.alacarte.roadWorks.recommendedPath.count =
                    ros->denm.alacarte.roadWorks.recommendedPath.count;

                int count_i = ros->denm.alacarte.roadWorks.recommendedPath.count;

                for(int i = 0; i < count_i; i++) {

                    wind->denm.alacarte.roadWorks.recommendedPath.elements[i].latitude.value =
                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].latitude.value;

                    wind->denm.alacarte.roadWorks.recommendedPath.elements[i].longitude.value =
                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].longitude.value;

                    wind->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMajorConfidence.value =
                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMajorConfidence.value;

                    wind->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMinorConfidence.value =
                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMinorConfidence.value;

                    wind->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMajorOrientation.value =
                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMajorOrientation.value;

                    wind->denm.alacarte.roadWorks.recommendedPath.elements[i].altitude.altitudeValue.value =
                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].altitude.altitudeValue.value;

                    wind->denm.alacarte.roadWorks.recommendedPath.elements[i].altitude.altitudeConfidence.value =
                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].altitude.altitudeConfidence.value;
                }
                // End Sequence of ItineraryPath
            }
            if(ros->denm.alacarte.roadWorks.startingPointSpeedLimitPresent) {

                wind->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaLatitude.value =
                    ros->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaLatitude.value;

                wind->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaLongitude.value =
                    ros->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaLongitude.value;

                wind->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaAltitude.value =
                    ros->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaAltitude.value;
            }
            if(ros->denm.alacarte.roadWorks.trafficFlowRulePresent) {

                wind->denm.alacarte.roadWorks.trafficFlowRule.value =
                    ros->denm.alacarte.roadWorks.trafficFlowRule.value;
            }
            if(ros->denm.alacarte.roadWorks.referenceDenmsPresent) {

                // Start SEQUENCE OF ReferenceDenms
                wind->denm.alacarte.roadWorks.referenceDenms.count =
                    ros->denm.alacarte.roadWorks.referenceDenms.count;

                int count_j = ros->denm.alacarte.roadWorks.referenceDenms.count;

                for(int j = 0; j < count_j; j++) {

                    wind->denm.alacarte.roadWorks.referenceDenms.elements[j].originatingStationID.value =
                        ros->denm.alacarte.roadWorks.referenceDenms.elements[j].originatingStationID.value;

                    wind->denm.alacarte.roadWorks.referenceDenms.elements[j].sequenceNumber.value =
                        ros->denm.alacarte.roadWorks.referenceDenms.elements[j].sequenceNumber.value;
                }
                // End Sequence of ReferenceDenms
            }
        }
        if(ros->denm.alacarte.positioningSolutionPresent) {

            wind->denm.alacarte.positioningSolution.value =
                ros->denm.alacarte.positioningSolution.value;
        }
        if(ros->denm.alacarte.stationaryVehiclePresent) {
            wind->denm.alacarte.stationaryVehicle.stationarySincePresent =
                ros->denm.alacarte.stationaryVehicle.stationarySincePresent;
            wind->denm.alacarte.stationaryVehicle.stationaryCausePresent =
                ros->denm.alacarte.stationaryVehicle.stationaryCausePresent;
            wind->denm.alacarte.stationaryVehicle.carryingDangerousGoodsPresent =
                ros->denm.alacarte.stationaryVehicle.carryingDangerousGoodsPresent;
            wind->denm.alacarte.stationaryVehicle.numberOfOccupantsPresent =
                ros->denm.alacarte.stationaryVehicle.numberOfOccupantsPresent;
            wind->denm.alacarte.stationaryVehicle.vehicleIdentificationPresent =
                ros->denm.alacarte.stationaryVehicle.vehicleIdentificationPresent;
            wind->denm.alacarte.stationaryVehicle.energyStorageTypePresent =
                ros->denm.alacarte.stationaryVehicle.energyStorageTypePresent;
            if(ros->denm.alacarte.stationaryVehicle.stationarySincePresent) {

                wind->denm.alacarte.stationaryVehicle.stationarySince.value =
                    ros->denm.alacarte.stationaryVehicle.stationarySince.value;
            }
            if(ros->denm.alacarte.stationaryVehicle.stationaryCausePresent) {

                wind->denm.alacarte.stationaryVehicle.stationaryCause.causeCode.value =
                    ros->denm.alacarte.stationaryVehicle.stationaryCause.causeCode.value;

                wind->denm.alacarte.stationaryVehicle.stationaryCause.subCauseCode.value =
                    ros->denm.alacarte.stationaryVehicle.stationaryCause.subCauseCode.value;
            }
            if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoodsPresent) {
                wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCodePresent =
                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCodePresent;
                wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumberPresent =
                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumberPresent;
                wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyNamePresent =
                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyNamePresent;

                wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.dangerousGoodsType.value =
                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.dangerousGoodsType.value;

                wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.unNumber.value =
                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.unNumber.value;

                wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.elevatedTemperature.value =
                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.elevatedTemperature.value;

                wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.tunnelsRestricted.value =
                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.tunnelsRestricted.value;

                wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.limitedQuantity.value =
                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.limitedQuantity.value;
                if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCodePresent) {


                    for(int k = 0; k < 24; k++) {  // DangerousGoodsExtended_emergencyActionCode
                        if(k < ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCode.value.length())
                            wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCode.value[k] = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCode.value.c_str()[k];
                        else
                            wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCode.value[k] = ' ';
                    }
                }
                if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumberPresent) {


                    for(int l = 0; l < 16; l++) {  // PhoneNumber
                        if(l < ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumber.value.length())
                            wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumber.value[l] = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumber.value.c_str()[l];
                        else
                            wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumber.value[l] = ' ';
                    }
                }
                if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyNamePresent) {


                    for(int m = 0; m < 24; m++) {  // DangerousGoodsExtended_companyName
                        if(m < ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyName.value.length())
                            wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyName.value[m] = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyName.value.c_str()[m];
                        else
                            wind->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyName.value[m] = ' ';
                    }
                }
            }
            if(ros->denm.alacarte.stationaryVehicle.numberOfOccupantsPresent) {

                wind->denm.alacarte.stationaryVehicle.numberOfOccupants.value =
                    ros->denm.alacarte.stationaryVehicle.numberOfOccupants.value;
            }
            if(ros->denm.alacarte.stationaryVehicle.vehicleIdentificationPresent) {
                wind->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumberPresent =
                    ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumberPresent;
                wind->denm.alacarte.stationaryVehicle.vehicleIdentification.vDSPresent =
                    ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDSPresent;
                if(ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumberPresent) {


                    for(int n = 0; n < 3; n++) {  // WMInumber
                        if(n < ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumber.value.length())
                            wind->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumber.value[n] = ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumber.value.c_str()[n];
                        else
                            wind->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumber.value[n] = ' ';
                    }
                }
                if(ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDSPresent) {


                    for(int o = 0; o < 6; o++) {  // VDS
                        if(o < ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDS.value.length())
                            wind->denm.alacarte.stationaryVehicle.vehicleIdentification.vDS.value[o] = ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDS.value.c_str()[o];
                        else
                            wind->denm.alacarte.stationaryVehicle.vehicleIdentification.vDS.value[o] = ' ';
                    }
                }
            }
            if(ros->denm.alacarte.stationaryVehicle.energyStorageTypePresent) {
                
                // START BIT STRING EnergyStorageType
                for(int p = 0; p < 7; p++) {
                    wind->denm.alacarte.stationaryVehicle.energyStorageType.values[p] = 
                        ros->denm.alacarte.stationaryVehicle.energyStorageType.values[p];
                }
                // END BIT STRING EnergyStorageType
                
            }
        }
    }
}
