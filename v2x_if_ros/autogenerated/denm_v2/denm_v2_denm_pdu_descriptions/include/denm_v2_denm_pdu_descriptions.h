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
#ifndef DENM_V2_DENM_PDU_DESCRIPTIONS_H
#define DENM_V2_DENM_PDU_DESCRIPTIONS_H

#include <stdint.h>
#include <its_container_v2_its_container.h>


/**
 * Repository de.dlr.ts.v2x:denm_v2:1.5
 * 
 * Main struct is DENM
 * 
 */
namespace wind
{
    namespace cpp
    {
        namespace DENM_PDU_Descriptions
        {
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 49 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ReferenceDenms
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 8;
                
                int8_t                  count; // 49
                ITS_Container::ActionID elements[8]; // 6
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 101 bytes.
             *
             */
            #pragma pack(push, 1)
            struct StationaryVehicleContainer
            {
                bool                                  stationarySincePresent; // 101
                bool                                  stationaryCausePresent; // 101
                bool                                  carryingDangerousGoodsPresent; // 101
                bool                                  numberOfOccupantsPresent; // 101
                bool                                  vehicleIdentificationPresent; // 101
                bool                                  energyStorageTypePresent; // 101
                ITS_Container::StationarySince        stationarySince; // 1
                ITS_Container::CauseCode              stationaryCause; // 2
                ITS_Container::DangerousGoodsExtended carryingDangerousGoods; // 73
                ITS_Container::NumberOfOccupants      numberOfOccupants; // 1
                ITS_Container::VehicleIdentification  vehicleIdentification; // 11
                ITS_Container::EnergyStorageType      energyStorageType; // 7
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1427 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RoadWorksContainerExtended
            {
                bool                                  lightBarSirenInUsePresent; // 1427
                bool                                  closedLanesPresent; // 1427
                bool                                  restrictionPresent; // 1427
                bool                                  speedLimitPresent; // 1427
                bool                                  incidentIndicationPresent; // 1427
                bool                                  recommendedPathPresent; // 1427
                bool                                  startingPointSpeedLimitPresent; // 1427
                bool                                  trafficFlowRulePresent; // 1427
                bool                                  referenceDenmsPresent; // 1427
                ITS_Container::LightBarSirenInUse     lightBarSirenInUse; // 2
                ITS_Container::ClosedLanes            closedLanes; // 18
                ITS_Container::RestrictedTypes        restriction; // 4
                ITS_Container::SpeedLimit             speedLimit; // 1
                ITS_Container::CauseCode              incidentIndication; // 2
                ITS_Container::ItineraryPath          recommendedPath; // 1321
                ITS_Container::DeltaReferencePosition startingPointSpeedLimit; // 20
                ITS_Container::TrafficRule            trafficFlowRule; // 1
                ReferenceDenms                        referenceDenms; // 49
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 65 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ImpactReductionContainer
            {
                ITS_Container::HeightLonCarr             heightLonCarrLeft; // 4
                ITS_Container::HeightLonCarr             heightLonCarrRight; // 4
                ITS_Container::PosLonCarr                posLonCarrLeft; // 4
                ITS_Container::PosLonCarr                posLonCarrRight; // 4
                ITS_Container::PositionOfPillars         positionOfPillars; // 13
                ITS_Container::PosCentMass               posCentMass; // 4
                ITS_Container::WheelBaseVehicle          wheelBaseVehicle; // 4
                ITS_Container::TurningRadius             turningRadius; // 1
                ITS_Container::PosFrontAx                posFrontAx; // 4
                ITS_Container::PositionOfOccupants       positionOfOccupants; // 20
                ITS_Container::VehicleMass               vehicleMass; // 2
                ITS_Container::RequestResponseIndication requestResponseIndication; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct Termination
            {
                enum Values {
                    isCancellation = 0,
                    isNegation     = 1
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1602 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AlacarteContainer
            {
                bool                                   lanePositionPresent; // 1602
                bool                                   impactReductionPresent; // 1602
                bool                                   externalTemperaturePresent; // 1602
                bool                                   roadWorksPresent; // 1602
                bool                                   positioningSolutionPresent; // 1602
                bool                                   stationaryVehiclePresent; // 1602
                ITS_Container::LanePosition            lanePosition; // 1
                ImpactReductionContainer               impactReduction; // 65
                ITS_Container::Temperature             externalTemperature; // 1
                RoadWorksContainerExtended             roadWorks; // 1427
                ITS_Container::PositioningSolutionType positioningSolution; // 1
                StationaryVehicleContainer             stationaryVehicle; // 101
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6468 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LocationContainer
            {
                bool                    eventSpeedPresent; // 6468
                bool                    eventPositionHeadingPresent; // 6468
                bool                    roadTypePresent; // 6468
                ITS_Container::Speed    eventSpeed; // 8
                ITS_Container::Heading  eventPositionHeading; // 8
                ITS_Container::Traces   traces; // 6448
                ITS_Container::RoadType roadType; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 560 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SituationContainer
            {
                bool                              linkedCausePresent; // 560
                bool                              eventHistoryPresent; // 560
                ITS_Container::InformationQuality informationQuality; // 1
                ITS_Container::CauseCode          eventType; // 2
                ITS_Container::CauseCode          linkedCause; // 2
                ITS_Container::EventHistory       eventHistory; // 553
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 69 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ManagementContainer
            {
                bool                                     terminationPresent; // 69
                bool                                     relevanceDistancePresent; // 69
                bool                                     relevanceTrafficDirectionPresent; // 69
                bool                                     transmissionIntervalPresent; // 69
                ITS_Container::ActionID                  actionID; // 6
                ITS_Container::TimestampIts              detectionTime; // 8
                ITS_Container::TimestampIts              referenceTime; // 8
                Termination                              termination; // 1
                ITS_Container::ReferencePosition         eventPosition; // 33
                ITS_Container::RelevanceDistance         relevanceDistance; // 1
                ITS_Container::RelevanceTrafficDirection relevanceTrafficDirection; // 1
                ITS_Container::ValidityDuration          validityDuration; // 4
                ITS_Container::TransmissionInterval      transmissionInterval; // 2
                ITS_Container::StationType               stationType; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8702 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DecentralizedEnvironmentalNotificationMessage
            {
                bool                situationPresent; // 8702
                bool                locationPresent; // 8702
                bool                alacartePresent; // 8702
                ManagementContainer management; // 69
                SituationContainer  situation; // 560
                LocationContainer   location; // 6468
                AlacarteContainer   alacarte; // 1602
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8716 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DENM
            {
                uint16_t                                      endiannessCheck = 1;
                uint16_t                                      wind_reserved_0 = 0;
                uint16_t                                      wind_reserved_1 = 0;
                uint16_t                                      wind_reserved_2 = 0;
            
                ITS_Container::ItsPduHeader                   header; // 6
                DecentralizedEnvironmentalNotificationMessage denm; // 8702
            };
            #pragma pack(pop)

        }  // Closing namespace DENM_PDU_Descriptions
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //DENM_V2_DENM_PDU_DESCRIPTIONS_H
