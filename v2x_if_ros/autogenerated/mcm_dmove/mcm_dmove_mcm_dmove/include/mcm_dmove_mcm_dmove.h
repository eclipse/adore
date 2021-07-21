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
 * Module: DENM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) denm(1) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */
#ifndef MCM_DMOVE_MCM_DMOVE_H
#define MCM_DMOVE_MCM_DMOVE_H

#include <stdint.h>
#include <its_container_v2_its_container.h>


/**
 * Repository de.dlr.ts.v2x:mcm_dmove:3.0
 * 
 * Main struct is MCM
 * 
 */
namespace wind
{
    namespace cpp
    {
        namespace MCM_DMove
        {
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimeDeviation
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 4095;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LateralDeviation
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 511;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DiffTime
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 4095;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DiffPosition
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 10000;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 33 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TrajectoryPoint
            {
                bool                                         headingValuePresent; // 33
                bool                                         possibleLateralDeviationLeftCmPresent; // 33
                bool                                         possibleLateralDeviationRightCmPresent; // 33
                bool                                         possibleTimeDeviationBeforeCsPresent; // 33
                bool                                         possibleTimeDeviationAfterCsPresent; // 33
                bool                                         absSpeedPresent; // 33
                bool                                         longitudinalAccelerationPresent; // 33
                DiffPosition                                 deltaXCm; // 2
                DiffPosition                                 deltaYCm; // 2
                DiffTime                                     deltaTimeCs; // 2
                ITS_Container::HeadingValue                  headingValue; // 4
                LateralDeviation                             possibleLateralDeviationLeftCm; // 2
                LateralDeviation                             possibleLateralDeviationRightCm; // 2
                TimeDeviation                                possibleTimeDeviationBeforeCs; // 2
                TimeDeviation                                possibleTimeDeviationAfterCs; // 2
                ITS_Container::SpeedValue                    absSpeed; // 4
                ITS_Container::LongitudinalAccelerationValue longitudinalAcceleration; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Millisecond
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Minute
            {
                static constexpr int32_t MIN        = 0;
                static constexpr int32_t MAX        = 527040;
                
                int32_t   value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct TargetSpeed
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct TargetGap
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 991 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TrajectoryAdvice_trajectory
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 30;
                
                int8_t          count; // 991
                TrajectoryPoint elements[30]; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct AdviceID
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PlaceOfEndTransition
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 10000;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimeOfTriggerTransition
            {
                Minute      minute; // 4
                Millisecond millisecond; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PlaceOfStartTransition
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 10000;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct TocAdviceReason
            {
                enum Values {
                    reason0    = 0,
                    reason1    = 1
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DesiredBehaviour
            {
                enum Choice {
                    C_targetGap   = 0,
                    C_targetSpeed = 1
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    TargetGap   targetGap; // 1
                    TargetSpeed targetSpeed; // 1
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AdvicePosition
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 10000;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct AdviceLaneID
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TriggeringPointOfToC
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 10000;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct TargetLane
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct FollowingVehicle
            {
                static constexpr uint32_t MIN        = 0;
                static constexpr uint32_t MAX        = 4294967295;
                
                uint32_t  value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LeadingVehicle
            {
                static constexpr uint32_t MIN        = 0;
                static constexpr uint32_t MAX        = 4294967295;
                
                uint32_t  value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneChangeSpeed
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 500;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneChangeMoment
            {
                Minute      minute; // 4
                Millisecond millisecond; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneChangePosition
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 10000;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct LaneAdviceReason
            {
                enum Values {
                    reason0    = 0,
                    reason1    = 1
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 992 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TrajectoryAdvice
            {
                AdviceID                    adviceID; // 1
                TrajectoryAdvice_trajectory trajectory; // 991
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 15 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TocAdvice
            {
                bool                    placeOfStartTransitionPresent; // 15
                bool                    timeOfTriggerTransitionPresent; // 15
                bool                    placeOfEndTransitionPresent; // 15
                AdviceID                adviceID; // 1
                TocAdviceReason         tocAdviceReason; // 1
                PlaceOfStartTransition  placeOfStartTransition; // 2
                TimeOfTriggerTransition timeOfTriggerTransition; // 6
                PlaceOfEndTransition    placeOfEndTransition; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CarFollowingAdvice
            {
                AdviceID         adviceID; // 1
                AdviceLaneID     adviceLaneID; // 1
                AdvicePosition   advicePosition; // 2
                DesiredBehaviour desiredBehaviour; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 27 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAdvice
            {
                bool                 laneChangeSpeedPresent; // 27
                bool                 leadingVehiclePresent; // 27
                bool                 followingVehiclePresent; // 27
                bool                 triggeringPointOfToCPresent; // 27
                AdviceID             adviceID; // 1
                LaneAdviceReason     laneAdviceReason; // 1
                LaneChangePosition   laneChangePosition; // 2
                LaneChangeMoment     laneChangeMoment; // 6
                LaneChangeSpeed      laneChangeSpeed; // 2
                LeadingVehicle       leadingVehicle; // 4
                FollowingVehicle     followingVehicle; // 4
                TargetLane           targetLane; // 1
                TriggeringPointOfToC triggeringPointOfToC; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TargetStationID
            {
                static constexpr uint32_t MIN        = 0;
                static constexpr uint32_t MAX        = 4294967295;
                
                uint32_t  value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AdviceFollowed
            {
                enum Options {
                    notFollowed = 0,
                    followed    = 1
                };
                bool      values[2]; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1048 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleAdvice
            {
                bool               laneAdvicePresent; // 1048
                bool               carFollowingAdvicePresent; // 1048
                bool               tocAdvicePresent; // 1048
                bool               trajectoryAdvicePresent; // 1048
                TargetStationID    targetStationID; // 4
                LaneAdvice         laneAdvice; // 27
                CarFollowingAdvice carFollowingAdvice; // 6
                TocAdvice          tocAdvice; // 15
                TrajectoryAdvice   trajectoryAdvice; // 992
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RoadSegmentID
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RoadRegulatorID
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IntersectionID
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AdviceResponse
            {
                AdviceID       adviceID; // 1
                AdviceFollowed adviceFollowed; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RespectedDesiredTrajectory
            {
                static constexpr uint32_t MIN        = 0;
                static constexpr uint32_t MAX        = 4294967295;
                
                uint32_t  value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ToleratedDistance
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 10000;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 8385 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleAdviceList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 8;
                
                int8_t        count; // 8385
                VehicleAdvice elements[8]; // 1048
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RoadSegmentReferenceID
            {
                bool            regionPresent; // 5
                RoadRegulatorID region; // 2
                RoadSegmentID   id; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IntersectionReferenceID
            {
                bool            regionPresent; // 5
                RoadRegulatorID region; // 2
                IntersectionID  id; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 10 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AdviceResponseList
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 3;
                
                int8_t         count; // 10
                AdviceResponse elements[3]; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TriggerTimeOfMRM
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct TargetAutomationLevel
            {
                enum Values {
                    saeLevel0  = 0,
                    saeLevel1  = 1,
                    saeLevel2  = 2,
                    saeLevel3  = 3,
                    saeLevel4  = 4,
                    saeLevel5  = 5
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TriggerTimeOfToC
            {
                Minute      minute; // 4
                Millisecond millisecond; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 21 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RespectedDesiredTrajectoriesList
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 5;
                
                int8_t                     count; // 21
                RespectedDesiredTrajectory elements[5]; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 991 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DesiredTrajectory
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 30;
                
                int8_t          count; // 991
                TrajectoryPoint elements[30]; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 991 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PlannedTrajectory
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 30;
                
                int8_t          count; // 991
                TrajectoryPoint elements[30]; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 29 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleCapabilities
            {
                bool                                         toleratedMinSpeedPresent; // 29
                bool                                         toleratedMaxSpeedPresent; // 29
                bool                                         toleratedMinLongitudinalAccelerationPresent; // 29
                bool                                         toleratedMaxLongitudinalAccelerationPresent; // 29
                bool                                         toleratedMaxLateralAccelerationPresent; // 29
                ToleratedDistance                            toleratedDistanceAheadCmps; // 2
                ToleratedDistance                            toleratedDistanceBehindCmps; // 2
                ITS_Container::SpeedValue                    toleratedMinSpeed; // 4
                ITS_Container::SpeedValue                    toleratedMaxSpeed; // 4
                ITS_Container::LongitudinalAccelerationValue toleratedMinLongitudinalAcceleration; // 4
                ITS_Container::LongitudinalAccelerationValue toleratedMaxLongitudinalAcceleration; // 4
                ITS_Container::LateralAccelerationValue      toleratedMaxLateralAcceleration; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8398 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RsuManeuver
            {
                bool                    intersectionReferenceIDPresent; // 8398
                bool                    roadSegmentReferenceIDPresent; // 8398
                bool                    vehicleAdviceListPresent; // 8398
                IntersectionReferenceID intersectionReferenceID; // 5
                RoadSegmentReferenceID  roadSegmentReferenceID; // 5
                VehicleAdviceList       vehicleAdviceList; // 8385
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2119 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleManeuver
            {
                bool                                    desiredTrajectoryPresent; // 2119
                bool                                    triggerTimeOfToCPresent; // 2119
                bool                                    targetAutomationLevelPresent; // 2119
                bool                                    triggerTimeOfMRMPresent; // 2119
                bool                                    adviceResponseListPresent; // 2119
                VehicleCapabilities                     vehicleCapabilities; // 29
                PlannedTrajectory                       plannedTrajectory; // 991
                DesiredTrajectory                       desiredTrajectory; // 991
                RespectedDesiredTrajectoriesList        respectedDesiredTrajectoriesList; // 21
                TriggerTimeOfToC                        triggerTimeOfToC; // 6
                TargetAutomationLevel                   targetAutomationLevel; // 1
                TriggerTimeOfMRM                        triggerTimeOfMRM; // 2
                ITS_Container::Heading                  heading; // 8
                ITS_Container::Speed                    speed; // 8
                ITS_Container::VehicleLength            vehicleLength; // 5
                ITS_Container::VehicleWidth             vehicleWidth; // 4
                ITS_Container::LongitudinalAcceleration longitudinalAcceleration; // 8
                ITS_Container::LateralAcceleration      lateralAcceleration; // 8
                ITS_Container::VerticalAcceleration     verticalAcceleration; // 8
                ITS_Container::YawRate                  yawRate; // 5
                ITS_Container::Curvature                curvature; // 3
                ITS_Container::CurvatureCalculationMode curvatureCalculationMode; // 1
                ITS_Container::DriveDirection           driveDirection; // 1
                ITS_Container::LanePosition             lanePosition; // 1
                ITS_Container::SteeringWheelAngle       steeringWheelAngle; // 3
                AdviceResponseList                      adviceResponseList; // 10
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 8399 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ManeuverContainer
            {
                enum Choice {
                    C_vehicleManeuver = 0,
                    C_rsuManeuver     = 1
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    VehicleManeuver vehicleManeuver; // 2119
                    RsuManeuver     rsuManeuver; // 8398
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 34 bytes.
             *
             */
            #pragma pack(push, 1)
            struct BasicContainer
            {
                ITS_Container::StationType       stationType; // 1
                ITS_Container::ReferencePosition referencePosition; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8433 bytes.
             *
             */
            #pragma pack(push, 1)
            struct McmParameters
            {
                BasicContainer    basicContainer; // 34
                ManeuverContainer maneuverContainer; // 8399
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct GenerationDeltaTime
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8435 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ManeuverCoordination
            {
                GenerationDeltaTime generationDeltaTime; // 2
                McmParameters       mcmParameters; // 8433
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RequestID
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8449 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MCM
            {
                uint16_t                    endiannessCheck = 1;
                uint16_t                    wind_reserved_0 = 0;
                uint16_t                    wind_reserved_1 = 0;
                uint16_t                    wind_reserved_2 = 0;
            
                ITS_Container::ItsPduHeader header; // 6
                ManeuverCoordination        maneuverCoordination; // 8435
            };
            #pragma pack(pop)

        }  // Closing namespace MCM_DMove
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //MCM_DMOVE_MCM_DMOVE_H
