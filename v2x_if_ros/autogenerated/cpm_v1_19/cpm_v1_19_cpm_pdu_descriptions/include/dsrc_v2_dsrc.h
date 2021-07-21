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
 * Module: CAM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) cam(2) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */
#ifndef DSRC_V2_DSRC_H
#define DSRC_V2_DSRC_H

#include <stdint.h>
#include <dsrc_v2_electronicregistrationidentificationvehicledatamodule.h>
#include <its_container_v2_its_container.h>


namespace wind
{
    namespace cpp
    {
        namespace DSRC
        {
            /*
             *
             *  a range of +- 327.68 meters 
             * 
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Offset_B16
            {
                static constexpr float MIN          = -327.68;
                static constexpr float MAX          = 327.67;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             *  a range of +- 81.91 meters 
             * 
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Offset_B14
            {
                static constexpr float MIN          = -81.92;
                static constexpr float MAX          = 81.91;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             *  a range of +- 40.95 meters 
             * 
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Offset_B13
            {
                static constexpr float MIN          = -40.96;
                static constexpr float MAX          = 40.95;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             *  a range of +- 20.47 meters 
             * 
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Offset_B12
            {
                static constexpr float MIN          = -20.48;
                static constexpr float MAX          = 20.47;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             *  a range of +- 10.23 meters 
             * 
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Offset_B11
            {
                static constexpr float MIN          = -10.24;
                static constexpr float MAX          = 10.23;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             *  a range of +- 5.11 meters 
             * 
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Offset_B10
            {
                static constexpr float MIN          = -5.12;
                static constexpr float MAX          = 5.11;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RestrictionClassID
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             *  Unit = 1 meter, 0 = unknown, 
             *  The value 10000 to be used for Distances >=10000 m 
             *  (e.g. from known point to another point along a  
             *  known path, often against traffic flow direction  
             *  when used for measuring queues) 
             * 
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ZoneLength
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
            struct SpeedConfidenceDSRC
            {
                enum Values {
                    unavailable = 0,
                    prec100ms   = 1,
                    prec10ms    = 2,
                    prec5ms     = 3,
                    prec1ms     = 4,
                    prec0_1ms   = 5,
                    prec0_05ms  = 6,
                    prec0_01ms  = 7
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * LSB units are 0.1 m/s^2
             * the value 499 shall be used for values at or greater than 49.9 m/s
             * the value 500 shall be used to indicate that speed is unavailable
             * 
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpeedAdvice
            {
                static constexpr float MIN          = 0;
                static constexpr float MAX          = 50;
                static constexpr int32_t SCALEDIVISOR = 10;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct AdvisorySpeedType
            {
                enum Values {
                    none       = 0,
                    greenwave  = 1,
                    ecoDrive   = 2,
                    transit    = 3
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Node_LLmD_64b
            {
                ITS_Container::Longitude lon; // 8
                ITS_Container::Latitude  lat; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Node_XY_32b
            {
                Offset_B16 x; // 4
                Offset_B16 y; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Node_XY_28b
            {
                Offset_B14 x; // 4
                Offset_B14 y; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Node_XY_26b
            {
                Offset_B13 x; // 4
                Offset_B13 y; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Node_XY_24b
            {
                Offset_B12 x; // 4
                Offset_B12 y; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Node_XY_22b
            {
                Offset_B11 x; // 4
                Offset_B11 y; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Node_XY_20b
            {
                Offset_B10 x; // 4
                Offset_B10 y; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 13 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AdvisorySpeed
            {
                bool                speedPresent; // 13
                bool                confidencePresent; // 13
                bool                distancePresent; // 13
                bool                class_Present; // 13
                AdvisorySpeedType   type; // 1
                SpeedAdvice         speed; // 4
                SpeedConfidenceDSRC confidence; // 1
                ZoneLength          distance; // 2
                RestrictionClassID  class_; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             *  Tenths of a second in the current or next hour 
             *  In units of 1/10th second from UTC time 
             *  A range of 0~36000 covers one hour 
             *  The values 35991..36000 are used when a leap second occurs 
             *  The value 36001 is used to indicate time >3600 seconds 
             *  36002 is to be used when value undefined or unknown 
             *  Note that this is NOT expressed in GPS time 
             *  or in local time 
             * 
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimeMark
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 36001;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct TimeIntervalConfidence
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 15;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             *  The values zero through 255 are allocated for testing purposes 
             *  Note that the value assigned to an intersection will be 
             *  unique within a given regional ID only 
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
             * Element type: BIT_STRING
             * Size: 12 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AllowedManeuvers
            {
                enum Options {
                    maneuverStraightAllowed       = 0,
                    maneuverLeftAllowed           = 1,
                    maneuverRightAllowed          = 2,
                    maneuverUTurnAllowed          = 3,
                    maneuverLeftTurnOnRedAllowed  = 4,
                    maneuverRightTurnOnRedAllowed = 5,
                    maneuverLaneChangeAllowed     = 6,
                    maneuverNoStoppingAllowed     = 7,
                    yieldAllwaysRequired          = 8,
                    goWithHalt                    = 9,
                    caution                       = 10,
                    reserved1                     = 11
                };
                bool      values[12]; // 12
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct LaneID
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
            struct DrivenLineOffsetLg
            {
                static constexpr int16_t MIN        = -32767;
                static constexpr int16_t MAX        = 32767;
                
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
            struct DrivenLineOffsetSm
            {
                static constexpr int16_t MIN        = -2047;
                static constexpr int16_t MAX        = 2047;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 17 bytes.
             *
             */
            #pragma pack(push, 1)
            struct NodeOffsetPointXY
            {
                enum Choice {
                    C_node_XY1    = 0,
                    C_node_XY2    = 1,
                    C_node_XY3    = 2,
                    C_node_XY4    = 3,
                    C_node_XY5    = 4,
                    C_node_XY6    = 5,
                    C_node_LatLon = 6
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    Node_XY_20b   node_XY1; // 8
                    Node_XY_22b   node_XY2; // 8
                    Node_XY_24b   node_XY3; // 8
                    Node_XY_26b   node_XY4; // 8
                    Node_XY_28b   node_XY5; // 8
                    Node_XY_32b   node_XY6; // 8
                    Node_LLmD_64b node_LatLon; // 16
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct VehicleType
            {
                enum Values {
                    none                 = 0,
                    unknown              = 1,
                    special              = 2,
                    moto                 = 3,
                    car                  = 4,
                    carOther             = 5,
                    bus                  = 6,
                    axleCnt2             = 7,
                    axleCnt3             = 8,
                    axleCnt4             = 9,
                    axleCnt4Trailer      = 10,
                    axleCnt5Trailer      = 11,
                    axleCnt6Trailer      = 12,
                    axleCnt5MultiTrailer = 13,
                    axleCnt6MultiTrailer = 14,
                    axleCnt7MultiTrailer = 15
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RequestImportanceLevel
            {
                enum Values {
                    requestImportanceLevelUnKnown = 0,
                    requestImportanceLevel1       = 1,
                    requestImportanceLevel2       = 2,
                    requestImportanceLevel3       = 3,
                    requestImportanceLevel4       = 4,
                    requestImportanceLevel5       = 5,
                    requestImportanceLevel6       = 6,
                    requestImportanceLevel7       = 7,
                    requestImportanceLevel8       = 8,
                    requestImportanceLevel9       = 9,
                    requestImportanceLevel10      = 10,
                    requestImportanceLevel11      = 11,
                    requestImportanceLevel12      = 12,
                    requestImportanceLevel13      = 13,
                    requestImportanceLevel14      = 14,
                    requestImportanceReserved     = 15
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RequestSubRole
            {
                enum Values {
                    requestSubRoleUnKnown  = 0,
                    requestSubRole1        = 1,
                    requestSubRole2        = 2,
                    requestSubRole3        = 3,
                    requestSubRole4        = 4,
                    requestSubRole5        = 5,
                    requestSubRole6        = 6,
                    requestSubRole7        = 7,
                    requestSubRole8        = 8,
                    requestSubRole9        = 9,
                    requestSubRole10       = 10,
                    requestSubRole11       = 11,
                    requestSubRole12       = 12,
                    requestSubRole13       = 13,
                    requestSubRole14       = 14,
                    requestSubRoleReserved = 15
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct BasicVehicleRole
            {
                enum Values {
                    basicVehicle     = 0,
                    publicTransport  = 1,
                    specialTransport = 2,
                    dangerousGoods   = 3,
                    roadWork         = 4,
                    roadRescue       = 5,
                    emergency        = 6,
                    safetyCar        = 7,
                    none_unknown     = 8,
                    truck            = 9,
                    motorcycle       = 10,
                    roadSideSource   = 11,
                    police           = 12,
                    fire             = 13,
                    ambulance        = 14,
                    dot              = 15,
                    transit          = 16,
                    slowMoving       = 17,
                    stopNgo          = 18,
                    cyclist          = 19,
                    pedestrian       = 20,
                    nonMotorized     = 21,
                    military         = 22
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: OCTET_STRING
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TemporaryID
            {
                int8_t    count; // 5
                int8_t    value[4]; // 5
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BOOLEAN
             *
             */
            #pragma pack(push, 1)
            struct PedestrianBicycleDetect
            {
                bool      value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BOOLEAN
             *
             */
            #pragma pack(push, 1)
            struct WaitOnStopline
            {
                bool      value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct LaneConnectionID
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 209 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AdvisorySpeedList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 16;
                
                int8_t        count; // 209
                AdvisorySpeed elements[16]; // 13
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimeChangeDetails
            {
                bool                   startTimePresent; // 16
                bool                   maxEndTimePresent; // 16
                bool                   likelyTimePresent; // 16
                bool                   confidencePresent; // 16
                bool                   nextTimePresent; // 16
                TimeMark               startTime; // 2
                TimeMark               minEndTime; // 2
                TimeMark               maxEndTime; // 2
                TimeMark               likelyTime; // 2
                TimeIntervalConfidence confidence; // 1
                TimeMark               nextTime; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct MovementPhaseState
            {
                enum Values {
                    unavailable                 = 0,
                    dark                        = 1,
                    stop_Then_Proceed           = 2,
                    stop_And_Remain             = 3,
                    pre_Movement                = 4,
                    permissive_Movement_Allowed = 5,
                    protected_Movement_Allowed  = 6,
                    permissive_clearance        = 7,
                    protected_clearance         = 8,
                    caution_Conflicting_Traffic = 9
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct SignalGroupID
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
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
             * Element type: SEQUENCE
             * Size: 14 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ConnectingLane
            {
                bool             maneuverPresent; // 14
                LaneID           lane; // 1
                AllowedManeuvers maneuver; // 12
            };
            #pragma pack(pop)
            
            /*
             *
             * in steps of 0.05 percent
             * 
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Scale_B12
            {
                static constexpr int16_t MIN        = -2048;
                static constexpr int16_t MAX        = 2047;
                
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
            struct Angle
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 28800;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ComputedLane_offsetYaxis
            {
                enum Choice {
                    C_small    = 0,
                    C_large    = 1
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    DrivenLineOffsetSm small; // 2
                    DrivenLineOffsetLg large; // 2
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ComputedLane_offsetXaxis
            {
                enum Choice {
                    C_small    = 0,
                    C_large    = 1
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    DrivenLineOffsetSm small; // 2
                    DrivenLineOffsetLg large; // 2
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 17 bytes.
             *
             */
            #pragma pack(push, 1)
            struct NodeXY
            {
                NodeOffsetPointXY delta; // 17
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes_Parking
            {
                enum Options {
                    parkingRevocableLane = 0,
                    parallelParkingInUse = 1,
                    headInParkingInUse   = 2,
                    doNotParkZone        = 3,
                    parkingForBusUse     = 4,
                    parkingForTaxiUse    = 5,
                    noPublicParkingUse   = 6,
                    reserved_00          = 7,
                    reserved_01          = 8,
                    reserved_02          = 9,
                    reserved_03          = 10,
                    reserved_04          = 11,
                    reserved_05          = 12,
                    reserved_06          = 13,
                    reserved_07          = 14,
                    reserved_08          = 15
                };
                bool      values[16]; // 16
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes_TrackedVehicle
            {
                enum Options {
                    spec_RevocableLane         = 0,
                    spec_commuterRailRoadTrack = 1,
                    spec_lightRailRoadTrack    = 2,
                    spec_heavyRailRoadTrack    = 3,
                    spec_otherRailType         = 4,
                    reserved_00                = 5,
                    reserved_01                = 6,
                    reserved_02                = 7,
                    reserved_03                = 8,
                    reserved_04                = 9,
                    reserved_05                = 10,
                    reserved_06                = 11,
                    reserved_07                = 12,
                    reserved_08                = 13,
                    reserved_09                = 14,
                    reserved_10                = 15
                };
                bool      values[16]; // 16
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes_Striping
            {
                enum Options {
                    stripeToConnectingLanesRevocableLane = 0,
                    stripeDrawOnLeft                     = 1,
                    stripeDrawOnRight                    = 2,
                    stripeToConnectingLanesLeft          = 3,
                    stripeToConnectingLanesRight         = 4,
                    stripeToConnectingLanesAhead         = 5,
                    reserved_00                          = 6,
                    reserved_01                          = 7,
                    reserved_02                          = 8,
                    reserved_03                          = 9,
                    reserved_04                          = 10,
                    reserved_05                          = 11,
                    reserved_06                          = 12,
                    reserved_07                          = 13,
                    reserved_08                          = 14,
                    reserved_09                          = 15
                };
                bool      values[16]; // 16
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes_Barrier
            {
                enum Options {
                    median_RevocableLane = 0,
                    median               = 1,
                    whiteLineHashing     = 2,
                    stripedLines         = 3,
                    doubleStripedLines   = 4,
                    trafficCones         = 5,
                    constructionBarrier  = 6,
                    trafficChannels      = 7,
                    lowCurbs             = 8,
                    highCurbs            = 9,
                    reserved_00          = 10,
                    reserved_01          = 11,
                    reserved_02          = 12,
                    reserved_03          = 13,
                    reserved_04          = 14,
                    reserved_05          = 15
                };
                bool      values[16]; // 16
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes_Sidewalk
            {
                enum Options {
                    sidewalk_RevocableLane = 0,
                    bicyleUseAllowed       = 1,
                    isSidewalkFlyOverLane  = 2,
                    walkBikes              = 3,
                    reserved_00            = 4,
                    reserved_01            = 5,
                    reserved_02            = 6,
                    reserved_03            = 7,
                    reserved_04            = 8,
                    reserved_05            = 9,
                    reserved_06            = 10,
                    reserved_07            = 11,
                    reserved_08            = 12,
                    reserved_09            = 13,
                    reserved_10            = 14,
                    reserved_11            = 15
                };
                bool      values[16]; // 16
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes_Bike
            {
                enum Options {
                    bikeRevocableLane           = 0,
                    pedestrianUseAllowed        = 1,
                    isBikeFlyOverLane           = 2,
                    fixedCycleTime              = 3,
                    biDirectionalCycleTimes     = 4,
                    isolatedByBarrier           = 5,
                    unsignalizedSegmentsPresent = 6,
                    reserved_00                 = 7,
                    reserved_01                 = 8,
                    reserved_02                 = 9,
                    reserved_03                 = 10,
                    reserved_04                 = 11,
                    reserved_05                 = 12,
                    reserved_06                 = 13,
                    reserved_07                 = 14,
                    reserved_08                 = 15
                };
                bool      values[16]; // 16
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes_Crosswalk
            {
                enum Options {
                    crosswalkRevocableLane      = 0,
                    bicyleUseAllowed            = 1,
                    isXwalkFlyOverLane          = 2,
                    fixedCycleTime              = 3,
                    biDirectionalCycleTimes     = 4,
                    hasPushToWalkButton         = 5,
                    audioSupport                = 6,
                    rfSignalRequestPresent      = 7,
                    unsignalizedSegmentsPresent = 8,
                    reserved_00                 = 9,
                    reserved_01                 = 10,
                    reserved_02                 = 11,
                    reserved_03                 = 12,
                    reserved_04                 = 13,
                    reserved_05                 = 14,
                    reserved_06                 = 15
                };
                bool      values[16]; // 16
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes_Vehicle
            {
                enum Options {
                    isVehicleRevocableLane  = 0,
                    isVehicleFlyOverLane    = 1,
                    hovLaneUseOnly          = 2,
                    restrictedToBusUse      = 3,
                    restrictedToTaxiUse     = 4,
                    restrictedFromPublicUse = 5,
                    hasIRbeaconCoverage     = 6,
                    permissionOnRequest     = 7
                };
                bool      values[8]; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct ApproachID
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 15;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 9 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RequestorType
            {
                bool                                                                      subrolePresent; // 9
                bool                                                                      requestPresent; // 9
                bool                                                                      iso3883Present; // 9
                bool                                                                      hpmsTypePresent; // 9
                BasicVehicleRole                                                          role; // 1
                RequestSubRole                                                            subrole; // 1
                RequestImportanceLevel                                                    request; // 1
                ElectronicRegistrationIdentificationVehicleDataModule::Iso3833VehicleType iso3883; // 1
                VehicleType                                                               hpmsType; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct MsgCount
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 127;
                
                int8_t    value; // 1
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
             * Element type: CHOICE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleID
            {
                enum Choice {
                    C_entityID  = 0,
                    C_stationID = 1
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    TemporaryID              entityID; // 5
                    ITS_Container::StationID stationID; // 4
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 11 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ConnectionManeuverAssist
            {
                bool                    queueLengthPresent; // 11
                bool                    availableStorageLengthPresent; // 11
                bool                    waitOnStopPresent; // 11
                bool                    pedBicycleDetectPresent; // 11
                LaneConnectionID        connectionID; // 1
                ZoneLength              queueLength; // 2
                ZoneLength              availableStorageLength; // 2
                WaitOnStopline          waitOnStop; // 1
                PedestrianBicycleDetect pedBicycleDetect; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 228 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MovementEvent
            {
                bool               timingPresent; // 228
                bool               speedsPresent; // 228
                MovementPhaseState eventState; // 1
                TimeChangeDetails  timing; // 16
                AdvisorySpeedList  speeds; // 209
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 26 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Connection
            {
                bool                    remoteIntersectionPresent; // 26
                bool                    signalGroupPresent; // 26
                bool                    userClassPresent; // 26
                bool                    connectionIDPresent; // 26
                ConnectingLane          connectingLane; // 14
                IntersectionReferenceID remoteIntersection; // 5
                SignalGroupID           signalGroup; // 1
                RestrictionClassID      userClass; // 1
                LaneConnectionID        connectionID; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ComputedLane
            {
                bool                     rotateXYPresent; // 16
                bool                     scaleXaxisPresent; // 16
                bool                     scaleYaxisPresent; // 16
                LaneID                   referenceLaneId; // 1
                ComputedLane_offsetXaxis offsetXaxis; // 3
                ComputedLane_offsetYaxis offsetYaxis; // 3
                Angle                    rotateXY; // 2
                Scale_B12                scaleXaxis; // 2
                Scale_B12                scaleYaxis; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 1072 bytes.
             *
             */
            #pragma pack(push, 1)
            struct NodeSetXY
            {
                static constexpr int8_t MIN        = 2;
                static constexpr int8_t MAX        = 63;
                
                int8_t    count; // 1072
                NodeXY    elements[63]; // 17
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 17 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneTypeAttributes
            {
                enum Choice {
                    C_vehicle        = 0,
                    C_crosswalk      = 1,
                    C_bikeLane       = 2,
                    C_sidewalk       = 3,
                    C_median         = 4,
                    C_striping       = 5,
                    C_trackedVehicle = 6,
                    C_parking        = 7
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    LaneAttributes_Vehicle        vehicle; // 8
                    LaneAttributes_Crosswalk      crosswalk; // 16
                    LaneAttributes_Bike           bikeLane; // 16
                    LaneAttributes_Sidewalk       sidewalk; // 16
                    LaneAttributes_Barrier        median; // 16
                    LaneAttributes_Striping       striping; // 16
                    LaneAttributes_TrackedVehicle trackedVehicle; // 16
                    LaneAttributes_Parking        parking; // 16
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 10 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneSharing
            {
                enum Options {
                    overlappingLaneDescriptionProvided = 0,
                    multipleLanesTreatedAsOneLane      = 1,
                    otherNonMotorizedTrafficTypes      = 2,
                    individualMotorizedVehicleTraffic  = 3,
                    busVehicleTraffic                  = 4,
                    taxiVehicleTraffic                 = 5,
                    pedestriansTraffic                 = 6,
                    cyclistVehicleTraffic              = 7,
                    trackedVehicleTraffic              = 8,
                    pedestrianTraffic                  = 9
                };
                bool      values[10]; // 10
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneDirection
            {
                enum Options {
                    ingressPath = 0,
                    egressPath  = 1
                };
                bool      values[2]; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct PrioritizationResponseStatus
            {
                enum Values {
                    unknown           = 0,
                    requested         = 1,
                    processing        = 2,
                    watchOtherTraffic = 3,
                    granted           = 4,
                    rejected          = 5,
                    maxPresence       = 6,
                    reserviceLocked   = 7
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DSecond
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             *  the value 527040 shall be used for invalid 
             * 
             * Element type: INTEGER
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MinuteOfTheYear
            {
                static constexpr int32_t MIN        = 0;
                static constexpr int32_t MAX        = 527040;
                
                int32_t   value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IntersectionAccessPoint
            {
                enum Choice {
                    C_lane       = 0,
                    C_approach   = 1,
                    C_connection = 2
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    LaneID           lane; // 1
                    ApproachID       approach; // 1
                    LaneConnectionID connection; // 1
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 20 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalRequesterInfo
            {
                bool             rolePresent; // 20
                bool             typeDataPresent; // 20
                VehicleID        id; // 6
                RequestID        request; // 1
                MsgCount         sequenceNumber; // 1
                BasicVehicleRole role; // 1
                RequestorType    typeData; // 9
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 177 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ManeuverAssistList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 16;
                
                int8_t                   count; // 177
                ConnectionManeuverAssist elements[16]; // 11
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 685 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MovementEventList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 3;
                
                int8_t        count; // 685
                MovementEvent elements[3]; // 228
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 63 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DescriptiveName
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 63;
                
                char      value[MAX]; // 63
            };
            #pragma pack(pop)
            
            /*
             *
             *  Units of 0.02 m/s 
             *  The value 8191 indicates that 
             *  velocity is unavailable 
             * 
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Velocity
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 8191;
                
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
            struct SpeedLimitType
            {
                enum Values {
                    unknown                                    = 0,
                    maxSpeedInSchoolZone                       = 1,
                    maxSpeedInSchoolZoneWhenChildrenArePresent = 2,
                    maxSpeedInConstructionZone                 = 3,
                    vehicleMinSpeed                            = 4,
                    vehicleMaxSpeed                            = 5,
                    vehicleNightMaxSpeed                       = 6,
                    truckMinSpeed                              = 7,
                    truckMaxSpeed                              = 8,
                    truckNightMaxSpeed                         = 9,
                    vehiclesWithTrailersMinSpeed               = 10,
                    vehiclesWithTrailersMaxSpeed               = 11,
                    vehiclesWithTrailersNightMaxSpeed          = 12
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RestrictionAppliesTo
            {
                enum Values {
                    none                     = 0,
                    equippedTransit          = 1,
                    equippedTaxis            = 2,
                    equippedOther            = 3,
                    emissionCompliant        = 4,
                    equippedBicycle          = 5,
                    weightCompliant          = 6,
                    heightCompliant          = 7,
                    pedestrians              = 8,
                    slowMovingPersons        = 9,
                    wheelchairUsers          = 10,
                    visualDisabilities       = 11,
                    audioDisabilities        = 12,
                    otherUnknownDisabilities = 13
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct OverlayLaneList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 5;
                
                int8_t    count; // 6
                LaneID    elements[5]; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 417 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ConnectsToList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 16;
                
                int8_t     count; // 417
                Connection elements[16]; // 26
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 1073 bytes.
             *
             */
            #pragma pack(push, 1)
            struct NodeListXY
            {
                enum Choice {
                    C_nodes    = 0,
                    C_computed = 1
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    NodeSetXY    nodes; // 1072
                    ComputedLane computed; // 16
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 29 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes
            {
                LaneDirection      directionalUse; // 2
                LaneSharing        sharedWith; // 10
                LaneTypeAttributes laneType; // 17
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 38 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalStatusPackage
            {
                bool                         requesterPresent; // 38
                bool                         outboundOnPresent; // 38
                bool                         minutePresent; // 38
                bool                         secondPresent; // 38
                bool                         durationPresent; // 38
                SignalRequesterInfo          requester; // 20
                IntersectionAccessPoint      inboundOn; // 2
                IntersectionAccessPoint      outboundOn; // 2
                MinuteOfTheYear              minute; // 4
                DSecond                      second; // 2
                DSecond                      duration; // 2
                PrioritizationResponseStatus status; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 928 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MovementState
            {
                bool               movementNamePresent; // 928
                bool               maneuverAssistListPresent; // 928
                DescriptiveName    movementName; // 63
                SignalGroupID      signalGroup; // 1
                MovementEventList  state_time_speed; // 685
                ManeuverAssistList maneuverAssistList; // 177
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RegulatorySpeedLimit
            {
                SpeedLimitType type; // 1
                Velocity       speed; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RestrictionUserType
            {
                enum Choice {
                    C_basicType = 0
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    RestrictionAppliesTo basicType; // 1
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1609 bytes.
             *
             */
            #pragma pack(push, 1)
            struct GenericLane
            {
                bool             namePresent; // 1609
                bool             ingressApproachPresent; // 1609
                bool             egressApproachPresent; // 1609
                bool             maneuversPresent; // 1609
                bool             connectsToPresent; // 1609
                bool             overlaysPresent; // 1609
                LaneID           laneID; // 1
                DescriptiveName  name; // 63
                ApproachID       ingressApproach; // 1
                ApproachID       egressApproach; // 1
                LaneAttributes   laneAttributes; // 29
                AllowedManeuvers maneuvers; // 12
                NodeListXY       nodeList; // 1073
                ConnectsToList   connectsTo; // 417
                OverlayLaneList  overlays; // 6
            };
            #pragma pack(pop)
            
            /*
             *
             *  In units of 1 m steps above or below the reference ellipsoid  
             *  Providing a range of -409.5 to + 6143.9 meters  
             *  The value -409.6 shall be used when Unknown is to be sent 
             * 
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Elevation
            {
                static constexpr float MIN          = -409.6;
                static constexpr float MAX          = 6143.9;
                static constexpr int32_t SCALEDIVISOR = 10;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct TransmissionState
            {
                enum Values {
                    neutral      = 0,
                    park         = 1,
                    forwardGears = 2,
                    reverseGears = 3,
                    reserved1    = 4,
                    reserved2    = 5,
                    reserved3    = 6,
                    unavailable  = 7
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct PriorityRequestType
            {
                enum Values {
                    priorityRequestTypeReserved = 0,
                    priorityRequest             = 1,
                    priorityRequestUpdate       = 2,
                    priorityCancellation        = 3
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 1217 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalStatusPackageList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 32;
                
                int8_t              count; // 1217
                SignalStatusPackage elements[32]; // 38
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 14849 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MovementList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 16;
                
                int8_t        count; // 14849
                MovementState elements[16]; // 928
            };
            #pragma pack(pop)
            
            /*
             *
             *  The unique ID numbers for each
             *  lane object which is 'active' 
             *  as part of the dynamic map contents.
             * 
             * Element type: SEQUENCE_OF
             * Size: 17 bytes.
             *
             */
            #pragma pack(push, 1)
            struct EnabledLaneList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 16;
                
                int8_t    count; // 17
                LaneID    elements[16]; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IntersectionStatusObject
            {
                enum Options {
                    manualControlIsEnabled                = 0,
                    stopTimeIsActivated                   = 1,
                    failureFlash                          = 2,
                    preemptIsActive                       = 3,
                    signalPriorityIsActive                = 4,
                    fixedTimeOperation                    = 5,
                    trafficDependentOperation             = 6,
                    standbyOperation                      = 7,
                    failureMode                           = 8,
                    off                                   = 9,
                    recentMAPmessageUpdate                = 10,
                    recentChangeInMAPassignedLanesIDsUsed = 11,
                    noValidMAPisAvailableAtThisTime       = 12,
                    noValidSPATisAvailableAtThisTime      = 13,
                    reserved_00                           = 14,
                    reserved_01                           = 15
                };
                bool      values[16]; // 16
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 28 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpeedLimitList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 9;
                
                int8_t               count; // 28
                RegulatorySpeedLimit elements[9]; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             *  In units of 1.5 degrees from north 
             *  the value -180 shall be used to represent  
             *  data is not available or unknown 
             * 
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MergeDivergeNodeAngle
            {
                static constexpr int16_t MIN        = -180;
                static constexpr int16_t MAX        = 180;
                
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
            struct RoadwayCrownAngle
            {
                static constexpr int8_t MIN        = -128;
                static constexpr int8_t MAX        = 127;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DeltaAngle
            {
                static constexpr int16_t MIN        = -150;
                static constexpr int16_t MAX        = 150;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 33 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RestrictionUserTypeList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 16;
                
                int8_t              count; // 33
                RestrictionUserType elements[16]; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 51489 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 32;
                
                int8_t      count; // 51489
                GenericLane elements[32]; // 1609
            };
            #pragma pack(pop)
            
            /*
             *
             *  units of 1 cm
             * 
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneWidth
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 32767;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 21 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Position3D
            {
                bool                     elevationPresent; // 21
                ITS_Container::Latitude  lat; // 8
                ITS_Container::Longitude long_; // 8
                Elevation                elevation; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TransmissionAndSpeed
            {
                TransmissionState transmisson; // 1
                Velocity          speed; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 12 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalRequest
            {
                bool                    outBoundLanePresent; // 12
                IntersectionReferenceID id; // 5
                RequestID               requestID; // 1
                PriorityRequestType     requestType; // 1
                IntersectionAccessPoint inBoundLane; // 2
                IntersectionAccessPoint outBoundLane; // 2
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
             *  a range of +- 2.55 meters 
             * 
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Offset_B09
            {
                static constexpr int16_t MIN        = -256;
                static constexpr int16_t MAX        = 255;
                
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
            struct ThrottleConfidence
            {
                enum Values {
                    unavailable    = 0,
                    prec10percent  = 1,
                    prec1percent   = 2,
                    prec0_5percent = 3
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct HeadingConfidenceDSRC
            {
                enum Values {
                    unavailable   = 0,
                    prec10deg     = 1,
                    prec05deg     = 2,
                    prec01deg     = 3,
                    prec0_1deg    = 4,
                    prec0_05deg   = 5,
                    prec0_01deg   = 6,
                    prec0_0125deg = 7
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct ElevationConfidence
            {
                enum Values {
                    unavailable = 0,
                    elev_500_00 = 1,
                    elev_200_00 = 2,
                    elev_100_00 = 3,
                    elev_050_00 = 4,
                    elev_020_00 = 5,
                    elev_010_00 = 6,
                    elev_005_00 = 7,
                    elev_002_00 = 8,
                    elev_001_00 = 9,
                    elev_000_50 = 10,
                    elev_000_20 = 11,
                    elev_000_10 = 12,
                    elev_000_05 = 13,
                    elev_000_02 = 14,
                    elev_000_01 = 15
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct PositionConfidence
            {
                enum Values {
                    unavailable = 0,
                    a500m       = 1,
                    a200m       = 2,
                    a100m       = 3,
                    a50m        = 4,
                    a20m        = 5,
                    a10m        = 6,
                    a5m         = 7,
                    a2m         = 8,
                    a1m         = 9,
                    a50cm       = 10,
                    a20cm       = 11,
                    a10cm       = 12,
                    a5cm        = 13,
                    a2cm        = 14,
                    a1cm        = 15
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SemiMajorAxisOrientation
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct SemiMinorAxisAccuracy
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
            struct SemiMajorAxisAccuracy
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
            struct DOffset
            {
                static constexpr int16_t MIN        = -840;
                static constexpr int16_t MAX        = 840;
                
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
            struct DMinute
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 60;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct DHour
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 31;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct DDay
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 31;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct DMonth
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 12;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DYear
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 4095;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1223 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalStatus
            {
                MsgCount                sequenceNumber; // 1
                IntersectionReferenceID id; // 5
                SignalStatusPackageList sigStatus; // 1217
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 15139 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IntersectionState
            {
                bool                     namePresent; // 15139
                bool                     moyPresent; // 15139
                bool                     timeStampPresent; // 15139
                bool                     enabledLanesPresent; // 15139
                bool                     maneuverAssistListPresent; // 15139
                DescriptiveName          name; // 63
                IntersectionReferenceID  id; // 5
                MsgCount                 revision; // 1
                IntersectionStatusObject status; // 16
                MinuteOfTheYear          moy; // 4
                DSecond                  timeStamp; // 2
                EnabledLaneList          enabledLanes; // 17
                MovementList             states; // 14849
                ManeuverAssistList       maneuverAssistList; // 177
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 29 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneDataAttribute
            {
                enum Choice {
                    C_pathEndPointAngle    = 0,
                    C_laneCrownPointCenter = 1,
                    C_laneCrownPointLeft   = 2,
                    C_laneCrownPointRight  = 3,
                    C_laneAngle            = 4,
                    C_speedLimits          = 5
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    DeltaAngle            pathEndPointAngle; // 2
                    RoadwayCrownAngle     laneCrownPointCenter; // 1
                    RoadwayCrownAngle     laneCrownPointLeft; // 1
                    RoadwayCrownAngle     laneCrownPointRight; // 1
                    MergeDivergeNodeAngle laneAngle; // 2
                    SpeedLimitList        speedLimits; // 28
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct SegmentAttributeXY
            {
                enum Values {
                    reserved                  = 0,
                    doNotBlock                = 1,
                    whiteLine                 = 2,
                    mergingLaneLeft           = 3,
                    mergingLaneRight          = 4,
                    curbOnLeft                = 5,
                    curbOnRight               = 6,
                    loadingzoneOnLeft         = 7,
                    loadingzoneOnRight        = 8,
                    turnOutPointOnLeft        = 9,
                    turnOutPointOnRight       = 10,
                    adjacentParkingOnLeft     = 11,
                    adjacentParkingOnRight    = 12,
                    adjacentBikeLaneOnLeft    = 13,
                    adjacentBikeLaneOnRight   = 14,
                    sharedBikeLane            = 15,
                    bikeBoxInFront            = 16,
                    transitStopOnLeft         = 17,
                    transitStopOnRight        = 18,
                    transitStopInLane         = 19,
                    sharedWithTrackedVehicle  = 20,
                    safeIsland                = 21,
                    lowCurbsPresent           = 22,
                    rumbleStripPresent        = 23,
                    audibleSignalingPresent   = 24,
                    adaptiveTimingPresent     = 25,
                    rfSignalRequestPresent    = 26,
                    partialCurbIntrusion      = 27,
                    taperToLeft               = 28,
                    taperToRight              = 29,
                    taperToCenterLine         = 30,
                    parallelParking           = 31,
                    headInParking             = 32,
                    freeParking               = 33,
                    timeRestrictionsOnParking = 34,
                    costToPark                = 35,
                    midBlockCurbPresent       = 36,
                    unEvenPavementPresent     = 37
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct NodeAttributeXY
            {
                enum Values {
                    reserved             = 0,
                    stopLine             = 1,
                    roundedCapStyleA     = 2,
                    roundedCapStyleB     = 3,
                    mergePoint           = 4,
                    divergePoint         = 5,
                    downstreamStopLine   = 6,
                    downstreamStartNode  = 7,
                    closedToTraffic      = 8,
                    safeIsland           = 9,
                    curbPresentAtStepOff = 10,
                    hydrantPresent       = 11
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 34 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RestrictionClassAssignment
            {
                RestrictionClassID      id; // 1
                RestrictionUserTypeList users; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 51612 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IntersectionGeometry
            {
                bool                    namePresent; // 51612
                bool                    laneWidthPresent; // 51612
                bool                    speedLimitsPresent; // 51612
                DescriptiveName         name; // 63
                IntersectionReferenceID id; // 5
                MsgCount                revision; // 1
                Position3D              refPoint; // 21
                LaneWidth               laneWidth; // 2
                SpeedLimitList          speedLimits; // 28
                LaneList                laneSet; // 51489
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct DeltaTime
            {
                static constexpr int8_t MIN        = -122;
                static constexpr int8_t MAX        = 121;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct TransitVehicleOccupancy
            {
                enum Values {
                    occupancyUnknown    = 0,
                    occupancyEmpty      = 1,
                    occupancyVeryLow    = 2,
                    occupancyLow        = 3,
                    occupancyMed        = 4,
                    occupancyHigh       = 5,
                    occupancyNearlyFull = 6,
                    occupancyFull       = 7
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TransitVehicleStatus
            {
                enum Options {
                    loading     = 0,
                    anADAuse    = 1,
                    aBikeLoad   = 2,
                    doorOpen    = 3,
                    charging    = 4,
                    atStopLine  = 5,
                    reserved_00 = 6,
                    reserved_01 = 7
                };
                bool      values[8]; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 28 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RequestorPositionVector
            {
                bool                 headingPresent; // 28
                bool                 speedPresent; // 28
                Position3D           position; // 21
                Angle                heading; // 2
                TransmissionAndSpeed speed; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 23 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalRequestPackage
            {
                bool            minutePresent; // 23
                bool            secondPresent; // 23
                bool            durationPresent; // 23
                SignalRequest   request; // 12
                MinuteOfTheYear minute; // 4
                DSecond         second; // 2
                DSecond         duration; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 410296 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RoadLaneSetList
            {
                static constexpr uint8_t MIN        = 1;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t     count; // 410296
                GenericLane elements[255]; // 1609
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
             * Element type: OCTET_STRING
             * Size: 1025 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RTCMmessage
            {
                int16_t   count; // 1025
                int8_t    value[1023]; // 1025
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 10 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AntennaOffsetSet
            {
                Offset_B12 antOffsetX; // 4
                Offset_B09 antOffsetY; // 2
                Offset_B10 antOffsetZ; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct GNSSstatus
            {
                enum Options {
                    unavailable               = 0,
                    isHealthy                 = 1,
                    isMonitored               = 2,
                    baseStationType           = 3,
                    aPDOPofUnder5             = 4,
                    inViewOfUnder5            = 5,
                    localCorrectionsPresent   = 6,
                    networkCorrectionsPresent = 7
                };
                bool      values[8]; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpeedandHeadingandThrottleConfidence
            {
                HeadingConfidenceDSRC heading; // 1
                SpeedConfidenceDSRC   speed; // 1
                ThrottleConfidence    throttle; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PositionConfidenceSet
            {
                PositionConfidence  pos; // 1
                ElevationConfidence elevation; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct TimeConfidence
            {
                enum Values {
                    unavailable             = 0,
                    time_100_000            = 1,
                    time_050_000            = 2,
                    time_020_000            = 3,
                    time_010_000            = 4,
                    time_002_000            = 5,
                    time_001_000            = 6,
                    time_000_500            = 7,
                    time_000_200            = 8,
                    time_000_100            = 9,
                    time_000_050            = 10,
                    time_000_020            = 11,
                    time_000_010            = 12,
                    time_000_005            = 13,
                    time_000_002            = 14,
                    time_000_001            = 15,
                    time_000_000_5          = 16,
                    time_000_000_2          = 17,
                    time_000_000_1          = 18,
                    time_000_000_05         = 19,
                    time_000_000_02         = 20,
                    time_000_000_01         = 21,
                    time_000_000_005        = 22,
                    time_000_000_002        = 23,
                    time_000_000_001        = 24,
                    time_000_000_000_5      = 25,
                    time_000_000_000_2      = 26,
                    time_000_000_000_1      = 27,
                    time_000_000_000_05     = 28,
                    time_000_000_000_02     = 29,
                    time_000_000_000_01     = 30,
                    time_000_000_000_005    = 31,
                    time_000_000_000_002    = 32,
                    time_000_000_000_001    = 33,
                    time_000_000_000_000_5  = 34,
                    time_000_000_000_000_2  = 35,
                    time_000_000_000_000_1  = 36,
                    time_000_000_000_000_05 = 37,
                    time_000_000_000_000_02 = 38,
                    time_000_000_000_000_01 = 39
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PositionalAccuracy
            {
                SemiMajorAxisAccuracy    semiMajor; // 1
                SemiMinorAxisAccuracy    semiMinor; // 1
                SemiMajorAxisOrientation orientation; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct HeadingDSRC
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 28800;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 17 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DDateTime
            {
                bool      yearPresent; // 17
                bool      monthPresent; // 17
                bool      dayPresent; // 17
                bool      hourPresent; // 17
                bool      minutePresent; // 17
                bool      secondPresent; // 17
                bool      offsetPresent; // 17
                DYear     year; // 2
                DMonth    month; // 1
                DDay      day; // 1
                DHour     hour; // 1
                DMinute   minute; // 1
                DSecond   second; // 2
                DOffset   offset; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 255 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DataParameters_geoidUsed
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 255;
                
                char      value[MAX]; // 255
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 255 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DataParameters_lastCheckedDate
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 255;
                
                char      value[MAX]; // 255
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 255 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DataParameters_processAgency
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 255;
                
                char      value[MAX]; // 255
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 255 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DataParameters_processMethod
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 255;
                
                char      value[MAX]; // 255
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 39137 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalStatusList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 32;
                
                int8_t       count; // 39137
                SignalStatus elements[32]; // 1223
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 15140 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IntersectionStateList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 1;
                
                int8_t            count; // 15140
                IntersectionState elements[1]; // 15139
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 233 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneDataAttributeList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 8;
                
                int8_t            count; // 233
                LaneDataAttribute elements[8]; // 29
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 9 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SegmentAttributeXYList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 8;
                
                int8_t             count; // 9
                SegmentAttributeXY elements[8]; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 9 bytes.
             *
             */
            #pragma pack(push, 1)
            struct NodeAttributeXYList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 8;
                
                int8_t          count; // 9
                NodeAttributeXY elements[8]; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 8637 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RestrictionClassList
            {
                static constexpr uint8_t MIN        = 1;
                static constexpr uint8_t MAX        = 254;
                
                uint8_t                    count; // 8637
                RestrictionClassAssignment elements[254]; // 34
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 51613 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IntersectionGeometryList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 1;
                
                int8_t               count; // 51613
                IntersectionGeometry elements[1]; // 51612
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct LayerID
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 100;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct LayerType
            {
                enum Values {
                    none               = 0,
                    mixedContent       = 1,
                    generalMapData     = 2,
                    intersectionData   = 3,
                    curveData          = 4,
                    roadwaySectionData = 5,
                    parkingAreaData    = 6,
                    sharedLaneData     = 7
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 186 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RequestorDescription
            {
                bool                    typePresent; // 186
                bool                    positionPresent; // 186
                bool                    namePresent; // 186
                bool                    routeNamePresent; // 186
                bool                    transitStatusPresent; // 186
                bool                    transitOccupancyPresent; // 186
                bool                    transitSchedulePresent; // 186
                VehicleID               id; // 6
                RequestorType           type; // 9
                RequestorPositionVector position; // 28
                DescriptiveName         name; // 63
                DescriptiveName         routeName; // 63
                TransitVehicleStatus    transitStatus; // 8
                TransitVehicleOccupancy transitOccupancy; // 1
                DeltaTime               transitSchedule; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 737 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalRequestList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 32;
                
                int8_t               count; // 737
                SignalRequestPackage elements[32]; // 23
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 410419 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RoadSegment
            {
                bool                   namePresent; // 410419
                bool                   laneWidthPresent; // 410419
                bool                   speedLimitsPresent; // 410419
                DescriptiveName        name; // 63
                RoadSegmentReferenceID id; // 5
                MsgCount               revision; // 1
                Position3D             refPoint; // 21
                LaneWidth              laneWidth; // 2
                SpeedLimitList         speedLimits; // 28
                RoadLaneSetList        roadLaneSet; // 410296
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 5126 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RTCMmessageList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 5;
                
                int8_t      count; // 5126
                RTCMmessage elements[5]; // 1025
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 18 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RTCMheader
            {
                GNSSstatus       status; // 8
                AntennaOffsetSet offsetSet; // 10
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 60 bytes.
             *
             */
            #pragma pack(push, 1)
            struct FullPositionVector
            {
                bool                                 utcTimePresent; // 60
                bool                                 elevationPresent; // 60
                bool                                 headingPresent; // 60
                bool                                 speedPresent; // 60
                bool                                 posAccuracyPresent; // 60
                bool                                 timeConfidencePresent; // 60
                bool                                 posConfidencePresent; // 60
                bool                                 speedConfidencePresent; // 60
                DDateTime                            utcTime; // 17
                ITS_Container::Longitude             long_; // 8
                ITS_Container::Latitude              lat; // 8
                Elevation                            elevation; // 4
                HeadingDSRC                          heading; // 2
                TransmissionAndSpeed                 speed; // 3
                PositionalAccuracy                   posAccuracy; // 4
                TimeConfidence                       timeConfidence; // 1
                PositionConfidenceSet                posConfidence; // 2
                SpeedandHeadingandThrottleConfidence speedConfidence; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RTCM_Revision
            {
                enum Values {
                    unknown    = 0,
                    rtcmRev2   = 1,
                    rtcmRev3   = 2,
                    reserved   = 3
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RegionId
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1024 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DataParameters
            {
                bool                           processMethodPresent; // 1024
                bool                           processAgencyPresent; // 1024
                bool                           lastCheckedDatePresent; // 1024
                bool                           geoidUsedPresent; // 1024
                DataParameters_processMethod   processMethod; // 255
                DataParameters_processAgency   processAgency; // 255
                DataParameters_lastCheckedDate lastCheckedDate; // 255
                DataParameters_geoidUsed       geoidUsed; // 255
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 39146 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalStatusMessage
            {
                bool             timeStampPresent; // 39146
                bool             sequenceNumberPresent; // 39146
                MinuteOfTheYear  timeStamp; // 4
                DSecond          second; // 2
                MsgCount         sequenceNumber; // 1
                SignalStatusList status; // 39137
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 15209 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SPAT
            {
                bool                  timeStampPresent; // 15209
                bool                  namePresent; // 15209
                MinuteOfTheYear       timeStamp; // 4
                DescriptiveName       name; // 63
                IntersectionStateList intersections; // 15140
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct VehicleHeight
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 127;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 274 bytes.
             *
             */
            #pragma pack(push, 1)
            struct NodeAttributeSetXY
            {
                bool                   localNodePresent; // 274
                bool                   disabledPresent; // 274
                bool                   enabledPresent; // 274
                bool                   dataPresent; // 274
                bool                   dWidthPresent; // 274
                bool                   dElevationPresent; // 274
                NodeAttributeXYList    localNode; // 9
                SegmentAttributeXYList disabled; // 9
                SegmentAttributeXYList enabled; // 9
                LaneDataAttributeList  data; // 233
                Offset_B10             dWidth; // 4
                Offset_B10             dElevation; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DSRCmsgID
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 32767;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 60262 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MapData
            {
                bool                     timeStampPresent; // 60262
                bool                     layerTypePresent; // 60262
                bool                     layerIDPresent; // 60262
                bool                     intersectionsPresent; // 60262
                bool                     restrictionListPresent; // 60262
                MinuteOfTheYear          timeStamp; // 4
                MsgCount                 msgIssueRevision; // 1
                LayerType                layerType; // 1
                LayerID                  layerID; // 1
                IntersectionGeometryList intersections; // 51613
                RestrictionClassList     restrictionList; // 8637
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 933 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalRequestMessage
            {
                bool                 timeStampPresent; // 933
                bool                 sequenceNumberPresent; // 933
                bool                 requestsPresent; // 933
                MinuteOfTheYear      timeStamp; // 4
                DSecond              second; // 2
                MsgCount             sequenceNumber; // 1
                SignalRequestList    requests; // 737
                RequestorDescription requestor; // 186
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct FuelType
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 15;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 410420 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RoadSegmentList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 1;
                
                int8_t      count; // 410420
                RoadSegment elements[1]; // 410419
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 5213 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RTCMcorrections
            {
                bool               timeStampPresent; // 5213
                bool               anchorPointPresent; // 5213
                bool               rtcmHeaderPresent; // 5213
                MsgCount           msgCnt; // 1
                RTCM_Revision      rev; // 1
                MinuteOfTheYear    timeStamp; // 4
                FullPositionVector anchorPoint; // 60
                RTCMheader         rtcmHeader; // 18
                RTCMmessageList    msgs; // 5126
            };
            #pragma pack(pop)

        }  // Closing namespace DSRC
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //DSRC_V2_DSRC_H
