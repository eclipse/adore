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
#ifndef DSRC_V2_ADDGRPC_H
#define DSRC_V2_ADDGRPC_H

#include <stdint.h>
#include <its_container_v2_its_container.h>
#include <dsrc_v2_dsrc.h>


namespace wind
{
    namespace cpp
    {
        namespace AddGrpC
        {
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimeReference
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 60000;
                
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
            struct Node_id
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 22 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalHeadLocation
            {
                DSRC::NodeOffsetPointXY      nodeXY; // 17
                ITS_Container::DeltaAltitude nodeZ; // 4
                DSRC::SignalGroupID          signalGroupID; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 27 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ItsStationPosition
            {
                bool                     laneIDPresent; // 27
                bool                     nodeXYPresent; // 27
                bool                     timeReferencePresent; // 27
                ITS_Container::StationID stationID; // 4
                DSRC::LaneID             laneID; // 1
                DSRC::NodeOffsetPointXY  nodeXY; // 17
                TimeReference            timeReference; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PrioritizationResponse
            {
                ITS_Container::StationID           stationID; // 4
                DSRC::PrioritizationResponseStatus priorState; // 1
                DSRC::SignalGroupID                signalGroup; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Node
            {
                bool                   lanePresent; // 8
                bool                   connectionIDPresent; // 8
                bool                   intersectionIDPresent; // 8
                Node_id                id; // 1
                DSRC::LaneID           lane; // 1
                DSRC::LaneConnectionID connectionID; // 1
                DSRC::IntersectionID   intersectionID; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 1409 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalHeadLocationList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 64;
                
                int8_t             count; // 1409
                SignalHeadLocation elements[64]; // 22
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct ExceptionalCondition
            {
                enum Values {
                    unknown                  = 0,
                    publicTransportPriority  = 1,
                    emergencyVehiclePriority = 2,
                    trainPriority            = 3,
                    bridgeOpen               = 4,
                    vehicleHeight            = 5,
                    weather                  = 6,
                    trafficJam               = 7,
                    tunnelClosure            = 8,
                    meteringActive           = 9,
                    truckPriority            = 10,
                    bicyclePlatoonPriority   = 11,
                    vehiclePlatoonPriority   = 12
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 136 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ItsStationPositionList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 5;
                
                int8_t             count; // 136
                ItsStationPosition elements[5]; // 27
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 61 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PrioritizationResponseList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 10;
                
                int8_t                 count; // 61
                PrioritizationResponse elements[10]; // 6
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct BatteryStatus
            {
                enum Values {
                    unknown    = 0,
                    critical   = 1,
                    low        = 2,
                    good       = 3
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
            struct EmissionType
            {
                enum Values {
                    euro1      = 0,
                    euro2      = 1,
                    euro3      = 2,
                    euro4      = 3,
                    euro5      = 4,
                    euro6      = 5
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 41 bytes.
             *
             */
            #pragma pack(push, 1)
            struct NodeLink
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 5;
                
                int8_t    count; // 41
                Node      elements[5]; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct PtvRequestType
            {
                enum Values {
                    preRequest       = 0,
                    mainRequest      = 1,
                    doorCloseRequest = 2,
                    cancelRequest    = 3,
                    emergencyRequest = 4
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
            struct RejectedReason
            {
                enum Values {
                    unknown                 = 0,
                    exceptionalCondition    = 1,
                    maxWaitingTimeExceeded  = 2,
                    ptPriorityDisabled      = 3,
                    higherPTPriorityGranted = 4,
                    vehicleTrackingUnknown  = 5
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1073 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ConnectionTrajectory_addGrpC
            {
                DSRC::NodeSetXY        nodes; // 1072
                DSRC::LaneConnectionID connectionID; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1410 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MapData_addGrpC
            {
                bool                   signalHeadLocationsPresent; // 1410
                SignalHeadLocationList signalHeadLocations; // 1409
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LaneAttributes_addGrpC
            {
                bool                       maxVehicleHeightPresent; // 5
                bool                       maxVehicleWeightPresent; // 5
                DSRC::VehicleHeight        maxVehicleHeight; // 1
                ITS_Container::VehicleMass maxVehicleWeight; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MovementEvent_addGrpC
            {
                bool                 stateChangeReasonPresent; // 2
                ExceptionalCondition stateChangeReason; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 137 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ConnectionManeuverAssist_addGrpC
            {
                bool                   itsStationPositionPresent; // 137
                ItsStationPositionList itsStationPosition; // 136
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 62 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IntersectionState_addGrpC
            {
                bool                       activePrioritizationsPresent; // 62
                PrioritizationResponseList activePrioritizations; // 61
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RequestorDescription_addGrpC
            {
                bool           fuelPresent; // 4
                bool           batteryStatusPresent; // 4
                DSRC::FuelType fuel; // 1
                BatteryStatus  batteryStatus; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RestrictionUserType_addGrpC
            {
                bool           emissionPresent; // 4
                bool           fuelPresent; // 4
                EmissionType   emission; // 1
                DSRC::FuelType fuel; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 53 bytes.
             *
             */
            #pragma pack(push, 1)
            struct NodeAttributeSet_addGrpC
            {
                bool           ptvRequestPresent; // 53
                bool           nodeLinkPresent; // 53
                bool           nodePresent; // 53
                PtvRequestType ptvRequest; // 1
                NodeLink       nodeLink; // 41
                Node           node; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Position3D_addGrpC
            {
                ITS_Container::Altitude altitude; // 5
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SignalStatusPackage_addGrpC
            {
                bool            synchToSchedulePresent; // 4
                bool            rejectedReasonPresent; // 4
                DSRC::DeltaTime synchToSchedule; // 1
                RejectedReason  rejectedReason; // 1
            };
            #pragma pack(pop)

        }  // Closing namespace AddGrpC
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //DSRC_V2_ADDGRPC_H
