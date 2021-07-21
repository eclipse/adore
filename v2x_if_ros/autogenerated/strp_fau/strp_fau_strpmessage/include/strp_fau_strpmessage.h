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
#ifndef STRP_FAU_STRPMESSAGE_H
#define STRP_FAU_STRPMESSAGE_H

#include <stdint.h>
#include <its_container_v1_its_container.h>


/**
 * Repository de.dlr.ts.v2x:strp_fau:1.0
 * 
 * Main struct is SpaceTimeReservationProcedure
 * 
 */
namespace wind
{
    namespace cpp
    {
        namespace STRPMessage
        {
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DistanceAlongPath
            {
                static constexpr int16_t MIN        = -10000;
                static constexpr int16_t MAX        = 10000;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimeOffset
            {
                static constexpr int32_t MIN        = 0;
                static constexpr int32_t MAX        = 100000;
                
                int32_t   value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 12 bytes.
             *
             */
            #pragma pack(push, 1)
            struct FixedBlock
            {
                TimeOffset        t0; // 4
                TimeOffset        t1; // 4
                DistanceAlongPath s0; // 2
                DistanceAlongPath s1; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PositionGate
            {
                TimeOffset                t0; // 4
                TimeOffset                t1; // 4
                DistanceAlongPath         s0; // 2
                DistanceAlongPath         s1; // 2
                ITS_Container::SpeedValue vmin; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimeGate
            {
                TimeOffset                t0; // 4
                TimeOffset                t1; // 4
                DistanceAlongPath         s0; // 2
                DistanceAlongPath         s1; // 2
                ITS_Container::SpeedValue vmin; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RelativePosition_up
            {
                static constexpr int16_t MIN        = -10000;
                static constexpr int16_t MAX        = 10000;
                
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
            struct RelativePosition_north
            {
                static constexpr int16_t MIN        = -10000;
                static constexpr int16_t MAX        = 10000;
                
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
            struct RelativePosition_east
            {
                static constexpr int16_t MIN        = -10000;
                static constexpr int16_t MAX        = 10000;
                
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
            struct RequestID
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 1024;
                
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
            struct ReservationShape
            {
                enum Choice {
                    C_timeGate     = 0,
                    C_positionGate = 1,
                    C_fixedBlock   = 2
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    TimeGate     timeGate; // 16
                    PositionGate positionGate; // 16
                    FixedBlock   fixedBlock; // 12
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RelativePosition
            {
                RelativePosition_east  east; // 2
                RelativePosition_north north; // 2
                RelativePosition_up    up; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 20 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ReferencePosition
            {
                ITS_Container::Latitude      latitude; // 8
                ITS_Container::Longitude     longitude; // 8
                ITS_Container::AltitudeValue altitude; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Cancel
            {
                RequestID requestID; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Reject
            {
                ITS_Container::StationID requestorID; // 4
                RequestID                requestID; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Accept
            {
                ITS_Container::StationID requestorID; // 4
                RequestID                requestID; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 57 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Request
            {
                ITS_Container::TimestampIts generationTime; // 8
                TimeOffset                  timeout; // 4
                RequestID                   requestID; // 2
                ReferencePosition           marker0; // 20
                RelativePosition            marker1; // 6
                ReservationShape            reservation; // 17
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 58 bytes.
             *
             */
            #pragma pack(push, 1)
            struct STRPMessageType
            {
                enum Choice {
                    C_request  = 0,
                    C_accept   = 1,
                    C_reject   = 2,
                    C_cancel   = 3
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    Request   request; // 57
                    Accept    accept; // 6
                    Reject    reject; // 6
                    Cancel    cancel; // 2
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 72 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpaceTimeReservationProcedure
            {
                uint16_t                    endiannessCheck = 1;
                uint16_t                    wind_reserved_0 = 0;
                uint16_t                    wind_reserved_1 = 0;
                uint16_t                    wind_reserved_2 = 0;
            
                ITS_Container::ItsPduHeader header; // 6
                STRPMessageType             data; // 58
            };
            #pragma pack(pop)

        }  // Closing namespace STRPMessage
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //STRP_FAU_STRPMESSAGE_H
