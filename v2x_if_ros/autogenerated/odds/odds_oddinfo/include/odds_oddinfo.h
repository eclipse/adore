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
#ifndef ODDS_ODDINFO_H
#define ODDS_ODDINFO_H

#include <stdint.h>
#include <its_container_v2_its_container.h>


/**
 * Repository de.dlr.ts.v2x:odds:12.0
 * 
 * Main struct is ODDMSG
 * 
 */
namespace wind
{
    namespace cpp
    {
        namespace ODDInfo
        {
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Bearing
            {
                static constexpr int16_t MIN         = 0;
                static constexpr int16_t MAX         = 360;
                static constexpr int16_t UNAVAILABLE = 360;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 18 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PositionWithBearing
            {
                ITS_Container::Latitude  latitude; // 8
                ITS_Container::Longitude longitude; // 8
                Bearing                  bearing; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BOOLEAN
             *
             */
            #pragma pack(push, 1)
            struct IsEmergencVehiclePresent
            {
                bool      value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct InfraTrafficLOS
            {
                enum Options {
                    freeFlow   = 0,
                    medium     = 1,
                    high       = 2,
                    jam        = 3
                };
                bool      values[4]; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BOOLEAN
             *
             */
            #pragma pack(push, 1)
            struct IsConstructionAreaTraffic
            {
                bool      value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ObstacleInformation
            {
                enum Options {
                    human      = 0,
                    animal     = 1,
                    other      = 2
                };
                bool      values[3]; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LightingCondition
            {
                enum Options {
                    day        = 0,
                    dawn       = 1,
                    night      = 2
                };
                bool      values[3]; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct WeatherCondition
            {
                enum Options {
                    sun        = 0,
                    rain       = 1,
                    snow       = 2,
                    ice        = 3,
                    wind       = 4,
                    sleet      = 5
                };
                bool      values[6]; // 6
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 19 bytes.
             *
             */
            #pragma pack(push, 1)
            struct InfraEmergencyVehiclePresence
            {
                IsEmergencVehiclePresent emergencyVehiclePresent; // 1
                PositionWithBearing      emergencyVehiclePos; // 18
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct InfraI2V
            {
                enum Options {
                    spatMap          = 0,
                    vss              = 1,
                    hazard           = 2,
                    constructionArea = 3,
                    cpm              = 4
                };
                bool      values[5]; // 5
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BOOLEAN
             *
             */
            #pragma pack(push, 1)
            struct IsDynamicSpeedDisplay
            {
                bool      value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 7 bytes.
             *
             */
            #pragma pack(push, 1)
            struct InfraTrafficCondition
            {
                bool                      constructionAreaTrafficPresent; // 7
                bool                      losPresent; // 7
                IsConstructionAreaTraffic constructionAreaTraffic; // 1
                InfraTrafficLOS           los; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct InfraEvents
            {
                enum Options {
                    demo         = 0,
                    specialEvent = 1,
                    laneBlocking = 2,
                    roadBlocking = 3
                };
                bool      values[4]; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct InfraBoundary
            {
                enum Options {
                    boundaryMark    = 0,
                    boundaryBarrier = 1,
                    boundaryCurb    = 2,
                    boundaryPole    = 3
                };
                bool      values[4]; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BOOLEAN
             *
             */
            #pragma pack(push, 1)
            struct SignalizedIntersection
            {
                bool      value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct InfraZone
            {
                enum Options {
                    tunnel             = 0,
                    parkingDeck        = 1,
                    parkingArea        = 2,
                    undergroundParking = 3,
                    city               = 4,
                    bridge             = 5,
                    school             = 6,
                    playZone           = 7
                };
                bool      values[8]; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct InfraCategory
            {
                enum Options {
                    highway    = 0,
                    urban      = 1,
                    rural      = 2
                };
                bool      values[3]; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct ValidityInformationConfidence
            {
                static constexpr int8_t MIN         = 0;
                static constexpr int8_t MAX         = 101;
                static constexpr int8_t UNAVAILABLE = 101;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimeStamp
            {
                static constexpr int64_t MIN        = 0;
                static constexpr int64_t MAX        = 9999999999;
                
                int64_t   value; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 15 bytes.
             *
             */
            #pragma pack(push, 1)
            struct EnvironmentalConditions
            {
                bool                weatherPresent; // 15
                bool                lightingPresent; // 15
                bool                obstaclePresent; // 15
                WeatherCondition    weather; // 6
                LightingCondition   lighting; // 3
                ObstacleInformation obstacle; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 41 bytes.
             *
             */
            #pragma pack(push, 1)
            struct OperationalConditions
            {
                bool                          eventsPresent; // 41
                bool                          trafficConditionPresent; // 41
                bool                          dspPresent; // 41
                bool                          i2VPresent; // 41
                bool                          emergencyVehiclePresent; // 41
                InfraEvents                   events; // 4
                InfraTrafficCondition         trafficCondition; // 7
                IsDynamicSpeedDisplay         dsp; // 1
                InfraI2V                      i2V; // 5
                InfraEmergencyVehiclePresence emergencyVehicle; // 19
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 18 bytes.
             *
             */
            #pragma pack(push, 1)
            struct StaticInfrastructure
            {
                bool                   zonePresent; // 18
                bool                   boundaryPresent; // 18
                InfraCategory          category; // 3
                InfraZone              zone; // 8
                SignalizedIntersection sigNode; // 1
                InfraBoundary          boundary; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 17 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ValidityInformation
            {
                TimeStamp                     startTime; // 8
                TimeStamp                     expirationTime; // 8
                ValidityInformationConfidence confidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 36 bytes.
             *
             */
            #pragma pack(push, 1)
            struct GeoInformation
            {
                PositionWithBearing startPosition; // 18
                PositionWithBearing endPosition; // 18
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SectionID
            {
                static constexpr int64_t MIN        = 1;
                static constexpr int64_t MAX        = 9999999999;
                
                int64_t   value; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 63 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Section
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 63;
                
                char      value[MAX]; // 63
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 200 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SectionObject
            {
                bool                    opConditionsPresent; // 200
                bool                    envConditionsPresent; // 200
                Section                 sectionType; // 63
                SectionID               id; // 8
                GeoInformation          geoInfo; // 36
                ValidityInformation     validityInfo; // 17
                StaticInfrastructure    staticInf; // 18
                OperationalConditions   opConditions; // 41
                EnvironmentalConditions envConditions; // 15
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 51001 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SectionObjectList
            {
                static constexpr uint8_t MIN        = 1;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t       count; // 51001
                SectionObject elements[255]; // 200
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 51001 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ODDS
            {
                SectionObjectList sectionObjects; // 51001
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct Status
            {
                enum Values {
                    test       = 0,
                    productive = 1
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 51018 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ODDInfo
            {
                Status    status; // 1
                TimeStamp creationTime; // 8
                TimeStamp serverTime; // 8
                ODDS      odds; // 51001
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 51032 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ODDMSG
            {
                uint16_t                    endiannessCheck = 1;
                uint16_t                    wind_reserved_0 = 0;
                uint16_t                    wind_reserved_1 = 0;
                uint16_t                    wind_reserved_2 = 0;
            
                ITS_Container::ItsPduHeader header; // 6
                ODDInfo                     oddinfo; // 51018
            };
            #pragma pack(pop)

        }  // Closing namespace ODDInfo
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //ODDS_ODDINFO_H
