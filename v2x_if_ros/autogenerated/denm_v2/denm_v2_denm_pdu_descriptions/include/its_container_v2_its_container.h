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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v2:1.6
 * 
 * Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) ts(102894) cdd(2) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */
#ifndef ITS_CONTAINER_V2_ITS_CONTAINER_H
#define ITS_CONTAINER_V2_ITS_CONTAINER_H

#include <stdint.h>


namespace wind
{
    namespace cpp
    {
        namespace ITS_Container
        {
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DeltaAltitude
            {
                static constexpr float MIN          = -127;
                static constexpr float MAX          = 128;
                static constexpr float UNAVAILABLE  = 128;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DeltaLongitude
            {
                static constexpr double MIN          = -0.0131071;
                static constexpr double MAX          = 0.0131072;
                static constexpr double UNAVAILABLE  = 0.0131072;
                static constexpr int32_t SCALEDIVISOR = 10000000;
                
                double    value; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DeltaLatitude
            {
                static constexpr double MIN          = -0.0131071;
                static constexpr double MAX          = 0.0131072;
                static constexpr double UNAVAILABLE  = 0.0131072;
                static constexpr int32_t SCALEDIVISOR = 10000000;
                
                double    value; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct AltitudeConfidence
            {
                enum Values {
                    alt_000_01  = 0,
                    alt_000_02  = 1,
                    alt_000_05  = 2,
                    alt_000_10  = 3,
                    alt_000_20  = 4,
                    alt_000_50  = 5,
                    alt_001_00  = 6,
                    alt_002_00  = 7,
                    alt_005_00  = 8,
                    alt_010_00  = 9,
                    alt_020_00  = 10,
                    alt_050_00  = 11,
                    alt_100_00  = 12,
                    alt_200_00  = 13,
                    outOfRange  = 14,
                    unavailable = 15
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AltitudeValue
            {
                static constexpr float MIN          = -1000;
                static constexpr float MAX          = 8000.01;
                static constexpr float UNAVAILABLE  = 8000.01;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct HeadingValue
            {
                static constexpr float MIN          = 0;
                static constexpr float MAX          = 360.1;
                static constexpr float UNAVAILABLE  = 360.1;
                static constexpr int32_t SCALEDIVISOR = 10;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SemiAxisLength
            {
                static constexpr float MIN          = 0;
                static constexpr float MAX          = 40.95;
                static constexpr float OUT_OF_RANGE = 40.94;
                static constexpr float UNAVAILABLE  = 40.95;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PathDeltaTime
            {
                static constexpr uint16_t MIN        = 1;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 20 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DeltaReferencePosition
            {
                DeltaLatitude  deltaLatitude; // 8
                DeltaLongitude deltaLongitude; // 8
                DeltaAltitude  deltaAltitude; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ProtectedZoneID
            {
                static constexpr int32_t MIN        = 0;
                static constexpr int32_t MAX        = 134217727;
                
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
            struct ProtectedZoneRadius
            {
                static constexpr uint8_t MIN        = 1;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Longitude
            {
                static constexpr double MIN          = -180;
                static constexpr double MAX          = 180.0000001;
                static constexpr double UNAVAILABLE  = 180.0000001;
                static constexpr int32_t SCALEDIVISOR = 10000000;
                
                double    value; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Latitude
            {
                static constexpr double MIN          = -90;
                static constexpr double MAX          = 90.0000001;
                static constexpr double UNAVAILABLE  = 90.0000001;
                static constexpr int32_t SCALEDIVISOR = 10000000;
                
                double    value; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimestampIts
            {
                static constexpr int64_t MIN        = 0;
                static constexpr int64_t MAX        = 4398046511103;
                
                int64_t   value; // 8
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct ProtectedZoneType
            {
                enum Values {
                    permanentCenDsrcTolling = 0,
                    temporaryCenDsrcTolling = 1
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Altitude
            {
                AltitudeValue      altitudeValue; // 4
                AltitudeConfidence altitudeConfidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 12 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PosConfidenceEllipse
            {
                SemiAxisLength semiMajorConfidence; // 4
                SemiAxisLength semiMinorConfidence; // 4
                HeadingValue   semiMajorOrientation; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct InformationQuality
            {
                static constexpr int8_t MIN         = 0;
                static constexpr int8_t MAX         = 7;
                static constexpr int8_t UNAVAILABLE = 0;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 23 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PathPoint
            {
                bool                   pathDeltaTimePresent; // 23
                DeltaReferencePosition pathPosition; // 20
                PathDeltaTime          pathDeltaTime; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct CurvatureConfidence
            {
                enum Values {
                    onePerMeter_0_00002 = 0,
                    onePerMeter_0_0001  = 1,
                    onePerMeter_0_0005  = 2,
                    onePerMeter_0_002   = 3,
                    onePerMeter_0_01    = 4,
                    onePerMeter_0_1     = 5,
                    outOfRange          = 6,
                    unavailable         = 7
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
            struct CurvatureValue
            {
                static constexpr int16_t MIN         = -1023;
                static constexpr int16_t MAX         = 1023;
                static constexpr int16_t UNAVAILABLE = 1023;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: OCTET_STRING
             * Size: 21 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PtActivationData
            {
                int8_t    count; // 21
                int8_t    value[20]; // 21
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct PtActivationType
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 33 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ProtectedCommunicationZone
            {
                bool                expiryTimePresent; // 33
                bool                protectedZoneRadiusPresent; // 33
                bool                protectedZoneIDPresent; // 33
                ProtectedZoneType   protectedZoneType; // 1
                TimestampIts        expiryTime; // 8
                Latitude            protectedZoneLatitude; // 8
                Longitude           protectedZoneLongitude; // 8
                ProtectedZoneRadius protectedZoneRadius; // 1
                ProtectedZoneID     protectedZoneID; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AccelerationConfidence
            {
                static constexpr float MIN          = 0;
                static constexpr float MAX          = 1.02;
                static constexpr float OUT_OF_RANGE = 1.01;
                static constexpr float UNAVAILABLE  = 1.02;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LateralAccelerationValue
            {
                static constexpr float MIN          = -16;
                static constexpr float MAX          = 16.1;
                static constexpr float UNAVAILABLE  = 16.1;
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
            struct VehicleLengthConfidenceIndication
            {
                enum Values {
                    noTrailerPresent                = 0,
                    trailerPresentWithKnownLength   = 1,
                    trailerPresentWithUnknownLength = 2,
                    trailerPresenceIsUnknown        = 3,
                    unavailable                     = 4
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleLengthValue
            {
                static constexpr float MIN          = 0.1;
                static constexpr float MAX          = 102.3;
                static constexpr float OUT_OF_RANGE = 102.2;
                static constexpr float UNAVAILABLE  = 102.3;
                static constexpr int32_t SCALEDIVISOR = 10;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpeedConfidence
            {
                static constexpr float MIN          = 0.01;
                static constexpr float MAX          = 1.27;
                static constexpr float OUT_OF_RANGE = 1.26;
                static constexpr float UNAVAILABLE  = 1.27;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpeedValue
            {
                static constexpr float MIN          = 0;
                static constexpr float MAX          = 163.83;
                static constexpr float UNAVAILABLE  = 163.83;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SequenceNumber
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
            struct StationID
            {
                static constexpr uint32_t MIN        = 0;
                static constexpr uint32_t MAX        = 4294967295;
                
                uint32_t  value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct SubCauseCodeType
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
            struct CauseCodeType
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
            struct CenDsrcTollingZoneID
            {
                static constexpr int32_t MIN        = 0;
                static constexpr int32_t MAX        = 134217727;
                
                int32_t   value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 33 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ReferencePosition
            {
                Latitude             latitude; // 8
                Longitude            longitude; // 8
                PosConfidenceEllipse positionConfidenceEllipse; // 12
                Altitude             altitude; // 5
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LongitudinalAccelerationValue
            {
                static constexpr float MIN          = -16;
                static constexpr float MAX          = 16.1;
                static constexpr float UNAVAILABLE  = 16.1;
                static constexpr int32_t SCALEDIVISOR = 10;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 24 bytes.
             *
             */
            #pragma pack(push, 1)
            struct EventPoint
            {
                bool                   eventDeltaTimePresent; // 24
                DeltaReferencePosition eventPosition; // 20
                PathDeltaTime          eventDeltaTime; // 2
                InformationQuality     informationQuality; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 24 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DangerousGoodsExtended_companyName
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 24;
                
                char      value[MAX]; // 24
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 16 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PhoneNumber
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 16;
                
                char      value[MAX]; // 16
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 24 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DangerousGoodsExtended_emergencyActionCode
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 24;
                
                char      value[MAX]; // 24
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BOOLEAN
             *
             */
            #pragma pack(push, 1)
            struct DangerousGoodsExtended_limitedQuantity
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
            struct DangerousGoodsExtended_tunnelsRestricted
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
            struct DangerousGoodsExtended_elevatedTemperature
            {
                bool      value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DangerousGoodsExtended_unNumber
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 9999;
                
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
            struct DangerousGoodsBasic
            {
                enum Values {
                    explosives1                                          = 0,
                    explosives2                                          = 1,
                    explosives3                                          = 2,
                    explosives4                                          = 3,
                    explosives5                                          = 4,
                    explosives6                                          = 5,
                    flammableGases                                       = 6,
                    nonFlammableGases                                    = 7,
                    toxicGases                                           = 8,
                    flammableLiquids                                     = 9,
                    flammableSolids                                      = 10,
                    substancesLiableToSpontaneousCombustion              = 11,
                    substancesEmittingFlammableGasesUponContactWithWater = 12,
                    oxidizingSubstances                                  = 13,
                    organicPeroxides                                     = 14,
                    toxicSubstances                                      = 15,
                    infectiousSubstances                                 = 16,
                    radioactiveMaterial                                  = 17,
                    corrosiveSubstances                                  = 18,
                    miscellaneousDangerousSubstances                     = 19
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
            struct YawRateConfidence
            {
                enum Values {
                    degSec_000_01 = 0,
                    degSec_000_05 = 1,
                    degSec_000_10 = 2,
                    degSec_001_00 = 3,
                    degSec_005_00 = 4,
                    degSec_010_00 = 5,
                    degSec_100_00 = 6,
                    outOfRange    = 7,
                    unavailable   = 8
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct YawRateValue
            {
                static constexpr float MIN          = -327.66;
                static constexpr float MAX          = 327.67;
                static constexpr float UNAVAILABLE  = 327.67;
                static constexpr int32_t SCALEDIVISOR = 100;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PosPillar
            {
                static constexpr float MIN          = 0.1;
                static constexpr float MAX          = 3;
                static constexpr float UNAVAILABLE  = 3;
                static constexpr int32_t SCALEDIVISOR = 10;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 13 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DrivingLaneStatus
            {
                enum Options {
                    value       = 0,
                    reserved_00 = 1,
                    reserved_01 = 2,
                    reserved_02 = 3,
                    reserved_03 = 4,
                    reserved_04 = 5,
                    reserved_05 = 6,
                    reserved_06 = 7,
                    reserved_07 = 8,
                    reserved_08 = 9,
                    reserved_09 = 10,
                    reserved_10 = 11,
                    reserved_11 = 12
                };
                bool      values[13]; // 13
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct HardShoulderStatus
            {
                enum Values {
                    availableForStopping = 0,
                    closed               = 1,
                    availableForDriving  = 2
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VerticalAccelerationValue
            {
                static constexpr float MIN          = -16;
                static constexpr float MAX          = 16.1;
                static constexpr float UNAVAILABLE  = 16.1;
                static constexpr int32_t SCALEDIVISOR = 10;
                
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
            struct StationType
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VDS
            {
                static constexpr int32_t MIN        = 0;
                static constexpr int32_t MAX        = 6;
                
                char      value[MAX]; // 6
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: TEXT
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct WMInumber
            {
                static constexpr int32_t MIN        = 1;
                static constexpr int32_t MAX        = 3;
                
                char      value[MAX]; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct HeadingConfidence
            {
                static constexpr float MIN          = 0.1;
                static constexpr float MAX          = 12.7;
                static constexpr float OUT_OF_RANGE = 12.6;
                static constexpr float UNAVAILABLE  = 12.7;
                static constexpr int32_t SCALEDIVISOR = 10;
                
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
            struct ItsPduHeader_messageID
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
            struct ItsPduHeader_protocolVersion
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 921 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PathHistory
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 40;
                
                int8_t    count; // 921
                PathPoint elements[40]; // 23
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct SteeringWheelAngleConfidence
            {
                static constexpr int8_t MIN          = 1;
                static constexpr int8_t MAX          = 127;
                static constexpr int8_t OUT_OF_RANGE = 126;
                static constexpr int8_t UNAVAILABLE  = 127;
                
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
            struct SteeringWheelAngleValue
            {
                static constexpr int16_t MIN         = -511;
                static constexpr int16_t MAX         = 512;
                static constexpr int16_t UNAVAILABLE = 512;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PosFrontAx
            {
                static constexpr float MIN          = 0.1;
                static constexpr float MAX          = 2;
                static constexpr float UNAVAILABLE  = 2;
                static constexpr int32_t SCALEDIVISOR = 10;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Curvature
            {
                CurvatureValue      curvatureValue; // 2
                CurvatureConfidence curvatureConfidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 22 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PtActivation
            {
                PtActivationType ptActivationType; // 1
                PtActivationData ptActivationData; // 21
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 529 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ProtectedCommunicationZonesRSU
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 16;
                
                int8_t                     count; // 529
                ProtectedCommunicationZone elements[16]; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LateralAcceleration
            {
                LateralAccelerationValue lateralAccelerationValue; // 4
                AccelerationConfidence   lateralAccelerationConfidence; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct VehicleLength
            {
                VehicleLengthValue                vehicleLengthValue; // 4
                VehicleLengthConfidenceIndication vehicleLengthConfidenceIndication; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct HazardousLocation_DangerousCurveSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct WrongWayDrivingSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct AdverseWeatherCondition_VisibilitySubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct HazardousLocation_ObstacleOnTheRoadSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct PositioningSolutionType
            {
                enum Values {
                    noPositioningSolution = 0,
                    sGNSS                 = 1,
                    dGNSS                 = 2,
                    sGNSSplusDR           = 3,
                    dGNSSplusDR           = 4,
                    dR                    = 5
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Speed
            {
                SpeedValue      speedValue; // 4
                SpeedConfidence speedConfidence; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleMass
            {
                static constexpr int16_t MIN         = 1;
                static constexpr int16_t MAX         = 1024;
                static constexpr int16_t UNAVAILABLE = 1024;
                
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
            struct ActionID
            {
                StationID      originatingStationID; // 4
                SequenceNumber sequenceNumber; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct HeightLonCarr
            {
                static constexpr float MIN          = 0.01;
                static constexpr float MAX          = 1;
                static constexpr float UNAVAILABLE  = 1;
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
            struct CollisionRiskSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct HazardousLocation_SurfaceConditionSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct Temperature
            {
                static constexpr int8_t MIN        = -60;
                static constexpr int8_t MAX        = 67;
                
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
            struct RoadType
            {
                enum Values {
                    urban_NoStructuralSeparationToOppositeLanes      = 0,
                    urban_WithStructuralSeparationToOppositeLanes    = 1,
                    nonUrban_NoStructuralSeparationToOppositeLanes   = 2,
                    nonUrban_WithStructuralSeparationToOppositeLanes = 3
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CauseCode
            {
                CauseCodeType    causeCode; // 1
                SubCauseCodeType subCauseCode; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct AdverseWeatherCondition_AdhesionSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct HumanPresenceOnTheRoadSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpecialTransportType
            {
                enum Options {
                    heavyLoad    = 0,
                    excessWidth  = 1,
                    excessLength = 2,
                    excessHeight = 3
                };
                bool      values[4]; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 21 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CenDsrcTollingZone
            {
                bool                 cenDsrcTollingZoneIDPresent; // 21
                Latitude             protectedZoneLatitude; // 8
                Longitude            protectedZoneLongitude; // 8
                CenDsrcTollingZoneID cenDsrcTollingZoneID; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct DangerousEndOfQueueSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct SpeedLimit
            {
                static constexpr uint8_t MIN        = 1;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct VehicleRole
            {
                enum Values {
                    default_         = 0,
                    publicTransport  = 1,
                    specialTransport = 2,
                    dangerousGoods   = 3,
                    roadWork         = 4,
                    rescue           = 5,
                    emergency        = 6,
                    safetyCar        = 7,
                    agriculture      = 8,
                    commercial       = 9,
                    military         = 10,
                    roadOperator     = 11,
                    taxi             = 12,
                    reserved1        = 13,
                    reserved2        = 14,
                    reserved3        = 15
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 1321 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ItineraryPath
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 40;
                
                int8_t            count; // 1321
                ReferencePosition elements[40]; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct DriveDirection
            {
                enum Values {
                    forward     = 0,
                    backward    = 1,
                    unavailable = 2
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
            struct TrafficRule
            {
                enum Values {
                    noPassing          = 0,
                    noPassingForTrucks = 1,
                    passToRight        = 2,
                    passToLeft         = 3
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LightBarSirenInUse
            {
                enum Options {
                    lightBarActivated = 0,
                    sirenActivated    = 1
                };
                bool      values[2]; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LongitudinalAcceleration
            {
                LongitudinalAccelerationValue longitudinalAccelerationValue; // 4
                AccelerationConfidence        longitudinalAccelerationConfidence; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 553 bytes.
             *
             */
            #pragma pack(push, 1)
            struct EventHistory
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 23;
                
                int8_t     count; // 553
                EventPoint elements[23]; // 24
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 73 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DangerousGoodsExtended
            {
                bool                                       emergencyActionCodePresent; // 73
                bool                                       phoneNumberPresent; // 73
                bool                                       companyNamePresent; // 73
                DangerousGoodsBasic                        dangerousGoodsType; // 1
                DangerousGoodsExtended_unNumber            unNumber; // 2
                DangerousGoodsExtended_elevatedTemperature elevatedTemperature; // 1
                DangerousGoodsExtended_tunnelsRestricted   tunnelsRestricted; // 1
                DangerousGoodsExtended_limitedQuantity     limitedQuantity; // 1
                DangerousGoodsExtended_emergencyActionCode emergencyActionCode; // 24
                PhoneNumber                                phoneNumber; // 16
                DangerousGoodsExtended_companyName         companyName; // 24
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BOOLEAN
             *
             */
            #pragma pack(push, 1)
            struct EmbarkationStatus
            {
                bool      value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PosLonCarr
            {
                static constexpr float MIN          = 0.01;
                static constexpr float MAX          = 1.27;
                static constexpr float UNAVAILABLE  = 1.27;
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
            struct NumberOfOccupants
            {
                static constexpr int8_t MIN         = 0;
                static constexpr int8_t MAX         = 127;
                static constexpr int8_t UNAVAILABLE = 127;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct YawRate
            {
                YawRateValue      yawRateValue; // 4
                YawRateConfidence yawRateConfidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 13 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PositionOfPillars
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 3;
                
                int8_t    count; // 13
                PosPillar elements[3]; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct EmergencyVehicleApproachingSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct AccidentSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct PerformanceClass
            {
                static constexpr int8_t MIN         = 0;
                static constexpr int8_t MAX         = 7;
                static constexpr int8_t UNAVAILABLE = 0;
                
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
            struct AdverseWeatherCondition_PrecipitationSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct TurningRadius
            {
                static constexpr uint8_t MIN         = 1;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 255;
                
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
            struct SlowVehicleSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct ValidityDuration
            {
                static constexpr int32_t MIN        = 0;
                static constexpr int32_t MAX        = 86400;
                
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
            struct PostCrashSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 7 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AccelerationControl
            {
                enum Options {
                    brakePedalEngaged       = 0,
                    gasPedalEngaged         = 1,
                    emergencyBrakeEngaged   = 2,
                    collisionWarningEngaged = 3,
                    accEngaged              = 4,
                    cruiseControlEngaged    = 5,
                    speedLimiterEngaged     = 6
                };
                bool      values[7]; // 7
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 18 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ClosedLanes
            {
                bool               innerhardShoulderStatusPresent; // 18
                bool               outerhardShoulderStatusPresent; // 18
                bool               drivingLaneStatusPresent; // 18
                HardShoulderStatus innerhardShoulderStatus; // 1
                HardShoulderStatus outerhardShoulderStatus; // 1
                DrivingLaneStatus  drivingLaneStatus; // 13
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct EmergencyPriority
            {
                enum Options {
                    requestForRightOfWay                  = 0,
                    requestForFreeCrossingAtATrafficLight = 1
                };
                bool      values[2]; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RescueAndRecoveryWorkInProgressSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VerticalAcceleration
            {
                VerticalAccelerationValue verticalAccelerationValue; // 4
                AccelerationConfidence    verticalAccelerationConfidence; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct StationaryVehicleSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 8450 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DigitalMap
            {
                static constexpr int16_t MIN        = 1;
                static constexpr int16_t MAX        = 256;
                
                int16_t           count; // 8450
                ReferencePosition elements[256]; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RestrictedTypes
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 3;
                
                int8_t      count; // 4
                StationType elements[3]; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TransmissionInterval
            {
                static constexpr int16_t MIN        = 1;
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
            struct TrafficConditionSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleWidth
            {
                static constexpr float MIN          = 0.1;
                static constexpr float MAX          = 6.2;
                static constexpr float OUT_OF_RANGE = 6.1;
                static constexpr float UNAVAILABLE  = 6.2;
                static constexpr int32_t SCALEDIVISOR = 10;
                
                float     value; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 11 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleIdentification
            {
                bool      wMInumberPresent; // 11
                bool      vDSPresent; // 11
                WMInumber wMInumber; // 3
                VDS       vDS; // 6
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Heading
            {
                HeadingValue      headingValue; // 4
                HeadingConfidence headingConfidence; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RelevanceTrafficDirection
            {
                enum Values {
                    allTrafficDirections = 0,
                    upstreamTraffic      = 1,
                    downstreamTraffic    = 2,
                    oppositeTraffic      = 3
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
            struct HumanProblemSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RelevanceDistance
            {
                enum Values {
                    lessThan50m   = 0,
                    lessThan100m  = 1,
                    lessThan200m  = 2,
                    lessThan500m  = 3,
                    lessThan1000m = 4,
                    lessThan5km   = 5,
                    lessThan10km  = 6,
                    over10km      = 7
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
            struct LanePosition
            {
                static constexpr int8_t MIN        = -1;
                static constexpr int8_t MAX        = 14;
                
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
            struct CurvatureCalculationMode
            {
                enum Values {
                    yawRateUsed    = 0,
                    yawRateNotUsed = 1,
                    unavailable    = 2
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct WheelBaseVehicle
            {
                static constexpr float MIN          = 0.1;
                static constexpr float MAX          = 12.7;
                static constexpr float UNAVAILABLE  = 12.7;
                static constexpr int32_t SCALEDIVISOR = 10;
                
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
            struct DangerousSituationSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
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
            struct RoadworksSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ItsPduHeader
            {
                ItsPduHeader_protocolVersion protocolVersion; // 1
                ItsPduHeader_messageID       messageID; // 1
                StationID                    stationID; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct HazardousLocation_AnimalOnTheRoadSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct StationarySince
            {
                enum Values {
                    lessThan1Minute         = 0,
                    lessThan2Minutes        = 1,
                    lessThan15Minutes       = 2,
                    equalOrGreater15Minutes = 3
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 6448 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Traces
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 7;
                
                int8_t      count; // 6448
                PathHistory elements[MAX]; // 921
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: REAL
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PosCentMass
            {
                static constexpr float MIN          = 0.1;
                static constexpr float MAX          = 6.3;
                static constexpr float UNAVAILABLE  = 6.3;
                static constexpr int32_t SCALEDIVISOR = 10;
                
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
            struct VehicleBreakdownSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RequestResponseIndication
            {
                enum Values {
                    request    = 0,
                    response   = 1
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
            struct ExteriorLights
            {
                enum Options {
                    lowBeamHeadlightsOn    = 0,
                    highBeamHeadlightsOn   = 1,
                    leftTurnSignalOn       = 2,
                    rightTurnSignalOn      = 3,
                    daytimeRunningLightsOn = 4,
                    reverseLightOn         = 5,
                    fogLightOn             = 6,
                    parkingLightsOn        = 7
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
            struct SteeringWheelAngle
            {
                SteeringWheelAngleValue      steeringWheelAngleValue; // 2
                SteeringWheelAngleConfidence steeringWheelAngleConfidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 20 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PositionOfOccupants
            {
                enum Options {
                    row1LeftOccupied  = 0,
                    row1RightOccupied = 1,
                    row1MidOccupied   = 2,
                    row1NotDetectable = 3,
                    row1NotPresent    = 4,
                    row2LeftOccupied  = 5,
                    row2RightOccupied = 6,
                    row2MidOccupied   = 7,
                    row2NotDetectable = 8,
                    row2NotPresent    = 9,
                    row3LeftOccupied  = 10,
                    row3RightOccupied = 11,
                    row3MidOccupied   = 12,
                    row3NotDetectable = 13,
                    row3NotPresent    = 14,
                    row4LeftOccupied  = 15,
                    row4RightOccupied = 16,
                    row4MidOccupied   = 17,
                    row4NotDetectable = 18,
                    row4NotPresent    = 19
                };
                bool      values[20]; // 20
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BIT_STRING
             * Size: 7 bytes.
             *
             */
            #pragma pack(push, 1)
            struct EnergyStorageType
            {
                enum Options {
                    hydrogenStorage       = 0,
                    electricEnergyStorage = 1,
                    liquidPropaneGas      = 2,
                    compressedNaturalGas  = 3,
                    diesel                = 4,
                    gasoline              = 5,
                    ammonia               = 6
                };
                bool      values[7]; // 7
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct SignalViolationSubCauseCode
            {
                static constexpr uint8_t MIN         = 0;
                static constexpr uint8_t MAX         = 255;
                static constexpr uint8_t UNAVAILABLE = 0;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)

        }  // Closing namespace ITS_Container
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //ITS_CONTAINER_V2_ITS_CONTAINER_H
