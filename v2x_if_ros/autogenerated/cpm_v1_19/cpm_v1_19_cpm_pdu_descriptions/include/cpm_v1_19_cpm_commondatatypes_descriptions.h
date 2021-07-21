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
#ifndef CPM_V1_19_CPM_COMMONDATATYPES_DESCRIPTIONS_H
#define CPM_V1_19_CPM_COMMONDATATYPES_DESCRIPTIONS_H

#include <stdint.h>
#include <its_container_v2_its_container.h>
#include <dsrc_v2_dsrc.h>


namespace wind
{
    namespace cpp
    {
        namespace CPM_CommonDataTypes_Descriptions
        {
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct AngleConfidence
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
            struct CartesianAngleValue
            {
                static constexpr int16_t MIN         = 0;
                static constexpr int16_t MAX         = 3601;
                static constexpr int16_t UNAVAILABLE = 3601;
                
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
            struct OtherSubclassType
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
            struct AnimalSubclassType
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
            struct PersonSubclassType
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
            struct VehicleSubclassType
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
            struct UnknownSubclassType
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct NodeOffsetPointZ
            {
                enum Choice {
                    C_node_Z1  = 0,
                    C_node_Z2  = 1,
                    C_node_Z3  = 2,
                    C_node_Z4  = 3,
                    C_node_Z5  = 4,
                    C_node_Z6  = 5
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    DSRC::Offset_B10 node_Z1; // 4
                    DSRC::Offset_B11 node_Z2; // 4
                    DSRC::Offset_B12 node_Z3; // 4
                    DSRC::Offset_B13 node_Z4; // 4
                    DSRC::Offset_B14 node_Z5; // 4
                    DSRC::Offset_B16 node_Z6; // 4
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CartesianAngle
            {
                CartesianAngleValue value; // 2
                AngleConfidence     confidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct RearOverhang
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 150;
                
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
            struct FrontOverhang
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 50;
                
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
            struct HitchPointOffset
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 100;
                
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
            struct RefPointId
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
            struct LongitudinalLanePositionConfidence
            {
                static constexpr int8_t MIN          = 0;
                static constexpr int8_t MAX          = 102;
                static constexpr int8_t OUT_OF_RANGE = 101;
                static constexpr int8_t UNAVAILABLE  = 102;
                
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
            struct LongitudinalLanePositionValue
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 32767;
                
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
            struct ClassConfidence
            {
                static constexpr int8_t MIN         = 0;
                static constexpr int8_t MAX         = 101;
                static constexpr int8_t UNAVAILABLE = 101;
                
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
            struct ObjectClass
            {
                enum Choice {
                    C_unknownSubclass = 0,
                    C_vehicleSubclass = 1,
                    C_personSubclass  = 2,
                    C_animalSubclass  = 3,
                    C_otherSubclass   = 4
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    UnknownSubclassType unknownSubclass; // 1
                    VehicleSubclassType vehicleSubclass; // 1
                    PersonSubclassType  personSubclass; // 1
                    AnimalSubclassType  animalSubclass; // 1
                    OtherSubclassType   otherSubclass; // 1
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 23 bytes.
             *
             */
            #pragma pack(push, 1)
            struct OffsetPoint
            {
                bool                    nodeOffsetPointZPresent; // 23
                DSRC::NodeOffsetPointXY nodeOffsetPointxy; // 17
                NodeOffsetPointZ        nodeOffsetPointZ; // 5
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct SegmentCount
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 127;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 13 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TrailerData
            {
                bool                        trailerWidthPresent; // 13
                bool                        hitchAnglePresent; // 13
                RefPointId                  refPointId; // 1
                HitchPointOffset            hitchPointOffset; // 1
                FrontOverhang               frontOverhang; // 1
                RearOverhang                rearOverhang; // 1
                ITS_Container::VehicleWidth trailerWidth; // 4
                CartesianAngle              hitchAngle; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpeedValueExtended
            {
                static constexpr int16_t MIN         = -16383;
                static constexpr int16_t MAX         = 16383;
                static constexpr int16_t UNAVAILABLE = 16383;
                
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
            struct SemiRangeLength
            {
                static constexpr int16_t MIN        = 0;
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
            struct WGS84AngleValue
            {
                static constexpr int16_t MIN         = 0;
                static constexpr int16_t MAX         = 3601;
                static constexpr int16_t UNAVAILABLE = 3601;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LongitudinalLanePosition
            {
                LongitudinalLanePositionValue      longitudinalLanePositionValue; // 2
                LongitudinalLanePositionConfidence longitudinalLanePositionConfidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ObjectClassWithConfidence
            {
                ObjectClass     objectClass; // 2
                ClassConfidence confidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 369 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PolyPointList
            {
                static constexpr int8_t MIN        = 3;
                static constexpr int8_t MAX        = 16;
                
                int8_t      count; // 369
                OffsetPoint elements[16]; // 23
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct Identifier
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
            struct DistanceConfidence
            {
                static constexpr int8_t MIN          = 0;
                static constexpr int8_t MAX          = 102;
                static constexpr int8_t OUT_OF_RANGE = 101;
                static constexpr int8_t UNAVAILABLE  = 102;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 4 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DistanceValue
            {
                static constexpr int32_t MIN        = -132768;
                static constexpr int32_t MAX        = 132767;
                
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
            struct ObjectDimensionConfidence
            {
                static constexpr int8_t MIN          = 0;
                static constexpr int8_t MAX          = 102;
                static constexpr int8_t OUT_OF_RANGE = 101;
                static constexpr int8_t UNAVAILABLE  = 102;
                
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
            struct ObjectDimensionValue
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 1023;
                
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
            struct SensorHeight
            {
                static constexpr int16_t MIN        = -5000;
                static constexpr int16_t MAX        = 5000;
                
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
            struct Range
            {
                static constexpr int16_t MIN        = 0;
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
            struct Radius
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 10000;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct MessageSegmentInfo
            {
                SegmentCount totalMsgSegments; // 1
                SegmentCount thisSegmentNum; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 27 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TrailerDataContainer
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 2;
                
                int8_t      count; // 27
                TrailerData elements[2]; // 13
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpeedExtended
            {
                SpeedValueExtended             value; // 2
                ITS_Container::SpeedConfidence confidence; // 4
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 33 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AreaRectangle
            {
                bool            nodeCenterPointPresent; // 33
                bool            semiHeightPresent; // 33
                OffsetPoint     nodeCenterPoint; // 23
                SemiRangeLength semiMajorRangeLength; // 2
                SemiRangeLength semiMinorRangeLength; // 2
                WGS84AngleValue semiMajorRangeOrientation; // 2
                SemiRangeLength semiHeight; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct TimeOfMeasurement
            {
                static constexpr int16_t MIN        = -1500;
                static constexpr int16_t MAX        = 1500;
                
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
            struct MatchedPosition
            {
                bool                     laneIDPresent; // 6
                bool                     longitudinalLanePositionPresent; // 6
                DSRC::LaneID             laneID; // 1
                LongitudinalLanePosition longitudinalLanePosition; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct DynamicStatus
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 2;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 33 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AreaEllipse
            {
                bool            nodeCenterPointPresent; // 33
                bool            semiHeightPresent; // 33
                OffsetPoint     nodeCenterPoint; // 23
                SemiRangeLength semiMinorRangeLength; // 2
                SemiRangeLength semiMajorRangeLength; // 2
                WGS84AngleValue semiMajorRangeOrientation; // 2
                SemiRangeLength semiHeight; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 25 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ObjectClassDescription
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 8;
                
                int8_t                    count; // 25
                ObjectClassWithConfidence elements[8]; // 3
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct SensorType
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 15;
                
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
            struct VehicleHeight
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
            struct ObjectRefPoint
            {
                static constexpr int8_t MIN        = 0;
                static constexpr int8_t MAX        = 8;
                
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
            struct ObjectConfidence
            {
                static constexpr int8_t MIN         = 0;
                static constexpr int8_t MAX         = 101;
                static constexpr int8_t UNAVAILABLE = 101;
                
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 369 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AreaPolygon
            {
                PolyPointList polyPointList; // 369
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct NumberOfPerceivedObjects
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 129 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SensorIdList
            {
                static constexpr uint8_t MIN        = 1;
                static constexpr uint8_t MAX        = 128;
                
                uint8_t    count; // 129
                Identifier elements[128]; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ObjectDistance
            {
                DistanceValue      value; // 4
                DistanceConfidence confidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ObjectDimension
            {
                ObjectDimensionValue      value; // 2
                ObjectDimensionConfidence confidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct FreeSpaceConfidence
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
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct YSensorOffset
            {
                static constexpr int16_t MIN        = -1000;
                static constexpr int16_t MAX        = 1000;
                
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
            struct ZSensorOffset
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 1000;
                
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
            struct XSensorOffset
            {
                static constexpr int16_t MIN        = -5000;
                static constexpr int16_t MAX        = 0;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 39 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AreaRadial
            {
                bool                verticalOpeningAngleStartPresent; // 39
                bool                verticalOpeningAngleEndPresent; // 39
                bool                sensorPositionOffsetPresent; // 39
                bool                sensorHeightPresent; // 39
                Range               range; // 2
                WGS84AngleValue     stationaryHorizontalOpeningAngleStart; // 2
                WGS84AngleValue     stationaryHorizontalOpeningAngleEnd; // 2
                CartesianAngleValue verticalOpeningAngleStart; // 2
                CartesianAngleValue verticalOpeningAngleEnd; // 2
                OffsetPoint         sensorPositionOffset; // 23
                SensorHeight        sensorHeight; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ObjectAge
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 1500;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: BOOLEAN
             *
             */
            #pragma pack(push, 1)
            struct ShadowingApplies
            {
                bool      value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct WGS84Angle
            {
                WGS84AngleValue value; // 2
                AngleConfidence confidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 26 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AreaCircular
            {
                bool        nodeCenterPointPresent; // 26
                OffsetPoint nodeCenterPoint; // 23
                Radius      radius; // 2
            };
            #pragma pack(pop)

        }  // Closing namespace CPM_CommonDataTypes_Descriptions
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //CPM_V1_19_CPM_COMMONDATATYPES_DESCRIPTIONS_H
