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
#ifndef CPM_V1_19_CPM_SENSORINFORMATION_DESCRIPTIONS_H
#define CPM_V1_19_CPM_SENSORINFORMATION_DESCRIPTIONS_H

#include <stdint.h>
#include <cpm_v1_19_cpm_commondatatypes_descriptions.h>


namespace wind
{
    namespace cpp
    {
        namespace CPM_SensorInformation_Descriptions
        {
            /*
             *
             * Element type: SEQUENCE
             * Size: 12 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleSensorProperties
            {
                bool                                                  verticalOpeningAngleStartPresent; // 12
                bool                                                  verticalOpeningAngleEndPresent; // 12
                CPM_CommonDataTypes_Descriptions::Range               range; // 2
                CPM_CommonDataTypes_Descriptions::CartesianAngleValue horizontalOpeningAngleStart; // 2
                CPM_CommonDataTypes_Descriptions::CartesianAngleValue horizontalOpeningAngleEnd; // 2
                CPM_CommonDataTypes_Descriptions::CartesianAngleValue verticalOpeningAngleStart; // 2
                CPM_CommonDataTypes_Descriptions::CartesianAngleValue verticalOpeningAngleEnd; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 121 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleSensorPropertyList
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 10;
                
                int8_t                  count; // 121
                VehicleSensorProperties elements[10]; // 12
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 129 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleSensor
            {
                bool                                            zSensorOffsetPresent; // 129
                CPM_CommonDataTypes_Descriptions::RefPointId    refPointId; // 1
                CPM_CommonDataTypes_Descriptions::XSensorOffset xSensorOffset; // 2
                CPM_CommonDataTypes_Descriptions::YSensorOffset ySensorOffset; // 2
                CPM_CommonDataTypes_Descriptions::ZSensorOffset zSensorOffset; // 2
                VehicleSensorPropertyList                       vehicleSensorPropertyList; // 121
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 370 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DetectionArea
            {
                enum Choice {
                    C_vehicleSensor             = 0,
                    C_stationarySensorRadial    = 1,
                    C_stationarySensorPolygon   = 2,
                    C_stationarySensorCircular  = 3,
                    C_stationarySensorEllipse   = 4,
                    C_stationarySensorRectangle = 5
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    VehicleSensor                                   vehicleSensor; // 129
                    CPM_CommonDataTypes_Descriptions::AreaRadial    stationarySensorRadial; // 39
                    CPM_CommonDataTypes_Descriptions::AreaPolygon   stationarySensorPolygon; // 369
                    CPM_CommonDataTypes_Descriptions::AreaCircular  stationarySensorCircular; // 26
                    CPM_CommonDataTypes_Descriptions::AreaEllipse   stationarySensorEllipse; // 33
                    CPM_CommonDataTypes_Descriptions::AreaRectangle stationarySensorRectangle; // 33
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 374 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SensorInformation
            {
                bool                                                  freeSpaceConfidencePresent; // 374
                CPM_CommonDataTypes_Descriptions::Identifier          sensorID; // 1
                CPM_CommonDataTypes_Descriptions::SensorType          type; // 1
                DetectionArea                                         detectionArea; // 370
                CPM_CommonDataTypes_Descriptions::FreeSpaceConfidence freeSpaceConfidence; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 2993 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SensorInformationContainer
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 8;
                
                int8_t            count; // 2993
                SensorInformation elements[8]; // 374
            };
            #pragma pack(pop)

        }  // Closing namespace CPM_SensorInformation_Descriptions
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //CPM_V1_19_CPM_SENSORINFORMATION_DESCRIPTIONS_H
