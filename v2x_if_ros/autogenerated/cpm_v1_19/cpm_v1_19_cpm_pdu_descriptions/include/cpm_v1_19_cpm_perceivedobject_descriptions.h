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
#ifndef CPM_V1_19_CPM_PERCEIVEDOBJECT_DESCRIPTIONS_H
#define CPM_V1_19_CPM_PERCEIVEDOBJECT_DESCRIPTIONS_H

#include <stdint.h>
#include <its_container_v2_its_container.h>
#include <cpm_v1_19_cpm_commondatatypes_descriptions.h>


namespace wind
{
    namespace cpp
    {
        namespace CPM_PerceivedObject_Descriptions
        {
            /*
             *
             * Element type: SEQUENCE
             * Size: 251 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PerceivedObject
            {
                bool                                                     zDistancePresent; // 251
                bool                                                     zSpeedPresent; // 251
                bool                                                     xAccelerationPresent; // 251
                bool                                                     yAccelerationPresent; // 251
                bool                                                     zAccelerationPresent; // 251
                bool                                                     yawAnglePresent; // 251
                bool                                                     planarObjectDimension1Present; // 251
                bool                                                     planarObjectDimension2Present; // 251
                bool                                                     verticalObjectDimensionPresent; // 251
                bool                                                     objectAgePresent; // 251
                bool                                                     sensorIDListPresent; // 251
                bool                                                     dynamicStatusPresent; // 251
                bool                                                     classificationPresent; // 251
                bool                                                     matchedPositionPresent; // 251
                CPM_CommonDataTypes_Descriptions::Identifier             objectID; // 1
                CPM_CommonDataTypes_Descriptions::TimeOfMeasurement      timeOfMeasurement; // 2
                CPM_CommonDataTypes_Descriptions::ObjectConfidence       objectConfidence; // 1
                CPM_CommonDataTypes_Descriptions::ObjectDistance         xDistance; // 5
                CPM_CommonDataTypes_Descriptions::ObjectDistance         yDistance; // 5
                CPM_CommonDataTypes_Descriptions::ObjectDistance         zDistance; // 5
                CPM_CommonDataTypes_Descriptions::SpeedExtended          xSpeed; // 6
                CPM_CommonDataTypes_Descriptions::SpeedExtended          ySpeed; // 6
                CPM_CommonDataTypes_Descriptions::SpeedExtended          zSpeed; // 6
                ITS_Container::LongitudinalAcceleration                  xAcceleration; // 8
                ITS_Container::LateralAcceleration                       yAcceleration; // 8
                ITS_Container::VerticalAcceleration                      zAcceleration; // 8
                CPM_CommonDataTypes_Descriptions::CartesianAngle         yawAngle; // 3
                CPM_CommonDataTypes_Descriptions::ObjectDimension        planarObjectDimension1; // 3
                CPM_CommonDataTypes_Descriptions::ObjectDimension        planarObjectDimension2; // 3
                CPM_CommonDataTypes_Descriptions::ObjectDimension        verticalObjectDimension; // 3
                CPM_CommonDataTypes_Descriptions::ObjectRefPoint         objectRefPoint; // 1
                CPM_CommonDataTypes_Descriptions::ObjectAge              objectAge; // 2
                CPM_CommonDataTypes_Descriptions::SensorIdList           sensorIDList; // 129
                CPM_CommonDataTypes_Descriptions::DynamicStatus          dynamicStatus; // 1
                CPM_CommonDataTypes_Descriptions::ObjectClassDescription classification; // 25
                CPM_CommonDataTypes_Descriptions::MatchedPosition        matchedPosition; // 6
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 12551 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PerceivedObjectContainer_perceivedObjects
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 50;
                
                int8_t          count; // 12551
                PerceivedObject elements[50]; // 251
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 12552 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PerceivedObjectContainer
            {
                CPM_CommonDataTypes_Descriptions::NumberOfPerceivedObjects numberOfPerceivedObjects; // 1
                PerceivedObjectContainer_perceivedObjects                  perceivedObjects; // 12551
            };
            #pragma pack(pop)

        }  // Closing namespace CPM_PerceivedObject_Descriptions
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //CPM_V1_19_CPM_PERCEIVEDOBJECT_DESCRIPTIONS_H
