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
#ifndef CPM_V1_19_CPM_PDU_DESCRIPTIONS_H
#define CPM_V1_19_CPM_PDU_DESCRIPTIONS_H

#include <stdint.h>
#include <cam_v2_cam_pdu_descriptions.h>
#include <its_container_v2_its_container.h>
#include <cpm_v1_19_cpm_perceivedobject_descriptions.h>
#include <cpm_v1_19_cpm_commondatatypes_descriptions.h>
#include <cpm_v1_19_cpm_sensorinformation_descriptions.h>


/**
 * Repository de.dlr.ts.v2x:cpm_v1_19:1.3
 * 
 * Main struct is CPM
 * 
 */
namespace wind
{
    namespace cpp
    {
        namespace CPM_PDU_Descriptions
        {
            /*
             *
             * Element type: CHOICE
             * Size: 12553 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CpmPerceptionDataContainer
            {
                enum Choice {
                    C_sensorInformationContainer = 0,
                    C_perceivedObjectContainer   = 1
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    CPM_SensorInformation_Descriptions::SensorInformationContainer sensorInformationContainer; // 2993
                    CPM_PerceivedObject_Descriptions::PerceivedObjectContainer     perceivedObjectContainer; // 12552
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 50213 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CpmParameters_perceptionData
            {
                static constexpr int8_t MIN        = 1;
                static constexpr int8_t MAX        = 4;
                
                int8_t                     count; // 50213
                CpmPerceptionDataContainer elements[4]; // 12553
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 37 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CpmManagementContainer
            {
                bool                                                 messageSegmentInfoPresent; // 37
                ITS_Container::StationType                           stationType; // 1
                CPM_CommonDataTypes_Descriptions::MessageSegmentInfo messageSegmentInfo; // 2
                ITS_Container::ReferencePosition                     referencePosition; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 50251 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CpmParameters
            {
                bool                         perceptionDataPresent; // 50251
                CpmManagementContainer       managementContainer; // 37
                CpmParameters_perceptionData perceptionData; // 50213
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 50253 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CollectivePerceptionMessage
            {
                CAM_PDU_Descriptions::GenerationDeltaTime generationDeltaTime; // 2
                CpmParameters                             cpmParameters; // 50251
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct CpmContainerId
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
            struct CpmStationDataId
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct CpmStationDataContainer
            {
                enum Choice {
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 50267 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CPM
            {
                uint16_t                    endiannessCheck = 1;
                uint16_t                    wind_reserved_0 = 0;
                uint16_t                    wind_reserved_1 = 0;
                uint16_t                    wind_reserved_2 = 0;
            
                ITS_Container::ItsPduHeader header; // 6
                CollectivePerceptionMessage cpm; // 50253
            };
            #pragma pack(pop)

        }  // Closing namespace CPM_PDU_Descriptions
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //CPM_V1_19_CPM_PDU_DESCRIPTIONS_H
