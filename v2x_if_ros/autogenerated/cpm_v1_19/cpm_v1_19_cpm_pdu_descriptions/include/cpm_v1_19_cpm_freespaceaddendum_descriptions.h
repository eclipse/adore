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
#ifndef CPM_V1_19_CPM_FREESPACEADDENDUM_DESCRIPTIONS_H
#define CPM_V1_19_CPM_FREESPACEADDENDUM_DESCRIPTIONS_H

#include <stdint.h>
#include <cpm_v1_19_cpm_commondatatypes_descriptions.h>


namespace wind
{
    namespace cpp
    {
        namespace CPM_FreeSpaceAddendum_Descriptions
        {
            /*
             *
             * Element type: CHOICE
             * Size: 370 bytes.
             *
             */
            #pragma pack(push, 1)
            struct FreeSpaceArea
            {
                enum Choice {
                    C_freeSpacePolygon   = 0,
                    C_freeSpaceCircular  = 1,
                    C_freeSpaceEllipse   = 2,
                    C_freeSpaceRectangle = 3
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    CPM_CommonDataTypes_Descriptions::AreaPolygon   freeSpacePolygon; // 369
                    CPM_CommonDataTypes_Descriptions::AreaCircular  freeSpaceCircular; // 26
                    CPM_CommonDataTypes_Descriptions::AreaEllipse   freeSpaceEllipse; // 33
                    CPM_CommonDataTypes_Descriptions::AreaRectangle freeSpaceRectangle; // 33
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 502 bytes.
             *
             */
            #pragma pack(push, 1)
            struct FreeSpaceAddendum
            {
                bool                                                  sensorIDListPresent; // 502
                CPM_CommonDataTypes_Descriptions::FreeSpaceConfidence freeSpaceConfidence; // 1
                FreeSpaceArea                                         freeSpaceArea; // 370
                CPM_CommonDataTypes_Descriptions::SensorIdList        sensorIDList; // 129
                CPM_CommonDataTypes_Descriptions::ShadowingApplies    shadowingApplies; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE_OF
             * Size: 64257 bytes.
             *
             */
            #pragma pack(push, 1)
            struct FreeSpaceAddendumContainer
            {
                static constexpr uint8_t MIN        = 1;
                static constexpr uint8_t MAX        = 128;
                
                uint8_t           count; // 64257
                FreeSpaceAddendum elements[128]; // 502
            };
            #pragma pack(pop)

        }  // Closing namespace CPM_FreeSpaceAddendum_Descriptions
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //CPM_V1_19_CPM_FREESPACEADDENDUM_DESCRIPTIONS_H
