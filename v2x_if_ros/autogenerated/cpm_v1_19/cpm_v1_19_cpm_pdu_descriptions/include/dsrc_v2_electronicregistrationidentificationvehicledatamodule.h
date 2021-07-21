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
#ifndef DSRC_V2_ELECTRONICREGISTRATIONIDENTIFICATIONVEHICLEDATAMODULE_H
#define DSRC_V2_ELECTRONICREGISTRATIONIDENTIFICATIONVEHICLEDATAMODULE_H

#include <stdint.h>


namespace wind
{
    namespace cpp
    {
        namespace ElectronicRegistrationIdentificationVehicleDataModule
        {
            /*
             *
             * Element type: ENUMERATED
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct EuVehicleCategoryO
            {
                enum Values {
                    o1         = 0,
                    o2         = 1,
                    o3         = 2,
                    o4         = 3
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
            struct EuVehicleCategoryN
            {
                enum Values {
                    n1         = 0,
                    n2         = 1,
                    n3         = 2
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
            struct EuVehicleCategoryM
            {
                enum Values {
                    m1         = 0,
                    m2         = 1,
                    m3         = 2
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
            struct EuVehicleCategoryL
            {
                enum Values {
                    l1         = 0,
                    l2         = 1,
                    l3         = 2,
                    l4         = 3,
                    l5         = 4,
                    l6         = 5,
                    l7         = 6
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
            struct Iso3833VehicleType
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct EuVehicleCategoryCode
            {
                enum Choice {
                    C_euVehicleCategoryL = 0,
                    C_euVehicleCategoryM = 1,
                    C_euVehicleCategoryN = 2,
                    C_euVehicleCategoryO = 3
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    EuVehicleCategoryL euVehicleCategoryL; // 1
                    EuVehicleCategoryM euVehicleCategoryM; // 1
                    EuVehicleCategoryN euVehicleCategoryN; // 1
                    EuVehicleCategoryO euVehicleCategoryO; // 1
                };
            };
            #pragma pack(pop)

        }  // Closing namespace ElectronicRegistrationIdentificationVehicleDataModule
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //DSRC_V2_ELECTRONICREGISTRATIONIDENTIFICATIONVEHICLEDATAMODULE_H
