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
#ifndef DSRC_V2_AVIAEINUMBERINGANDDATASTRUCTURES_H
#define DSRC_V2_AVIAEINUMBERINGANDDATASTRUCTURES_H

#include <stdint.h>


namespace wind
{
    namespace cpp
    {
        namespace AVIAEINumberingAndDataStructures
        {
            /*
             *
             * Element type: BIT_STRING
             * Size: 10 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CountryCode
            {
                enum Options {
                    reserved_00 = 0,
                    reserved_01 = 1,
                    reserved_02 = 2,
                    reserved_03 = 3,
                    reserved_04 = 4,
                    reserved_05 = 5,
                    reserved_06 = 6,
                    reserved_07 = 7,
                    reserved_08 = 8,
                    reserved_09 = 9
                };
                bool      values[10]; // 10
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct IssuerIdentifier
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 16383;
                
                int16_t   value; // 2
            };
            #pragma pack(pop)

        }  // Closing namespace AVIAEINumberingAndDataStructures
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //DSRC_V2_AVIAEINUMBERINGANDDATASTRUCTURES_H
