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
#ifndef DSRC_V2_SSEM_PDU_DESCRIPTIONS_H
#define DSRC_V2_SSEM_PDU_DESCRIPTIONS_H

#include <stdint.h>
#include <its_container_v2_its_container.h>
#include <dsrc_v2_dsrc.h>


/**
 * Repository de.dlr.ts.v2x:dsrc_v2:2.5
 * 
 * Main struct is SSEM
 * 
 */
namespace wind
{
    namespace cpp
    {
        namespace SSEM_PDU_Descriptions
        {
            /*
             *
             * Element type: SEQUENCE
             * Size: 39160 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SSEM
            {
                uint16_t                    endiannessCheck = 1;
                uint16_t                    wind_reserved_0 = 0;
                uint16_t                    wind_reserved_1 = 0;
                uint16_t                    wind_reserved_2 = 0;
            
                ITS_Container::ItsPduHeader header; // 6
                DSRC::SignalStatusMessage   ssm; // 39146
            };
            #pragma pack(pop)

        }  // Closing namespace SSEM_PDU_Descriptions
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //DSRC_V2_SSEM_PDU_DESCRIPTIONS_H
