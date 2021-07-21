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
#ifndef DSRC_V2_EFCDSRCAPPLICATION_H
#define DSRC_V2_EFCDSRCAPPLICATION_H

#include <stdint.h>
#include <dsrc_v2_aviaeinumberinganddatastructures.h>


namespace wind
{
    namespace cpp
    {
        namespace EfcDsrcApplication
        {
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DieselEmissionValues_particulate_value
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 32767;
                
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
            struct UnitType
            {
                enum Values {
                    mg_km      = 0,
                    mg_kWh     = 1
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
            struct Int2
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DieselEmissionValues_particulate
            {
                UnitType                               unitType; // 1
                DieselEmissionValues_particulate_value value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct Int1
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ExhaustEmissionValues_emissionCO
            {
                static constexpr int16_t MIN        = 0;
                static constexpr int16_t MAX        = 32767;
                
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
            struct CopValue
            {
                enum Values {
                    noEntry        = 0,
                    co2class1      = 1,
                    co2class2      = 2,
                    co2class3      = 3,
                    co2class4      = 4,
                    co2class5      = 5,
                    co2class6      = 6,
                    co2class7      = 7,
                    reservedforUse = 8
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
            struct EuroValue
            {
                enum Values {
                    noEntry         = 0,
                    euro_1          = 1,
                    euro_2          = 2,
                    euro_3          = 3,
                    euro_4          = 4,
                    euro_5          = 5,
                    euro_6          = 6,
                    reservedForUse1 = 7,
                    reservedForUse2 = 8,
                    reservedForUse3 = 9,
                    reservedForUse4 = 10,
                    reservedForUse5 = 11,
                    reservedForUse6 = 12,
                    reservedForUse7 = 13,
                    reservedForUse8 = 14,
                    eev             = 15
                };
                int8_t    value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 10 bytes.
             *
             */
            #pragma pack(push, 1)
            struct AxleWeightLimits
            {
                Int2      maxLadenweightOnAxle1; // 2
                Int2      maxLadenweightOnAxle2; // 2
                Int2      maxLadenweightOnAxle3; // 2
                Int2      maxLadenweightOnAxle4; // 2
                Int2      maxLadenweightOnAxle5; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 12 bytes.
             *
             */
            #pragma pack(push, 1)
            struct Provider
            {
                AVIAEINumberingAndDataStructures::CountryCode      countryCode; // 10
                AVIAEINumberingAndDataStructures::IssuerIdentifier providerIdentifier; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 5 bytes.
             *
             */
            #pragma pack(push, 1)
            struct DieselEmissionValues
            {
                DieselEmissionValues_particulate particulate; // 3
                Int2                             absorptionCoeff; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PassengerCapacity
            {
                Int1      numberOfSeats; // 1
                Int1      numberOfStandingPlaces; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 9 bytes.
             *
             */
            #pragma pack(push, 1)
            struct ExhaustEmissionValues
            {
                UnitType                         unitType; // 1
                ExhaustEmissionValues_emissionCO emissionCO; // 2
                Int2                             emissionHC; // 2
                Int2                             emissionNOX; // 2
                Int2                             emissionHCNOX; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleWeightLimits
            {
                Int2      vehicleMaxLadenWeight; // 2
                Int2      vehicleTrainMaximumWeight; // 2
                Int2      vehicleWeightUnladen; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 3 bytes.
             *
             */
            #pragma pack(push, 1)
            struct VehicleDimensions
            {
                Int1      vehicleLengthOverall; // 1
                Int1      vehicleHeigthOverall; // 1
                Int1      vehicleWidthOverall; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct EnvironmentalCharacteristics
            {
                EuroValue euroValue; // 1
                CopValue  copValue; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct EngineCharacteristics
            {
                static constexpr uint8_t MIN        = 0;
                static constexpr uint8_t MAX        = 255;
                
                uint8_t   value; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SoundLevel
            {
                Int1      soundstationary; // 1
                Int1      sounddriveby; // 1
            };
            #pragma pack(pop)

        }  // Closing namespace EfcDsrcApplication
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //DSRC_V2_EFCDSRCAPPLICATION_H
