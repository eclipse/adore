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
#ifndef CPM_V1_19_CPM_ORIGINATINGSTATIONDATA_DESCRIPTIONS_H
#define CPM_V1_19_CPM_ORIGINATINGSTATIONDATA_DESCRIPTIONS_H

#include <stdint.h>
#include <its_container_v2_its_container.h>
#include <cpm_v1_19_cpm_commondatatypes_descriptions.h>
#include <dsrc_v2_dsrc.h>


namespace wind
{
    namespace cpp
    {
        namespace CPM_OriginatingStationData_Descriptions
        {
            /*
             *
             * Element type: SEQUENCE
             * Size: 103 bytes.
             *
             */
            #pragma pack(push, 1)
            struct OriginatingVehicleContainer
            {
                bool                                                   vehicleOrientationAnglePresent; // 103
                bool                                                   longitudinalAccelerationPresent; // 103
                bool                                                   lateralAccelerationPresent; // 103
                bool                                                   verticalAccelerationPresent; // 103
                bool                                                   yawRatePresent; // 103
                bool                                                   pitchAnglePresent; // 103
                bool                                                   rollAnglePresent; // 103
                bool                                                   vehicleLengthPresent; // 103
                bool                                                   vehicleWidthPresent; // 103
                bool                                                   vehicleHeightPresent; // 103
                bool                                                   trailerDataContainerPresent; // 103
                ITS_Container::Heading                                 heading; // 8
                ITS_Container::Speed                                   speed; // 8
                CPM_CommonDataTypes_Descriptions::WGS84Angle           vehicleOrientationAngle; // 3
                ITS_Container::DriveDirection                          driveDirection; // 1
                ITS_Container::LongitudinalAcceleration                longitudinalAcceleration; // 8
                ITS_Container::LateralAcceleration                     lateralAcceleration; // 8
                ITS_Container::VerticalAcceleration                    verticalAcceleration; // 8
                ITS_Container::YawRate                                 yawRate; // 5
                CPM_CommonDataTypes_Descriptions::CartesianAngle       pitchAngle; // 3
                CPM_CommonDataTypes_Descriptions::CartesianAngle       rollAngle; // 3
                ITS_Container::VehicleLength                           vehicleLength; // 5
                ITS_Container::VehicleWidth                            vehicleWidth; // 4
                DSRC::VehicleHeight                                    vehicleHeight; // 1
                CPM_CommonDataTypes_Descriptions::TrailerDataContainer trailerDataContainer; // 27
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct OriginatingRSUContainer
            {
                enum Choice {
                    C_intersectionReferenceId = 0,
                    C_roadSegmentReferenceId  = 1
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    DSRC::IntersectionReferenceID intersectionReferenceId; // 5
                    DSRC::RoadSegmentReferenceID  roadSegmentReferenceId; // 5
                };
            };
            #pragma pack(pop)

        }  // Closing namespace CPM_OriginatingStationData_Descriptions
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //CPM_V1_19_CPM_ORIGINATINGSTATIONDATA_DESCRIPTIONS_H
