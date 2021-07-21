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
#ifndef CAM_V2_CAM_PDU_DESCRIPTIONS_H
#define CAM_V2_CAM_PDU_DESCRIPTIONS_H

#include <stdint.h>
#include <its_container_v2_its_container.h>


/**
 * Repository de.dlr.ts.v2x:cam_v2:1.4
 * 
 * Main struct is CAM
 * 
 */
namespace wind
{
    namespace cpp
    {
        namespace CAM_PDU_Descriptions
        {
            /*
             *
             * Element type: SEQUENCE
             * Size: 9 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SafetyCarContainer
            {
                bool                              incidentIndicationPresent; // 9
                bool                              trafficRulePresent; // 9
                bool                              speedLimitPresent; // 9
                ITS_Container::LightBarSirenInUse lightBarSirenInUse; // 2
                ITS_Container::CauseCode          incidentIndication; // 2
                ITS_Container::TrafficRule        trafficRule; // 1
                ITS_Container::SpeedLimit         speedLimit; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 8 bytes.
             *
             */
            #pragma pack(push, 1)
            struct EmergencyContainer
            {
                bool                              incidentIndicationPresent; // 8
                bool                              emergencyPriorityPresent; // 8
                ITS_Container::LightBarSirenInUse lightBarSirenInUse; // 2
                ITS_Container::CauseCode          incidentIndication; // 2
                ITS_Container::EmergencyPriority  emergencyPriority; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RescueContainer
            {
                ITS_Container::LightBarSirenInUse lightBarSirenInUse; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 23 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RoadWorksContainerBasic
            {
                bool                                 roadworksSubCauseCodePresent; // 23
                bool                                 closedLanesPresent; // 23
                ITS_Container::RoadworksSubCauseCode roadworksSubCauseCode; // 1
                ITS_Container::LightBarSirenInUse    lightBarSirenInUse; // 2
                ITS_Container::ClosedLanes           closedLanes; // 18
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1 byte.
             *
             */
            #pragma pack(push, 1)
            struct DangerousGoodsContainer
            {
                ITS_Container::DangerousGoodsBasic dangerousGoodsBasic; // 1
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 6 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpecialTransportContainer
            {
                ITS_Container::SpecialTransportType specialTransportType; // 4
                ITS_Container::LightBarSirenInUse   lightBarSirenInUse; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 24 bytes.
             *
             */
            #pragma pack(push, 1)
            struct PublicTransportContainer
            {
                bool                             ptActivationPresent; // 24
                ITS_Container::EmbarkationStatus embarkationStatus; // 1
                ITS_Container::PtActivation      ptActivation; // 22
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 930 bytes.
             *
             */
            #pragma pack(push, 1)
            struct BasicVehicleContainerLowFrequency
            {
                ITS_Container::VehicleRole    vehicleRole; // 1
                ITS_Container::ExteriorLights exteriorLights; // 8
                ITS_Container::PathHistory    pathHistory; // 921
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 530 bytes.
             *
             */
            #pragma pack(push, 1)
            struct RSUContainerHighFrequency
            {
                bool                                          protectedCommunicationZonesRSUPresent; // 530
                ITS_Container::ProtectedCommunicationZonesRSU protectedCommunicationZonesRSU; // 529
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 99 bytes.
             *
             */
            #pragma pack(push, 1)
            struct BasicVehicleContainerHighFrequency
            {
                bool                                    accelerationControlPresent; // 99
                bool                                    lanePositionPresent; // 99
                bool                                    steeringWheelAnglePresent; // 99
                bool                                    lateralAccelerationPresent; // 99
                bool                                    verticalAccelerationPresent; // 99
                bool                                    performanceClassPresent; // 99
                bool                                    cenDsrcTollingZonePresent; // 99
                ITS_Container::Heading                  heading; // 8
                ITS_Container::Speed                    speed; // 8
                ITS_Container::DriveDirection           driveDirection; // 1
                ITS_Container::VehicleLength            vehicleLength; // 5
                ITS_Container::VehicleWidth             vehicleWidth; // 4
                ITS_Container::LongitudinalAcceleration longitudinalAcceleration; // 8
                ITS_Container::Curvature                curvature; // 3
                ITS_Container::CurvatureCalculationMode curvatureCalculationMode; // 1
                ITS_Container::YawRate                  yawRate; // 5
                ITS_Container::AccelerationControl      accelerationControl; // 7
                ITS_Container::LanePosition             lanePosition; // 1
                ITS_Container::SteeringWheelAngle       steeringWheelAngle; // 3
                ITS_Container::LateralAcceleration      lateralAcceleration; // 8
                ITS_Container::VerticalAcceleration     verticalAcceleration; // 8
                ITS_Container::PerformanceClass         performanceClass; // 1
                ITS_Container::CenDsrcTollingZone       cenDsrcTollingZone; // 21
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 25 bytes.
             *
             */
            #pragma pack(push, 1)
            struct SpecialVehicleContainer
            {
                enum Choice {
                    C_publicTransportContainer  = 0,
                    C_specialTransportContainer = 1,
                    C_dangerousGoodsContainer   = 2,
                    C_roadWorksContainerBasic   = 3,
                    C_rescueContainer           = 4,
                    C_emergencyContainer        = 5,
                    C_safetyCarContainer        = 6
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    PublicTransportContainer  publicTransportContainer; // 24
                    SpecialTransportContainer specialTransportContainer; // 6
                    DangerousGoodsContainer   dangerousGoodsContainer; // 1
                    RoadWorksContainerBasic   roadWorksContainerBasic; // 23
                    RescueContainer           rescueContainer; // 2
                    EmergencyContainer        emergencyContainer; // 8
                    SafetyCarContainer        safetyCarContainer; // 9
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 931 bytes.
             *
             */
            #pragma pack(push, 1)
            struct LowFrequencyContainer
            {
                enum Choice {
                    C_basicVehicleContainerLowFrequency = 0
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    BasicVehicleContainerLowFrequency basicVehicleContainerLowFrequency; // 930
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: CHOICE
             * Size: 531 bytes.
             *
             */
            #pragma pack(push, 1)
            struct HighFrequencyContainer
            {
                enum Choice {
                    C_basicVehicleContainerHighFrequency = 0,
                    C_rsuContainerHighFrequency          = 1
                };
                // Choice selection
                int8_t choice;
            
                union 
                {
                    BasicVehicleContainerHighFrequency basicVehicleContainerHighFrequency; // 99
                    RSUContainerHighFrequency          rsuContainerHighFrequency; // 530
                };
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 34 bytes.
             *
             */
            #pragma pack(push, 1)
            struct BasicContainer
            {
                ITS_Container::StationType       stationType; // 1
                ITS_Container::ReferencePosition referencePosition; // 33
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1523 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CamParameters
            {
                bool                    lowFrequencyContainerPresent; // 1523
                bool                    specialVehicleContainerPresent; // 1523
                BasicContainer          basicContainer; // 34
                HighFrequencyContainer  highFrequencyContainer; // 531
                LowFrequencyContainer   lowFrequencyContainer; // 931
                SpecialVehicleContainer specialVehicleContainer; // 25
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: INTEGER
             * Size: 2 bytes.
             *
             */
            #pragma pack(push, 1)
            struct GenerationDeltaTime
            {
                static constexpr uint16_t MIN        = 0;
                static constexpr uint16_t MAX        = 65535;
                
                uint16_t  value; // 2
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1525 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CoopAwareness
            {
                GenerationDeltaTime generationDeltaTime; // 2
                CamParameters       camParameters; // 1523
            };
            #pragma pack(pop)
            
            /*
             *
             * Element type: SEQUENCE
             * Size: 1539 bytes.
             *
             */
            #pragma pack(push, 1)
            struct CAM
            {
                uint16_t                    endiannessCheck = 1;
                uint16_t                    wind_reserved_0 = 0;
                uint16_t                    wind_reserved_1 = 0;
                uint16_t                    wind_reserved_2 = 0;
            
                ITS_Container::ItsPduHeader header; // 6
                CoopAwareness               cam; // 1525
            };
            #pragma pack(pop)

        }  // Closing namespace CAM_PDU_Descriptions
    }  // Closing namespace cpp
}  // Closing namespace wind

#endif //CAM_V2_CAM_PDU_DESCRIPTIONS_H
