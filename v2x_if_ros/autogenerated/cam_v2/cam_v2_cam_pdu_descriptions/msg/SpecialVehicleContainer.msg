#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)               *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                           *
#*                                                                                         *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore                *
#*                                                                                         *
#* This program and the accompanying materials are made available under the                *
#* terms of the Eclipse Public License 2.0 which is available at                           *
#* http://www.eclipse.org/legal/epl-2.0.                                                   *
#*                                                                                         *
#* SPDX-License-Identifier: EPL-2.0                                                        *
#*                                                                                         *
#*                                                                                         *
#* File automatically generated with DLR Wind v2 (2021)                                    *
#*                                                                                         *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:cam_v2:1.4                      *
#*                                                                                         *
#* Module: CAM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) en(302637) cam(2) version(2)}                                                *
#*                                                                                         *
#* For support contact v2x-ts@dlr.de                                                       *
#*                                                                                         *

#
# Element type: CHOICE
# 
# ASN.1 Definition:
# CHOICE {
#    publicTransportContainer PublicTransportContainer,
#    specialTransportContainer SpecialTransportContainer,
#    dangerousGoodsContainer DangerousGoodsContainer,
#    roadWorksContainerBasic RoadWorksContainerBasic,
#    rescueContainer RescueContainer,
#    emergencyContainer EmergencyContainer,
#    safetyCarContainer SafetyCarContainer,
#    ... 
# } 
#
# ********** START of SpecialVehicleContainer **********
#

uint8                     choice                    # Use this field to indicate which of the following fields contains valid data.

PublicTransportContainer  publicTransportContainer  # choice = 0
SpecialTransportContainer specialTransportContainer # choice = 1
DangerousGoodsContainer   dangerousGoodsContainer   # choice = 2
RoadWorksContainerBasic   roadWorksContainerBasic   # choice = 3
RescueContainer           rescueContainer           # choice = 4
EmergencyContainer        emergencyContainer        # choice = 5
SafetyCarContainer        safetyCarContainer        # choice = 6

# ********** END of SpecialVehicleContainer   **********
