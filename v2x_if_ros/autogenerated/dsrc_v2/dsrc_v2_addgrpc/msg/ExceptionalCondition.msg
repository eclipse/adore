#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)                  *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                              *
#*                                                                                            *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore                   *
#*                                                                                            *
#* This program and the accompanying materials are made available under the                   *
#* terms of the Eclipse Public License 2.0 which is available at                              *
#* http://www.eclipse.org/legal/epl-2.0.                                                      *
#*                                                                                            *
#* SPDX-License-Identifier: EPL-2.0                                                           *
#*                                                                                            *
#*                                                                                            *
#* File automatically generated with DLR Wind v2 (2021)                                       *
#*                                                                                            *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:dsrc_v2:2.5                        *
#*                                                                                            *
#* Module: RTCMEM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) en(103301) rtcmem(5) version1(1)}                                               *
#*                                                                                            *
#* For support contact v2x-ts@dlr.de                                                          *
#*                                                                                            *

#
# Element type: ENUMERATED
# 
# ASN.1 Definition:
# ENUMERATED { unknown , publicTransportPriority , emergencyVehiclePriority , trainPriority , bridgeOpen , vehicleHeight , weather , trafficJam , tunnelClosure , meteringActive , truckPriority , bicyclePlatoonPriority , vehiclePlatoonPriority , ... } 
#
# ********** START of ExceptionalCondition **********
#

int8            value

int8            MIN                        = 0
int8            MAX                        = 12

int8            UNKNOWN                    = 0
int8            PUBLIC_TRANSPORT_PRIORITY  = 1
int8            EMERGENCY_VEHICLE_PRIORITY = 2
int8            TRAIN_PRIORITY             = 3
int8            BRIDGE_OPEN                = 4
int8            VEHICLE_HEIGHT             = 5
int8            WEATHER                    = 6
int8            TRAFFIC_JAM                = 7
int8            TUNNEL_CLOSURE             = 8
int8            METERING_ACTIVE            = 9
int8            TRUCK_PRIORITY             = 10
int8            BICYCLE_PLATOON_PRIORITY   = 11
int8            VEHICLE_PLATOON_PRIORITY   = 12

# ********** END of ExceptionalCondition   **********
