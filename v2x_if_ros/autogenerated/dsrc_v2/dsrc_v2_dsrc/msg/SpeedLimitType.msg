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
# ENUMERATED { unknown , maxSpeedInSchoolZone , maxSpeedInSchoolZoneWhenChildrenArePresent , maxSpeedInConstructionZone , vehicleMinSpeed , vehicleMaxSpeed , vehicleNightMaxSpeed , truckMinSpeed , truckMaxSpeed , truckNightMaxSpeed , vehiclesWithTrailersMinSpeed , vehiclesWithTrailersMaxSpeed , vehiclesWithTrailersNightMaxSpeed , ... } 
#
# ********** START of SpeedLimitType **********
#

int8            value

int8            MIN                                                = 0
int8            MAX                                                = 12

int8            UNKNOWN                                            = 0
int8            MAX_SPEED_IN_SCHOOL_ZONE                           = 1
int8            MAX_SPEED_IN_SCHOOL_ZONE_WHEN_CHILDREN_ARE_PRESENT = 2
int8            MAX_SPEED_IN_CONSTRUCTION_ZONE                     = 3
int8            VEHICLE_MIN_SPEED                                  = 4
int8            VEHICLE_MAX_SPEED                                  = 5
int8            VEHICLE_NIGHT_MAX_SPEED                            = 6
int8            TRUCK_MIN_SPEED                                    = 7
int8            TRUCK_MAX_SPEED                                    = 8
int8            TRUCK_NIGHT_MAX_SPEED                              = 9
int8            VEHICLES_WITH_TRAILERS_MIN_SPEED                   = 10
int8            VEHICLES_WITH_TRAILERS_MAX_SPEED                   = 11
int8            VEHICLES_WITH_TRAILERS_NIGHT_MAX_SPEED             = 12

# ********** END of SpeedLimitType   **********
