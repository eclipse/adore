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
# ENUMERATED { occupancyUnknown(0), occupancyEmpty(1), occupancyVeryLow(2), occupancyLow(3), occupancyMed(4), occupancyHigh(5), occupancyNearlyFull(6), occupancyFull(7) } 
#
# ********** START of TransitVehicleOccupancy **********
#

int8            value

int8            MIN                   = 0
int8            MAX                   = 7

int8            OCCUPANCY_UNKNOWN     = 0
int8            OCCUPANCY_EMPTY       = 1
int8            OCCUPANCY_VERY_LOW    = 2
int8            OCCUPANCY_LOW         = 3
int8            OCCUPANCY_MED         = 4
int8            OCCUPANCY_HIGH        = 5
int8            OCCUPANCY_NEARLY_FULL = 6
int8            OCCUPANCY_FULL        = 7

# ********** END of TransitVehicleOccupancy   **********
