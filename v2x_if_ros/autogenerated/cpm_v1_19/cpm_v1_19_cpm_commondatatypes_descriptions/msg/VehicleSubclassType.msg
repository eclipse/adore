#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)                             *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                                         *
#*                                                                                                       *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore                              *
#*                                                                                                       *
#* This program and the accompanying materials are made available under the                              *
#* terms of the Eclipse Public License 2.0 which is available at                                         *
#* http://www.eclipse.org/legal/epl-2.0.                                                                 *
#*                                                                                                       *
#* SPDX-License-Identifier: EPL-2.0                                                                      *
#*                                                                                                       *
#*                                                                                                       *
#* File automatically generated with DLR Wind v2 (2021)                                                  *
#*                                                                                                       *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:cpm_v1_19:1.3                                 *
#*                                                                                                       *
#* Module: CPM_SensorInformation_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(103324) sensorInformation(4) version1(1)}                                               *
#*                                                                                                       *
#* For support contact v2x-ts@dlr.de                                                                     *
#*                                                                                                       *

#
# Element type: NUMERICAL
# 
# ASN.1 Definition (Copyright 2019 ETSI):
# INTEGER { unknown(0), moped(1), motorcycle(2), passengerCar(3), bus(4), lightTruck(5), heavyTruck(6), trailer(7), specialVehicles(8), tram(9), emergencyVehicle(10), agricultural(11) }(0..255) 
#
# ********** START of VehicleSubclassType **********
#

uint8           value

uint8           MIN               = 0
uint8           MAX               = 255

uint8           UNKNOWN           = 0
uint8           MOPED             = 1
uint8           MOTORCYCLE        = 2
uint8           PASSENGER_CAR     = 3
uint8           BUS               = 4
uint8           LIGHT_TRUCK       = 5
uint8           HEAVY_TRUCK       = 6
uint8           TRAILER           = 7
uint8           SPECIAL_VEHICLES  = 8
uint8           TRAM              = 9
uint8           EMERGENCY_VEHICLE = 10
uint8           AGRICULTURAL      = 11

# ********** END of VehicleSubclassType   **********
