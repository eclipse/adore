# *******************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *  Daniel Heß
# ********************************************************************************
uint8 type_id # type of traffic participant -> classification values

# classification values
uint8 UNCLASSIFIED=0
uint8 UNKNOWN_SMALL=1
uint8 UNKNOWN_BIG=2
uint8 UNDER_DRIVABLE=3  #TODO consider depricating this: not a vehicle type
uint8 OVER_DRIVABLE=4   #TODO consider depricating this: not a vehicle type
uint8 PEDESTRIAN=5
uint8 BIKE=6
uint8 BICYCLE=7
uint8 MOPED=8
uint8 MOTORCYCLE=9
uint8 CAR=10
uint8 VAN=11
uint8 BUS=12
uint8 TRUCK=13
uint8 LIGHT_TRUCK=14
uint8 HEAVY_TRUCK=15
uint8 TRAM=16
uint8 TRAILER=17
uint8 SPECIAL_VEHICLE=18
uint8 PEDESTRIAN_GROUP=19
uint8 TRAIN=20
uint8 HORSE_RIDER=21
uint8 ANIMAL_SMALL=22
uint8 ANIMAL_BIG=23
