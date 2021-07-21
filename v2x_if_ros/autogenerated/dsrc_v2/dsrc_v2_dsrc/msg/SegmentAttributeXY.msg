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
# ENUMERATED { reserved , doNotBlock , whiteLine , mergingLaneLeft , mergingLaneRight , curbOnLeft , curbOnRight , loadingzoneOnLeft , loadingzoneOnRight , turnOutPointOnLeft , turnOutPointOnRight , adjacentParkingOnLeft , adjacentParkingOnRight , adjacentBikeLaneOnLeft , adjacentBikeLaneOnRight , sharedBikeLane , bikeBoxInFront , transitStopOnLeft , transitStopOnRight , transitStopInLane , sharedWithTrackedVehicle , safeIsland , lowCurbsPresent , rumbleStripPresent , audibleSignalingPresent , adaptiveTimingPresent , rfSignalRequestPresent , partialCurbIntrusion , taperToLeft , taperToRight , taperToCenterLine , parallelParking , headInParking , freeParking , timeRestrictionsOnParking , costToPark , midBlockCurbPresent , unEvenPavementPresent , ... } 
#
# ********** START of SegmentAttributeXY **********
#

int8            value

int8            MIN                          = 0
int8            MAX                          = 37

int8            RESERVED                     = 0
int8            DO_NOT_BLOCK                 = 1
int8            WHITE_LINE                   = 2
int8            MERGING_LANE_LEFT            = 3
int8            MERGING_LANE_RIGHT           = 4
int8            CURB_ON_LEFT                 = 5
int8            CURB_ON_RIGHT                = 6
int8            LOADINGZONE_ON_LEFT          = 7
int8            LOADINGZONE_ON_RIGHT         = 8
int8            TURN_OUT_POINT_ON_LEFT       = 9
int8            TURN_OUT_POINT_ON_RIGHT      = 10
int8            ADJACENT_PARKING_ON_LEFT     = 11
int8            ADJACENT_PARKING_ON_RIGHT    = 12
int8            ADJACENT_BIKE_LANE_ON_LEFT   = 13
int8            ADJACENT_BIKE_LANE_ON_RIGHT  = 14
int8            SHARED_BIKE_LANE             = 15
int8            BIKE_BOX_IN_FRONT            = 16
int8            TRANSIT_STOP_ON_LEFT         = 17
int8            TRANSIT_STOP_ON_RIGHT        = 18
int8            TRANSIT_STOP_IN_LANE         = 19
int8            SHARED_WITH_TRACKED_VEHICLE  = 20
int8            SAFE_ISLAND                  = 21
int8            LOW_CURBS_PRESENT            = 22
int8            RUMBLE_STRIP_PRESENT         = 23
int8            AUDIBLE_SIGNALING_PRESENT    = 24
int8            ADAPTIVE_TIMING_PRESENT      = 25
int8            RF_SIGNAL_REQUEST_PRESENT    = 26
int8            PARTIAL_CURB_INTRUSION       = 27
int8            TAPER_TO_LEFT                = 28
int8            TAPER_TO_RIGHT               = 29
int8            TAPER_TO_CENTER_LINE         = 30
int8            PARALLEL_PARKING             = 31
int8            HEAD_IN_PARKING              = 32
int8            FREE_PARKING                 = 33
int8            TIME_RESTRICTIONS_ON_PARKING = 34
int8            COST_TO_PARK                 = 35
int8            MID_BLOCK_CURB_PRESENT       = 36
int8            UN_EVEN_PAVEMENT_PRESENT     = 37

# ********** END of SegmentAttributeXY   **********
