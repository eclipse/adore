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
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    connectionID LaneConnectionID ,
#    queueLength ZoneLength OPTIONAL ,
#    availableStorageLength ZoneLength OPTIONAL ,
#    waitOnStop WaitOnStopline OPTIONAL ,
#    pedBicycleDetect PedestrianBicycleDetect OPTIONAL ,
#    ... 
# } 
#
# ********** START of ConnectionManeuverAssist **********
#

bool                    queueLengthPresent
bool                    availableStorageLengthPresent
bool                    waitOnStopPresent
bool                    pedBicycleDetectPresent

LaneConnectionID        connectionID                  #  the common connectionID used by all lanes to which 
                                                      #  this data applies 
                                                      #  (this value traces to ConnectsTo entries in lanes)
ZoneLength              queueLength                   #  Unit = 1 meter, 0 = no queue
                                                      #  The distance from the stop line to the back 
                                                      #  edge of the last vehicle in the queue,         
                                                      #  as measured along the lane center line.
                                                      # 
                                                      #  Unit = 1 meter, 0 = unknown, 
                                                      #  The value 10000 to be used for Distances >=10000 m 
                                                      #  (e.g. from known point to another point along a  
                                                      #  known path, often against traffic flow direction  
                                                      #  when used for measuring queues) 
                                                      # OPTIONAL
ZoneLength              availableStorageLength        #  Unit = 1 meter, 0 = no space remains 
                                                      #  Distance (e.g. beginning from the downstream  
                                                      #  stop-line up to a given distance) with a high  
                                                      #  probability for successfully executing the  
                                                      #  connecting maneuver between the two lanes  
                                                      #  during the current cycle.  
                                                      #  Used for enhancing the awareness of vehicles  
                                                      #  to anticipate if they can pass the stop line 
                                                      #  of the lane. Used for optimizing the green wave, 
                                                      #  due to knowledge of vehicles waiting in front  
                                                      #  of a red light (downstream). 
                                                      #  The element nextTime in TimeChangeDetails 
                                                      #  in the containing data frame contains the next  
                                                      #  timemark at which an active phase is expected,  
                                                      #  a form of storage flush interval. 
                                                      # 
                                                      #  Unit = 1 meter, 0 = unknown, 
                                                      #  The value 10000 to be used for Distances >=10000 m 
                                                      #  (e.g. from known point to another point along a  
                                                      #  known path, often against traffic flow direction  
                                                      #  when used for measuring queues) 
                                                      # OPTIONAL
WaitOnStopline          waitOnStop                    #  If "true", the vehicles on this specific connecting 
                                                      #  maneuver have to stop on the stop-line and not 
                                                      #  to enter the collision area 
                                                      # OPTIONAL
PedestrianBicycleDetect pedBicycleDetect              #  true if ANY ped or bicycles are detected crossing 
                                                      #  the above lanes. Set to false ONLY if there is a 
                                                      #  high certainty that there are none present, 
                                                      #  otherwise element is not sent.
                                                      # OPTIONAL

# ********** END of ConnectionManeuverAssist   **********
