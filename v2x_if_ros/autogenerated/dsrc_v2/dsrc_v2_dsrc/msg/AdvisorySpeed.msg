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
#    type AdvisorySpeedType ,
#    speed SpeedAdvice OPTIONAL ,
#    confidence SpeedConfidenceDSRC OPTIONAL ,
#    distance ZoneLength OPTIONAL ,
#    class_ RestrictionClassID OPTIONAL ,
#    ... 
# } 
#
# ********** START of AdvisorySpeed **********
#

bool                speedPresent
bool                confidencePresent
bool                distancePresent
bool                class_Present

AdvisorySpeedType   type              #  the type of advisory which this is.
SpeedAdvice         speed             #  See Section 11 for converting and translating speed 
                                      #  expressed in mph into units of m/s 
                                      #  This element is optional ONLY when superceded 
                                      #  by the presence of a regional speed element found in 
                                      #  Reg-AdvisorySpeed entry 
                                      # 
                                      # LSB units are 0.1 m/s^2
                                      # the value 499 shall be used for values at or greater than 49.9 m/s
                                      # the value 500 shall be used to indicate that speed is unavailable
                                      # OPTIONAL
SpeedConfidenceDSRC confidence        #  A confidence value for the above speed
                                      # OPTIONAL
ZoneLength          distance          #  Unit = 1 meter, 
                                      #  The distance indicates the region for which the advised speed 
                                      #  is recommended, it is specified upstream from the stop bar  
                                      #  along the connected egressing lane 
                                      # 
                                      #  Unit = 1 meter, 0 = unknown, 
                                      #  The value 10000 to be used for Distances >=10000 m 
                                      #  (e.g. from known point to another point along a  
                                      #  known path, often against traffic flow direction  
                                      #  when used for measuring queues) 
                                      # OPTIONAL
RestrictionClassID  class_            #  the vehicle types to which it applies 
                                      #  when absent, the AdvisorySpeed applies to 
                                      #  all motor vehicle types 
                                      # OPTIONAL

# ********** END of AdvisorySpeed   **********
