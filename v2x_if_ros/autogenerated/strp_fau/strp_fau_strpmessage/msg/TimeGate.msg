#* Copyright (C) 2017-2020 German Aerospace Center e.V. (https://www.dlr.de) *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)             *
#*                                                                           *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore  *
#*                                                                           *
#* This program and the accompanying materials are made available under the  *
#* terms of the Eclipse Public License 2.0 which is available at             *
#* http://www.eclipse.org/legal/epl-2.0.                                     *
#*                                                                           *
#* SPDX-License-Identifier: EPL-2.0                                          *
#*                                                                           *
#*                                                                           *
#* File automatically generated with DLR Wind v2 (30.11.2020 11:00:04)       *
#*                                                                           *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:strp_fau:1.0      *
#*                                                                           *
#* Module: STRPMessage {}                                                    *
#*                                                                           *
#* For support contact v2x-ts@dlr.de                                         *
#*                                                                           *

#
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    t0 TimeOffset ,
#    t1 TimeOffset ,
#    s0 DistanceAlongPath ,
#    s1 DistanceAlongPath ,
#    vmin SpeedValue  
# }  
#
# ********** START of TimeGate **********
#

TimeOffset                                t0              # earliest penetration of critical area at s0 with foremost position of vehicle, time relative to generationTime
TimeOffset                                t1              # latest penetration of critical area to activate the maneuver, time relative to generationTime
DistanceAlongPath                         s0              # start of the critical area, relative to marker0
DistanceAlongPath                         s1              # end of the critical area, relative to marker0
its_container_v1_its_container/SpeedValue vmin            # lowest entrance speed at t1

# ********** END of TimeGate   **********
