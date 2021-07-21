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
#    s1 DistanceAlongPath  
# }  
#
# ********** START of FixedBlock **********
#

TimeOffset        t0              # earliest penetration of critical area, time relative to generationTime
TimeOffset        t1              # latest penetration of critical area to activate the maneuver, time relative to generationTime
DistanceAlongPath s0              # nearest position where critical area may be entered at t>=t0
DistanceAlongPath s1              # furthest position until which the critical area has to be exited => extend of reservation is s1-s0
                                  # removed speed interval: a minimum speed does not guarantee exiting the area before certain time

# ********** END of FixedBlock   **********
