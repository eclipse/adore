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
#    generationTime TimestampIts ,
#    timeout TimeOffset ,
#    requestID RequestID ,
#    marker0 ReferencePosition ,
#    marker1 RelativePosition ,
#    reservation ReservationShape  
# }  
#
# ********** START of Request **********
#

its_container_v1_its_container/TimestampIts generationTime  # reservation times are relative to this generationTime
TimeOffset                                  timeout         # do not reply to this message after generationTime+timeout
RequestID                                   requestID       # id managed by requesting vehicle
ReferencePosition                           marker0         # beginning of path
RelativePosition                            marker1         # end of path, position relative to marker0
ReservationShape                            reservation     # the reserved space-time-set in path coordinates

# ********** END of Request   **********
