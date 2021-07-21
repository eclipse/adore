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
#    utcTime DDateTime OPTIONAL ,
#    long_ Longitude,
#    lat Latitude,
#    elevation Elevation OPTIONAL,
#    heading HeadingDSRC OPTIONAL,
#    speed TransmissionAndSpeed OPTIONAL,
#    posAccuracy PositionalAccuracy OPTIONAL,
#    timeConfidence TimeConfidence OPTIONAL,
#    posConfidence PositionConfidenceSet OPTIONAL,
#    speedConfidence SpeedandHeadingandThrottleConfidence OPTIONAL,
#    ... 
# } 
#
# ********** START of FullPositionVector **********
#

bool                                     utcTimePresent
bool                                     elevationPresent
bool                                     headingPresent
bool                                     speedPresent
bool                                     posAccuracyPresent
bool                                     timeConfidencePresent
bool                                     posConfidencePresent
bool                                     speedConfidencePresent

DDateTime                                utcTime                #  time with mSec precision
                                                                # OPTIONAL
its_container_v2_its_container/Longitude long_
its_container_v2_its_container/Latitude  lat
Elevation                                elevation              # 
                                                                #  In units of 1 m steps above or below the reference ellipsoid  
                                                                #  Providing a range of -409.5 to + 6143.9 meters  
                                                                #  The value -409.6 shall be used when Unknown is to be sent 
                                                                # OPTIONAL
HeadingDSRC                              heading                # OPTIONAL
TransmissionAndSpeed                     speed                  # OPTIONAL
PositionalAccuracy                       posAccuracy            # OPTIONAL
TimeConfidence                           timeConfidence         # OPTIONAL
PositionConfidenceSet                    posConfidence          # OPTIONAL
SpeedandHeadingandThrottleConfidence     speedConfidence        # OPTIONAL

# ********** END of FullPositionVector   **********
