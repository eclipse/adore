#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)        *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                    *
#*                                                                                  *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore         *
#*                                                                                  *
#* This program and the accompanying materials are made available under the         *
#* terms of the Eclipse Public License 2.0 which is available at                    *
#* http://www.eclipse.org/legal/epl-2.0.                                            *
#*                                                                                  *
#* SPDX-License-Identifier: EPL-2.0                                                 *
#*                                                                                  *
#*                                                                                  *
#* File automatically generated with DLR Wind v2 (2021)                             *
#*                                                                                  *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v2:1.4     *
#*                                                                                  *
#* Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(102894) cdd(2) version(2)}                                         *
#*                                                                                  *
#* For support contact v2x-ts@dlr.de                                                *
#*                                                                                  *

#
# Element type: NUMERICAL
# 
# ASN.1 Definition:
# INTEGER { denm(1), cam(2), poi(3), spatem(4), mapem(5), ivim(6), ev_rsr(7), tistpgtransaction(8), srem(9), ssem(10), evcsn(11), saem(12), rtcmem(13), mcm(33) }(0..255), 
#
# ********** START of ItsPduHeader_messageID **********
#

uint8           value

uint8           MIN               = 0
uint8           MAX               = 255

uint8           DENM              = 1
uint8           CAM               = 2
uint8           POI               = 3
uint8           SPATEM            = 4
uint8           MAPEM             = 5
uint8           IVIM              = 6
uint8           EV__RSR           = 7
uint8           TISTPGTRANSACTION = 8
uint8           SREM              = 9
uint8           SSEM              = 10
uint8           EVCSN             = 11
uint8           SAEM              = 12
uint8           RTCMEM            = 13
uint8           MCM               = 33

# ********** END of ItsPduHeader_messageID   **********
