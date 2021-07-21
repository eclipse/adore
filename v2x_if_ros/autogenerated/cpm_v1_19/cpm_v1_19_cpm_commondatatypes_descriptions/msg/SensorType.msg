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
# INTEGER { undefined(0), radar(1), lidar(2), monovideo(3), stereovision(4), nightvision(5), ultrasonic(6), pmd(7), fusion(8), inductionloop(9), sphericalCamera(10), itssaggregation(11) }(0..15) 
#
# ********** START of SensorType **********
#

int8            value

int8            MIN              = 0
int8            MAX              = 15

int8            UNDEFINED        = 0
int8            RADAR            = 1
int8            LIDAR            = 2
int8            MONOVIDEO        = 3
int8            STEREOVISION     = 4
int8            NIGHTVISION      = 5
int8            ULTRASONIC       = 6
int8            PMD              = 7
int8            FUSION           = 8
int8            INDUCTIONLOOP    = 9
int8            SPHERICAL_CAMERA = 10
int8            ITSSAGGREGATION  = 11

# ********** END of SensorType   **********
