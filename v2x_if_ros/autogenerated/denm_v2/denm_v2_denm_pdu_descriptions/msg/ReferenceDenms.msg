#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)                *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)                            *
#*                                                                                          *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore                 *
#*                                                                                          *
#* This program and the accompanying materials are made available under the                 *
#* terms of the Eclipse Public License 2.0 which is available at                            *
#* http://www.eclipse.org/legal/epl-2.0.                                                    *
#*                                                                                          *
#* SPDX-License-Identifier: EPL-2.0                                                         *
#*                                                                                          *
#*                                                                                          *
#* File automatically generated with DLR Wind v2 (2021)                                     *
#*                                                                                          *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:denm_v2:1.5                      *
#*                                                                                          *
#* Module: DENM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) en(302637) denm(1) version(2)}                                                *
#*                                                                                          *
#* For support contact v2x-ts@dlr.de                                                        *
#*                                                                                          *

#
# Element type: SEQUENCE OF
# 
# ASN.1 Definition:
# SEQUENCE(SIZE(1..8, ...)) OF ActionID 
#
# ********** START of ReferenceDenms **********
#

int8                                      count           # Use this field to indicate how many valid elements the array has

its_container_v2_its_container/ActionID[] elements        # Range(1, 8)

int8                                      ARRAY_SIZE      = 8

# ********** END of ReferenceDenms   **********
