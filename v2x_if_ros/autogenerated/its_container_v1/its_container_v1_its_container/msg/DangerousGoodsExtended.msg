#* Copyright (C) 2017-2020 German Aerospace Center e.V. (https://www.dlr.de)        *
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
#* File automatically generated with DLR Wind (07.10.2020 16:24:31)                 *
#*                                                                                  *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:its_container_v1:1.2     *
#*                                                                                  *
#* Module: ITS_Container {itu-t(0) identified-organization(4) etsi(0) itsDomain(5)  *
#*     wg1(1) ts(102894) cdd(2) version(1)}                                         *
#*                                                                                  *
#* For support contact v2x-ts@dlr.de                                                *
#*                                                                                  *

#
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    dangerousGoodsType DangerousGoodsBasic,
#    unNumber INTEGER(0..9999),
#    elevatedTemperature BOOLEAN,
#    tunnelsRestricted BOOLEAN,
#    limitedQuantity BOOLEAN,
#    emergencyActionCode IA5String(SIZE(1..24)) OPTIONAL,
#    phoneNumber IA5String(SIZE(1..24)) OPTIONAL,
#    companyName UTF8String(SIZE(1..24)) OPTIONAL 
# } 
#
# ********** START of DangerousGoodsExtended **********
#

bool                                       emergencyActionCodePresent
bool                                       phoneNumberPresent
bool                                       companyNamePresent

DangerousGoodsBasic                        dangerousGoodsType
DangerousGoodsExtended_unNumber            unNumber
DangerousGoodsExtended_elevatedTemperature elevatedTemperature
DangerousGoodsExtended_tunnelsRestricted   tunnelsRestricted
DangerousGoodsExtended_limitedQuantity     limitedQuantity
DangerousGoodsExtended_emergencyActionCode emergencyActionCode        # OPTIONAL
DangerousGoodsExtended_phoneNumber         phoneNumber                # OPTIONAL
DangerousGoodsExtended_companyName         companyName                # OPTIONAL

# ********** END of DangerousGoodsExtended   **********
