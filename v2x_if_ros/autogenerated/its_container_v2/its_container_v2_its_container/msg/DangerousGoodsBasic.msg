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
# Element type: ENUMERATED
# 
# ASN.1 Definition:
# ENUMERATED { explosives1(0), explosives2(1), explosives3(2), explosives4(3), explosives5(4), explosives6(5), flammableGases(6), nonFlammableGases(7), toxicGases(8), flammableLiquids(9), flammableSolids(10), substancesLiableToSpontaneousCombustion(11), substancesEmittingFlammableGasesUponContactWithWater(12), oxidizingSubstances(13), organicPeroxides(14), toxicSubstances(15), infectiousSubstances(16), radioactiveMaterial(17), corrosiveSubstances(18), miscellaneousDangerousSubstances(19) } 
#
# ********** START of DangerousGoodsBasic **********
#

int8            value

int8            MIN                                                         = 0
int8            MAX                                                         = 19

int8            EXPLOSIVES_1                                                = 0
int8            EXPLOSIVES_2                                                = 1
int8            EXPLOSIVES_3                                                = 2
int8            EXPLOSIVES_4                                                = 3
int8            EXPLOSIVES_5                                                = 4
int8            EXPLOSIVES_6                                                = 5
int8            FLAMMABLE_GASES                                             = 6
int8            NON_FLAMMABLE_GASES                                         = 7
int8            TOXIC_GASES                                                 = 8
int8            FLAMMABLE_LIQUIDS                                           = 9
int8            FLAMMABLE_SOLIDS                                            = 10
int8            SUBSTANCES_LIABLE_TO_SPONTANEOUS_COMBUSTION                 = 11
int8            SUBSTANCES_EMITTING_FLAMMABLE_GASES_UPON_CONTACT_WITH_WATER = 12
int8            OXIDIZING_SUBSTANCES                                        = 13
int8            ORGANIC_PEROXIDES                                           = 14
int8            TOXIC_SUBSTANCES                                            = 15
int8            INFECTIOUS_SUBSTANCES                                       = 16
int8            RADIOACTIVE_MATERIAL                                        = 17
int8            CORROSIVE_SUBSTANCES                                        = 18
int8            MISCELLANEOUS_DANGEROUS_SUBSTANCES                          = 19

# ********** END of DangerousGoodsBasic   **********
