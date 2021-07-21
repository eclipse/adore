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
# Element type: BIT STRING
# 
# ASN.1 Definition:
# BIT STRING { hydrogenStorage(0), electricEnergyStorage(1), liquidPropaneGas(2), compressedNaturalGas(3), diesel(4), gasoline(5), ammonia(6) }(SIZE(7)) 
#
# ********** START of EnergyStorageType **********
#

bool[]          values

uint8           ARRAY_SIZE              = 7

uint8           HYDROGEN_STORAGE        = 0
uint8           ELECTRIC_ENERGY_STORAGE = 1
uint8           LIQUID_PROPANE_GAS      = 2
uint8           COMPRESSED_NATURAL_GAS  = 3
uint8           DIESEL                  = 4
uint8           GASOLINE                = 5
uint8           AMMONIA                 = 6

# ********** END of EnergyStorageType   **********
