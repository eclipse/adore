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
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    stationarySince StationarySince OPTIONAL,
#    stationaryCause CauseCode OPTIONAL,
#    carryingDangerousGoods DangerousGoodsExtended OPTIONAL,
#    numberOfOccupants NumberOfOccupants OPTIONAL,
#    vehicleIdentification VehicleIdentification OPTIONAL,
#    energyStorageType EnergyStorageType OPTIONAL 
# } 
#
# ********** START of StationaryVehicleContainer **********
#

bool                                                  stationarySincePresent
bool                                                  stationaryCausePresent
bool                                                  carryingDangerousGoodsPresent
bool                                                  numberOfOccupantsPresent
bool                                                  vehicleIdentificationPresent
bool                                                  energyStorageTypePresent

its_container_v2_its_container/StationarySince        stationarySince               # OPTIONAL
its_container_v2_its_container/CauseCode              stationaryCause               # OPTIONAL
its_container_v2_its_container/DangerousGoodsExtended carryingDangerousGoods        # OPTIONAL
its_container_v2_its_container/NumberOfOccupants      numberOfOccupants             # OPTIONAL
its_container_v2_its_container/VehicleIdentification  vehicleIdentification         # OPTIONAL
its_container_v2_its_container/EnergyStorageType      energyStorageType             # OPTIONAL

# ********** END of StationaryVehicleContainer   **********
