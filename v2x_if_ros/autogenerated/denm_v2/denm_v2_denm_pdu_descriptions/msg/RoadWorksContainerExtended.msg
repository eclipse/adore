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
#    lightBarSirenInUse LightBarSirenInUse OPTIONAL,
#    closedLanes ClosedLanes OPTIONAL,
#    restriction RestrictedTypes OPTIONAL,
#    speedLimit SpeedLimit OPTIONAL,
#    incidentIndication CauseCode OPTIONAL,
#    recommendedPath ItineraryPath OPTIONAL,
#    startingPointSpeedLimit DeltaReferencePosition OPTIONAL,
#    trafficFlowRule TrafficRule OPTIONAL,
#    referenceDenms ReferenceDenms OPTIONAL 
# } 
#
# ********** START of RoadWorksContainerExtended **********
#

bool                                                  lightBarSirenInUsePresent
bool                                                  closedLanesPresent
bool                                                  restrictionPresent
bool                                                  speedLimitPresent
bool                                                  incidentIndicationPresent
bool                                                  recommendedPathPresent
bool                                                  startingPointSpeedLimitPresent
bool                                                  trafficFlowRulePresent
bool                                                  referenceDenmsPresent

its_container_v2_its_container/LightBarSirenInUse     lightBarSirenInUse             # OPTIONAL
its_container_v2_its_container/ClosedLanes            closedLanes                    # OPTIONAL
its_container_v2_its_container/RestrictedTypes        restriction                    # OPTIONAL
its_container_v2_its_container/SpeedLimit             speedLimit                     # OPTIONAL
its_container_v2_its_container/CauseCode              incidentIndication             # OPTIONAL
its_container_v2_its_container/ItineraryPath          recommendedPath                # OPTIONAL
its_container_v2_its_container/DeltaReferencePosition startingPointSpeedLimit        # OPTIONAL
its_container_v2_its_container/TrafficRule            trafficFlowRule                # OPTIONAL
ReferenceDenms                                        referenceDenms                 # OPTIONAL

# ********** END of RoadWorksContainerExtended   **********
