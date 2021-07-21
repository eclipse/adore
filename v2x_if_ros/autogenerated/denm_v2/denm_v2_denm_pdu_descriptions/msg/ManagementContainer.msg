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
#    actionID ActionID,
#    detectionTime TimestampIts,
#    referenceTime TimestampIts,
#    termination Termination OPTIONAL,
#    eventPosition ReferencePosition,
#    relevanceDistance RelevanceDistance OPTIONAL,
#    relevanceTrafficDirection RelevanceTrafficDirection OPTIONAL,
#    validityDuration ValidityDuration,
#    transmissionInterval TransmissionInterval OPTIONAL,
#    stationType StationType,
#    ... 
# } 
#
# ********** START of ManagementContainer **********
#

bool                                                     terminationPresent
bool                                                     relevanceDistancePresent
bool                                                     relevanceTrafficDirectionPresent
bool                                                     transmissionIntervalPresent

its_container_v2_its_container/ActionID                  actionID
its_container_v2_its_container/TimestampIts              detectionTime
its_container_v2_its_container/TimestampIts              referenceTime
Termination                                              termination                      # OPTIONAL
its_container_v2_its_container/ReferencePosition         eventPosition
its_container_v2_its_container/RelevanceDistance         relevanceDistance                # OPTIONAL
its_container_v2_its_container/RelevanceTrafficDirection relevanceTrafficDirection        # OPTIONAL
its_container_v2_its_container/ValidityDuration          validityDuration
its_container_v2_its_container/TransmissionInterval      transmissionInterval             # OPTIONAL
its_container_v2_its_container/StationType               stationType

# ********** END of ManagementContainer   **********
