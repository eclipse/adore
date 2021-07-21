#* Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de) *
#* Institute of Transportation Systems. (https://www.dlr.de/ts/)             *
#*                                                                           *
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore  *
#*                                                                           *
#* This program and the accompanying materials are made available under the  *
#* terms of the Eclipse Public License 2.0 which is available at             *
#* http://www.eclipse.org/legal/epl-2.0.                                     *
#*                                                                           *
#* SPDX-License-Identifier: EPL-2.0                                          *
#*                                                                           *
#*                                                                           *
#* File automatically generated with DLR Wind v2 (2021)                      *
#*                                                                           *
#* This file belongs to the DLR Wind project de.dlr.ts.v2x:odds:12.0         *
#*                                                                           *
#* Module: ODDInfo {}                                                        *
#*                                                                           *
#* For support contact v2x-ts@dlr.de                                         *
#*                                                                           *

#
# Element type: SEQUENCE
# 
# ASN.1 Definition:
# SEQUENCE {
#    sectionType Section,
#    id SectionID,
#    geoInfo GeoInformation,
#    validityInfo ValidityInformation,
#    staticInf StaticInfrastructure,
#    opConditions OperationalConditions OPTIONAL,
#    envConditions EnvironmentalConditions OPTIONAL,
#    ... 
# } 
#
# ********** START of SectionObject **********
#

bool                    opConditionsPresent
bool                    envConditionsPresent

Section                 sectionType
SectionID               id
GeoInformation          geoInfo
ValidityInformation     validityInfo
StaticInfrastructure    staticInf
OperationalConditions   opConditions         # OPTIONAL
EnvironmentalConditions envConditions        # OPTIONAL

# ********** END of SectionObject   **********
