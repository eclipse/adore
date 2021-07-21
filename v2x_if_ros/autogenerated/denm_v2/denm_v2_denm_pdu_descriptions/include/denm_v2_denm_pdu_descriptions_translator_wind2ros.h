/*
 *
 * Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)
 * Institute of Transportation Systems. (https://www.dlr.de/ts/)
 *
 * 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 * 
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 * 
 * SPDX-License-Identifier: EPL-2.0
 * 
 *
 * 
 * File automatically generated with DLR Wind v2 (2021)
 * 
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:denm_v2:1.5
 * 
 * Module: DENM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) denm(1) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef DENM_V2_DENM_PDU_DESCRIPTIONS_TRANSLATOR_WIND2ROS
#define DENM_V2_DENM_PDU_DESCRIPTIONS_TRANSLATOR_WIND2ROS

#include <wind_constants.h>
#include <ros/ros.h>
#include <denm_v2_denm_pdu_descriptions/DENM.h>         // ROS
#include <denm_v2_denm_pdu_descriptions.h>

namespace wind
{
    namespace wind_ros
    {
        void wind2ros(denm_v2_denm_pdu_descriptions::DENM* ros, wind::cpp::DENM_PDU_Descriptions::DENM* wind);
    }
}

#endif //DENM_V2_DENM_PDU_DESCRIPTIONS_TRANSLATOR_WIND2ROS
