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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_transaid:3.0
 * 
 * Module: MCM_TransAID {version(3)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef MCM_TRANSAID_MCM_TRANSAID_TRANSLATOR_ROS2WIND
#define MCM_TRANSAID_MCM_TRANSAID_TRANSLATOR_ROS2WIND

#include <wind_constants.h>
#include <ros/ros.h>
#include <mcm_transaid_mcm_transaid/MCM.h>         // ROS
#include <mcm_transaid_mcm_transaid.h>

namespace wind
{
    namespace wind_ros
    {
        void ros2wind(const mcm_transaid_mcm_transaid::MCM::ConstPtr& ros, wind::cpp::MCM_TransAID::MCM* wind);
    }
}

#endif //MCM_TRANSAID_MCM_TRANSAID_TRANSLATOR_ROS2WIND
