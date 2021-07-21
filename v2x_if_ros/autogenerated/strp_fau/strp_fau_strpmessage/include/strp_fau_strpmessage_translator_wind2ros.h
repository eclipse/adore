/*
 *
 * Copyright (C) 2017-2020 German Aerospace Center e.V. (https://www.dlr.de)
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
 * File automatically generated with DLR Wind v2 (30.11.2020 11:00:04)
 * 
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:strp_fau:1.0
 * 
 * Module: STRPMessage {}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef STRP_FAU_STRPMESSAGE_TRANSLATOR_WIND2ROS
#define STRP_FAU_STRPMESSAGE_TRANSLATOR_WIND2ROS

#include <ros/ros.h>
#include <strp_fau_strpmessage/SpaceTimeReservationProcedure.h>         // ROS
#include <strp_fau_strpmessage.h>

namespace wind
{
    namespace wind_ros
    {
        void wind2ros(strp_fau_strpmessage::SpaceTimeReservationProcedure* ros, wind::cpp::STRPMessage::SpaceTimeReservationProcedure* wind);
    }
}

#endif //STRP_FAU_STRPMESSAGE_TRANSLATOR_WIND2ROS
