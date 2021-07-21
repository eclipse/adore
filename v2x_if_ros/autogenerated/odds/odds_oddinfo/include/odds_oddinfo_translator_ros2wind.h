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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:odds:12.0
 * 
 * Module: ODDInfo {}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef ODDS_ODDINFO_TRANSLATOR_ROS2WIND
#define ODDS_ODDINFO_TRANSLATOR_ROS2WIND

#include <wind_constants.h>
#include <ros/ros.h>
#include <odds_oddinfo/ODDMSG.h>         // ROS
#include <odds_oddinfo.h>

namespace wind
{
    namespace wind_ros
    {
        void ros2wind(const odds_oddinfo::ODDMSG::ConstPtr& ros, wind::cpp::ODDInfo::ODDMSG* wind);
    }
}

#endif //ODDS_ODDINFO_TRANSLATOR_ROS2WIND
