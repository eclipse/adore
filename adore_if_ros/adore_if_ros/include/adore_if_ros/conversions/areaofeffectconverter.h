/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore_if_ros_msg/AreaOfEffect.h>
#include <adore/env/situation/areaofeffect.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         *  Conversion between adore::env::AreaOfEffect and adore_if_ros_msg::AreaOfEffect.
         */
        struct AreaOfEffectConverter
        {
             /**
             * Conversion of adore::env::AreaOfEffect to adore_if_ros_msg::AreaOfEffect
             */
            adore_if_ros_msg::AreaOfEffect operator()(const adore::env::AreaOfEffect & aoe)
            {
                adore_if_ros_msg::AreaOfEffect msg;
                for(auto point:aoe)
                {
                    msg.X.push_back(point.first);
                    msg.Y.push_back(point.second);
                }
                return msg;
            }
       };
    }
}
