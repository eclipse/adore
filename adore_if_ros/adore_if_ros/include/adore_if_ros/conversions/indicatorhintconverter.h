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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/
#pragma once

#include <adore/env/afactory.h>
#include <adore_if_ros_msg/IndicatorHint.h>

namespace adore
{
    namespace if_ROS
    {
        struct IndicatorHintConverter
        {
            /**
             * Conversion of adore::env::TIndicatorHint to adore_if_ros_msg::IndicatorHint message
             */
            adore_if_ros_msg::IndicatorHint operator()(const adore::env::IndicatorHint& data)
            {
                adore_if_ros_msg::IndicatorHint msg;
                msg.IndicatorHint  = adore::env::IndicatorHint::stringFromIndicatorSide(data.value);
                msg.validFromTo.first.x = data.startX;
                msg.validFromTo.first.y = data.startY;
                msg.validFromTo.first.z = 0.0;
                msg.validFromTo.last.x  = data.stopX;
                msg.validFromTo.last.y  = data.stopY;
                msg.validFromTo.last.z  = 0.0;
                msg.id                  = data.id;
                return msg;
            }
            /**
             * Conversion of adore_if_ros_msg::IndicatorHint message to adore::env::IndicatorHint*
             */
            void operator()(adore_if_ros_msg::IndicatorHintConstPtr msg,adore::env::IndicatorHint& data)
            {
                data.value  = adore::env::IndicatorHint::indicatorSideFromString(msg->IndicatorHint);
                data.startX = msg->validFromTo.first.x;
                data.startY = msg->validFromTo.first.y;
                data.stopX  = msg->validFromTo.last.x;
                data.stopY  = msg->validFromTo.last.y;
                data.id     = msg->id;                 
            }

        };
    }
}



