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
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/

#include <adore/sim/stdstate.h>
#include <adore_if_ros_msg/StdState.h>
namespace adore
{
    namespace if_ROS
    {
       
        struct StdStateConverter
        {
            void operator()(adore_if_ros_msg::StdStateConstPtr msg, adore::sim::StdState& ser)
            {
                for(auto& it : msg.get()->values)
                {
                    ser.values_.push_back(it);
                }            
            }     
             void operator()(const adore_if_ros_msg::StdState& ser, adore::sim::StdState& object)
            {
                for(auto& it : ser.values)
                {
                    object.values_.push_back(*(&it));
                }
            }
            adore_if_ros_msg::StdState operator()(const adore::sim::StdState& ser)
            {
                adore_if_ros_msg::StdState msg;
                for(auto& it : ser.values_)
                {
                    msg.values.push_back(*(&it));
                }
                return msg;
            }
        };
    }
}