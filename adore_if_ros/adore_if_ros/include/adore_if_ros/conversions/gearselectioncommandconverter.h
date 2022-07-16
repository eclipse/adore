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
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/

#pragma once

#include <adore/fun/afactory.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/NavigationGoal.h>
#include <adore_if_ros_msg/SetPointRequest.h>
#include <adore_if_ros_msg/TerminalRequest.h>
#include <adore_if_ros_msg/WheelSpeed.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * Converts between adore::fun::GearSelectionCommand and ROS std_msgs::Int8
         */
        struct GearSelectionCommandConverter
        {

            /**
             * Converts adore::fun::GearSelectionCommand to ROS std_msgs::Int8
             */
            std_msgs::Int8 operator()(const adore::fun::GearSelectionCommand& cmd)
            {
                std_msgs::Int8 msg;
                switch(cmd.getGear())
                {
                    case adore::fun::VehicleExtendedState::Park:
                        msg.data = 0;
                        break;
                    case adore::fun::VehicleExtendedState::Drive:
                        msg.data = 1;
                        break;
                    case adore::fun::VehicleExtendedState::Reverse:
                        msg.data = 2;
                        break;
                    case adore::fun::VehicleExtendedState::Neutral:
                        msg.data = 3;
                        break;
                    default:
                        msg.data = 0;
                        break;
                }
                return msg;
            }        
            
            /**
             * Converts ROS std_msgs::Int8 to adore::fun::GearSelectionCommand
             */
            void operator()(std_msgs::Int8ConstPtr msg,adore::fun::GearSelectionCommand* cmd)
            {
                switch(msg.get()->data)
                {
                    case 0:
                        cmd->setGear(adore::fun::VehicleExtendedState::Park);
                        break;
                    case 1:
                        cmd->setGear(adore::fun::VehicleExtendedState::Drive);
                        break;
                    case 2:
                        cmd->setGear(adore::fun::VehicleExtendedState::Reverse);
                        break;
                    case 3:
                        cmd->setGear(adore::fun::VehicleExtendedState::Neutral);
                        break;
                    default:
                        cmd->setGear(adore::fun::VehicleExtendedState::Park);
                        break;
                }
            }

        };
    }
}