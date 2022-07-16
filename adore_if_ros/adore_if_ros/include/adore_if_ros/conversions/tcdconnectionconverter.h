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
#include <adore_if_ros_msg/TCDConnectionStateTrace.h>
#include <adore/env/tcd/controlledconnection.h>

namespace adore
{
    namespace if_ROS
    {
        struct TCDConnectionConverter
        {
            void operator()(adore_if_ros_msg::TCDConnectionStateTraceConstPtr msg , adore::env::ControlledConnection & con)
            {
                con.setID( 
                           msg.get()->connection.first.x, 
                           msg.get()->connection.first.y, 
                           msg.get()->connection.first.z, 
                           msg.get()->connection.last.x, 
                           msg.get()->connection.last.y, 
                           msg.get()->connection.last.z
                );
                for( auto state: msg.get()->data )
                {
                    adore::env::ConnectionStateEvent e(
                        (adore::env::ConnectionState::EConnectionState)(int)state.state,
                        state.minEndTime,
                        state.maxEndTime_present,
                        state.likelyTime_present,
                        state.maxEndTime,
                        state.likelyTime
                    );
                    con.insertStateEvent(e);
                }
            }
            /**
             * Conversion of adore::env::ControlledConnection to adore_if_ros_msg::TCDConnectionStateTrace message
             */
            adore_if_ros_msg::TCDConnectionStateTrace operator()(const adore::env::ControlledConnection & con)
            {
                adore_if_ros_msg::TCDConnectionStateTrace msg;
                msg.connection.first.x = con.getID().getFrom().get<0>();
                msg.connection.first.y = con.getID().getFrom().get<1>();
                msg.connection.first.z = con.getID().getFrom().get<2>();
                msg.connection.last.x = con.getID().getTo().get<0>();
                msg.connection.last.y = con.getID().getTo().get<1>();
                msg.connection.last.z = con.getID().getTo().get<2>();
                for( auto& state: con.data_ )
                {
                    adore_if_ros_msg::TCDConnectionState smsg;
                    smsg.minEndTime = state.getMinEndTime();
                    smsg.maxEndTime = state.getMaxEndTime();
                    smsg.maxEndTime_present = state.hasMaxEndTime();
                    smsg.likelyTime = state.getLikelyTime();
                    smsg.likelyTime_present = state.hasLikelyTime();
                    smsg.state = (int) state.getState();
                    msg.data.push_back(smsg);
                }
                return msg;
            }
        };
    }
}
