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
         * Converts between adore::fun::SetPointRequest and adore_if_ros_msg::SetPointRequest.
         */
        struct SetPointRequestConverter
        {
            /**
             *  convert a SetPointRequest ROS message into a SetPointRequest c++ object
             */
            void operator()(adore_if_ros_msg::SetPointRequestConstPtr msg, adore::fun::SetPointRequest* spr)
            {
                spr->setPointRequestID = msg.get()->requestId;
                spr->setPoints.clear();
                for(auto& it : msg.get()->setPoints)
                {   
                    adore::fun::SetPoint sp;
                    sp.maneuverID = it.maneuverId;
                    sp.tStart = it.tStart;
                    sp.tEnd = it.tEnd;
                    sp.x0ref.setX(it.X);
                    sp.x0ref.setY(it.Y);
                    sp.x0ref.setPSI(it.PSI);
                    sp.x0ref.setvx(it.vx);
                    sp.x0ref.setvy(it.vy);
                    sp.x0ref.setOmega(it.omega);
                    sp.x0ref.setAx(it.ax);
                    sp.x0ref.setDelta(it.delta);
                    sp.x0ref.setDAx(it.dax);
                    sp.x0ref.setDDelta(it.ddelta);
                    spr->setPoints.push_back(sp);
                }
            }
            /**
             * convert a single SetPoint ROS->c++
             */
            void operator()(adore_if_ros_msg::SetPointConstPtr msg,adore::fun::SetPoint* sp)
            {
                sp->maneuverID = msg.get()->maneuverId;
                sp->tStart = msg.get()->tStart;
                sp->tEnd = msg.get()->tEnd;
                sp->x0ref.setX(msg.get()->X);
                sp->x0ref.setY(msg.get()->Y);
                sp->x0ref.setPSI(msg.get()->PSI);
                sp->x0ref.setvx(msg.get()->vx);
                sp->x0ref.setvy(msg.get()->vy);
                sp->x0ref.setOmega(msg.get()->omega);
                sp->x0ref.setAx(msg.get()->ax);
                sp->x0ref.setDelta(msg.get()->delta);
                sp->x0ref.setDAx(msg.get()->dax);
                sp->x0ref.setDDelta(msg.get()->ddelta);
            }
            /**
             * convert a single SetPoint c++->ROS
             */
            adore_if_ros_msg::SetPoint operator()(const adore::fun::SetPoint* sp)
            {
                adore_if_ros_msg::SetPoint msg;
                msg.maneuverId = sp->maneuverID;
                msg.tStart = sp->tStart;
                msg.tEnd = sp->tEnd;
                msg.X = sp->x0ref.getX();
                msg.Y = sp->x0ref.getY();
                msg.PSI = sp->x0ref.getPSI();
                msg.vx = sp->x0ref.getvx();
                msg.vy = sp->x0ref.getvy();
                msg.omega = sp->x0ref.getOmega();
                msg.ax = sp->x0ref.getAx();
                msg.delta = sp->x0ref.getDelta();
                msg.dax = sp->x0ref.getDAx();
                msg.ddelta = sp->x0ref.getDDelta();
                return msg;
            }
            /**
             * convert a SetPointRequest c++ to ROS message
             */
            adore_if_ros_msg::SetPointRequest operator()(const adore::fun::SetPointRequest& spr)
            {
                adore_if_ros_msg::SetPointRequest msg;
                msg.requestId = spr.setPointRequestID;
                for(auto& it : spr.setPoints)
                {
                    msg.setPoints.push_back((*this)(&it));
                }
                return msg;
            }
        };
    }
}