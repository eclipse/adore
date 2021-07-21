/****************************************************************************/
// Eclipse ADORe, Automated Driving Open Research; see https://eclipse.org/adore
// Copyright (C) 2017-2020 German Aerospace Center (DLR).
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    station.h
/// @author  Daniel Heß
/// @contact Daniel.Hess@DLR.de
/// @date 2020/04/02
/// @initialrelease X.X.X
///
// Station class tracks station position and time.
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>

namespace v2xsim
{
    /**
     * Station tracks position and time of a v2x sender/receiver.
     */
    class Station
    {
    private:
        double X_;
        double Y_;
        double Z_;
        double t_;
        ros::Subscriber odom_subscriber_;
        void odom_receive(nav_msgs::OdometryConstPtr msg)
        {
            X_ = msg->pose.pose.position.x;
            Y_ = msg->pose.pose.position.y;
            Z_ = msg->pose.pose.position.z;
            t_ = msg->header.stamp.toSec();
        }
    public:
        Station(ros::NodeHandle n,std::string odom_topic)
        {
            if(odom_topic!="")odom_subscriber_ = n.subscribe(odom_topic,10,&Station::odom_receive,this);
        }
        void setPosition(double X,double Y,double Z)
        {
            X_=X;Y_=Y;Z_=Z;
        }
        void setTime(double t)
        {
            t_=t;
        }
        double getX() const{return X_;}
        double getY() const{return Y_;}
        double getZ() const{return Z_;}
        double getT() const{return t_;}
    };
}