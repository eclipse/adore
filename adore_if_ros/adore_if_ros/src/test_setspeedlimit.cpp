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
 *   Daniel Heß
 ********************************************************************************/


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class Setter
{
    private:
    ros::Subscriber subscriber_;
    ros::NodeHandle n_;
    public:
    void receive(nav_msgs::OdometryConstPtr msg)
    {
        double speedlimit = 50.0/3.6;
        double y = msg->pose.pose.position.y;
        if(5796000>y && y>5795569)
        {
            speedlimit = 30.0/3.6;
        }
        n_.setParam("PARAMS/tactical_planner/global_speed_limit",speedlimit);
    }
    
    Setter()
    {
        subscriber_ = n_.subscribe("localization",10,&Setter::receive,this);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_setspeedlimit");
    Setter setter;
    ros::spin();

    return 0;
}