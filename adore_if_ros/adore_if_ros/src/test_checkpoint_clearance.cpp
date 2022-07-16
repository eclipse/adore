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
#include <std_msgs/Bool.h>

class Clearance
{
    private:
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    ros::NodeHandle n_;
    public:
    void receive(nav_msgs::OdometryConstPtr msg)
    {
        double speed = msg->twist.twist.linear.x;
        std_msgs::Bool out_msg;
        out_msg.data = true;
        if(speed<1.0)publisher_.publish(out_msg);
    }
    
    Clearance()
    {
        subscriber_ = n_.subscribe("localization",10,&Clearance::receive,this);
        publisher_ = n_.advertise<std_msgs::Bool>("VEH/Checkpoints/clearance",1);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_checkpoint_clearance");
    Clearance clearance;
    ros::spin();

    return 0;
}