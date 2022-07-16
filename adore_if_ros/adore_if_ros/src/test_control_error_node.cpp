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
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <adore_if_ros_msg/Proposition.h>

class ControlErrorNode
{
    public:
    bool terminated_;
    int mode_;
    double time_;
    double activation_time_;
    bool error_mode_;
    ros::Subscriber subscriber_odom_;
    ros::Subscriber subscriber_steering_;
    ros::Subscriber subscriber_acceleration_;
    ros::Publisher publisher_steering_;
    ros::Publisher publisher_acceleration_;
    ros::Publisher publisher_propositions_;
    ControlErrorNode()
    : terminated_(false),mode_(0),error_mode_(false),activation_time_(0)
    {
    }
    void init()
    {
        ros::NodeHandle n;    
        subscriber_odom_ = n.subscribe("odom",1,&ControlErrorNode::receive_odom,this);
        subscriber_steering_ = n.subscribe("original_steeringAngle",1,&ControlErrorNode::receive_steering,this);
        subscriber_acceleration_ = n.subscribe("original_acceleration",1,&ControlErrorNode::receive_acceleration,this);
        publisher_steering_ = n.advertise<std_msgs::Float32>("FUN/MotionCommand/steeringAngle",1);
        publisher_acceleration_ = n.advertise<std_msgs::Float32>("FUN/MotionCommand/acceleration",1);
        publisher_propositions_ = n.advertise<adore_if_ros_msg::Proposition>("ENV/propositions",1);
    }
    void receive_steering(std_msgs::Float32ConstPtr msg)
    {
        //pass through steering in mode 0
        if(mode_==0)publisher_steering_.publish(*msg);
    }
    void receive_acceleration(std_msgs::Float32ConstPtr msg)
    {
        //pass through acceleration in mode 0
        if(mode_==0)publisher_acceleration_.publish(*msg);
    }
    void receive_odom(nav_msgs::OdometryConstPtr msg)
    {
        double error_timout = 5;
        double max_angle = 450;
        ros::param::getCached("test_control_error_node/error_timeout",error_timout);
        ros::param::getCached("test_control_error_node/max_angle",max_angle);
        time_ = msg->header.stamp.toSec();
        if(error_mode_ && (mode_==0 || time_>activation_time_+error_timout))
        {
            error_mode_ = false;
            mode_ = 0;
            std::cout<<"switching back to 0"<<std::endl;
        }
        else if(mode_>0 && !error_mode_)
        {
           error_mode_ = true;
           activation_time_ = time_;
        }
        adore_if_ros_msg::Proposition proposition;
        proposition.has_timeout=false;
        proposition.order=0;
        proposition.term="CONTROL_ERROR";
        proposition.value=mode_;
        publisher_propositions_.publish(proposition);
        switch(mode_)
        {
            case 0:
            break;
            case 1:
            {
                std_msgs::Float32 steering;
                steering.data = max_angle;
                std_msgs::Float32 acceleration;
                acceleration.data = 0;
                publisher_steering_.publish(steering);
                publisher_acceleration_.publish(acceleration);
            }
            break;
            case 2:
            {
                std_msgs::Float32 steering;
                steering.data = 0;
                std_msgs::Float32 acceleration;
                acceleration.data = 0;
                publisher_steering_.publish(steering);
                publisher_acceleration_.publish(acceleration);
            }
            break;
            case 3:
            {
                std_msgs::Float32 steering;
                steering.data = -max_angle;
                std_msgs::Float32 acceleration;
                acceleration.data = 0;
                publisher_steering_.publish(steering);
                publisher_acceleration_.publish(acceleration);
            }
            break;
        }
    }



};

ControlErrorNode node;

void kbinput()
{
  while(!node.terminated_)
  {
    std::cout<<"Change control error: \n0 no error, 1 steering left, 2 steering straight, 3 steering right.\n";
    int i = std::cin.get();
    if(i<48||i>51)continue;
    node.mode_  = i-48;
    std::cout<<"mode="<<node.mode_<<std::endl;
  }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"test_control_error_node");
    std::thread kbinput_thread(kbinput);
    node.init();
    ros::spin();
    node.terminated_=true;
    return 0;
}
