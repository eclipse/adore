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
         * ROS specific implementation of AWriter for adore::fun::IndicatorCommand.
         * Transmits adore::fun::IndicatorCommand by sending two ROS std_msgs::Bool.
         */
        class IndicatorCommandWriter:public adore::mad::AWriter<adore::fun::IndicatorCommand>
        {
            private:
            ros::Publisher leftPublisher_;
            ros::Publisher rightPublisher_;
            public:
            IndicatorCommandWriter(ros::NodeHandle* n,const std::string lefttopic,const std::string righttopic,int qsize)
            {
                leftPublisher_ = n->advertise<std_msgs::Bool>(lefttopic,qsize);
                rightPublisher_ = n->advertise<std_msgs::Bool>(righttopic,qsize);
            }
            ///canWriteMore indicates whether more data can be written
            virtual bool canWriteMore() const override
            {
                return true;
            }
            ///write sends out data value
            virtual void write(const adore::fun::IndicatorCommand& value)override
            {
                std_msgs::Bool msg_left,msg_right;
                msg_left.data = value.getIndicatorLeftOn();
                msg_right.data = value.getIndicatorRightOn();
                leftPublisher_.publish(msg_left);
                rightPublisher_.publish(msg_right);
            }
        };
                
        /**
         * ROS specific implementation of AReader for adore::fun::IndicatorCommand.
         * Receives ROS std_msgs::Bool and creates adore::fun::IndicatorCommand. 
         */
        class IndicatorCommandReader:public adore::mad::AReader<adore::fun::IndicatorCommand>
        {
            private:
            adore::fun::IndicatorCommand data_;
            ros::Subscriber leftIndicatorSubscriber_;
            ros::Subscriber rightIndicatorSubscriber_;
            bool changed_;
            void receive_left(std_msgs::BoolConstPtr msg)
            {
                data_.setIndicatorLeftOn(msg.get()->data);
            }
            void receive_right(std_msgs::BoolConstPtr msg)
            {
                data_.setIndicatorRightOn(msg.get()->data);
            }
            public:
            IndicatorCommandReader(ros::NodeHandle* n,const std::string& topic_leftIndicator,const std::string& topic_rightIndicator,int qsize)
            :changed_(false)
            {
                leftIndicatorSubscriber_ = n->subscribe(topic_leftIndicator,qsize,&IndicatorCommandReader::receive_left,this);
                rightIndicatorSubscriber_ = n->subscribe(topic_rightIndicator,qsize,&IndicatorCommandReader::receive_right,this);
            }
            virtual bool hasData() const override
            {
                return true;
            }
            bool hasUpdate() const override
            {
                return hasData() && changed_;
            }
            virtual void getData(adore::fun::IndicatorCommand& value)  override
            {
                value = data_;
            }
        };
    }
}