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
         * ROS specific implementation of AWriter for adore::fun::MotionCommand.
         * Transmits a MotionCommand by sending two ROS std_msgs::Float32 for acceleration and steering angle.
         */
        class MotionCommandWriter:public adore::mad::AWriter<adore::fun::MotionCommand>
        {
            private:
            ros::Publisher accelerationPublisher_;
            ros::Publisher steeringAnglePublisher_;
            public:
            MotionCommandWriter(ros::NodeHandle* n,const std::string axtopic,const std::string deltatopic,int qsize)
            {
                accelerationPublisher_ = n->advertise<std_msgs::Float32>(axtopic,qsize);
                steeringAnglePublisher_ = n->advertise<std_msgs::Float32>(deltatopic,qsize);
            }
            ///canWriteMore indicates whether more data can be written
            virtual bool canWriteMore() const override
            {
                return true;
            }
            ///write sends out data value
            virtual void write(const adore::fun::MotionCommand& value)override
            {
                std_msgs::Float32 msg_acceleration,msg_steeringAngle;
                msg_acceleration.data = value.getAcceleration();
                msg_steeringAngle.data = value.getSteeringAngle();
                accelerationPublisher_.publish(msg_acceleration);
                steeringAnglePublisher_.publish(msg_steeringAngle);
            }
        };
        /**
         * ROS specific implementation of AReader for adore::fun::MotionCommand.
         * Receives two std_msgs::Float32 to create adore::fun::MotionCommand.
         */
        class MotionCommandReader:public adore::mad::AReader<adore::fun::MotionCommand>
        {
            private:
                bool changed_;
                bool acceleration_initialized_;
                bool steering_initialized_;
                ros::Subscriber accelerationSubscriber_;
                ros::Subscriber steeringSubscriber_;


                adore::fun::MotionCommand data_;

                void receive_accelerationRequest(std_msgs::Float32ConstPtr msg)
                {
                    data_.setAcceleration(msg->data);
                    acceleration_initialized_ = true;
                    changed_ = true;
                }                        
                void receive_steeringRequest(std_msgs::Float32ConstPtr msg)
                {
                    data_.setSteeringAngle(msg->data);
                    steering_initialized_ = true;
                    changed_ = true;
                }                        
            public:
                MotionCommandReader(ros::NodeHandle* n,const std::string axtopic,const std::string steeringtopic,int qsize)
                {
                    accelerationSubscriber_ = n->subscribe(axtopic,qsize,&MotionCommandReader::receive_accelerationRequest,this);
                    steeringSubscriber_ = n->subscribe(steeringtopic,qsize,&MotionCommandReader::receive_steeringRequest,this);
                    changed_ = false;
                    acceleration_initialized_ = false;
                    steering_initialized_ = false;
                }
                ///hasData indicates whether the data has been initialized with a first data item
                virtual bool hasData() const override
                {
                    return acceleration_initialized_ && steering_initialized_;
                }
                ///hasUpdate indicates whether the data item was updated since last getdata
                virtual bool hasUpdate() const override
                {
                    return changed_;
                }
                ///getData returns the latest data item
                virtual void getData(adore::fun::MotionCommand& value)
                {
                    changed_ = false;
                    value = data_;
                }
        };       
    }
}