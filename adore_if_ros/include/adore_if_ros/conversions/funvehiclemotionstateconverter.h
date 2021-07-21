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
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "quaternion.h"
#include <adore/fun/vehiclemotionstate9d.h>
#include <adore/fun/afactory.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/Border.h>
#include <adore_if_ros_msg/NavigationGoal.h>
#include <adore_if_ros_msg/SetPointRequest.h>
#include <adore_if_ros_msg/TerminalRequest.h>
#include <adore_if_ros_msg/WheelSpeed.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <adore/params/afactory.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * ROS specific implementation of AReader for VehicleMotionState9d.
         * Reads nav_msgs::Odometry, std_msgs::Float32 (steering angle), std_msgs::Float32 (acceleration) and converts to adore::fun::VehicleMotionState9d
         */
        class MotionStateReader:public adore::mad::AReader<adore::fun::VehicleMotionState9d>
        {
            private:
            ros::Subscriber odom_subscriber_;
            ros::Subscriber steering_subscriber_;
            ros::Subscriber acceleration_subscriber_;
            bool odom_initialized_;
            bool steering_initialized_;
            bool acceleration_initialized_;
            bool wheel_speed_available_;
            bool changed_;
            adore::fun::VehicleMotionState9d data_;
            adore::params::APVehicle* params_;

            void receive_odom(nav_msgs::OdometryConstPtr msg)
            {
                data_.setTime(msg->header.stamp.toSec());
                data_.setX(msg->pose.pose.position.x);
                data_.setY(msg->pose.pose.position.y);
                data_.setZ(msg->pose.pose.position.z);
                QuaternionConverter qc;
                data_.setPSI(qc.quaternionToHeading(msg->pose.pose));
                data_.setvx(msg->twist.twist.linear.x);
                data_.setvy(msg->twist.twist.linear.y);
                data_.setOmega(msg->twist.twist.angular.z);
                odom_initialized_ = true;
                changed_ = true;
            }
            void receive_steering(std_msgs::Float32ConstPtr msg)
            {
                if(params_==nullptr)
                {
                    auto pfactory = adore::params::ParamsFactoryInstance::get();
                    if(pfactory==nullptr)
                    {
                        std::cerr << " WARNING not reading motion state (steering angle) until parameter factory and steeringRatio parameter are available." 
                            << std::endl;
                        return;
                    }
                    else
                    {
                        params_ = pfactory->getVehicle();
                    }
                }
                data_.setDelta(msg->data /  params_->get_steeringRatio());
                steering_initialized_ = true;
                changed_ = true;
            }
            void receive_acceleration(std_msgs::Float32ConstPtr msg)
            {
                data_.setAx(msg->data);
                acceleration_initialized_ = true;
                changed_ = true;
            }

            public:
            MotionStateReader(ros::NodeHandle* n,
                                const std::string odom_topic,
                                const std::string accelerationTopic,
                                const std::string steeringAngleTopic,
                                int qsize)
                                :odom_initialized_(false),
                                steering_initialized_(false),
                                acceleration_initialized_(false),
                                changed_(false)
            {
                bool no_delay;
                n->param("/tcp_no_delay", no_delay, false);
                odom_subscriber_ = n->subscribe(odom_topic,qsize,&MotionStateReader::receive_odom,this,ros::TransportHints().tcpNoDelay(no_delay));
                acceleration_subscriber_ = n->subscribe(accelerationTopic,qsize,&MotionStateReader::receive_acceleration,this,ros::TransportHints().tcpNoDelay(no_delay));
                steering_subscriber_ = n->subscribe(steeringAngleTopic,qsize,&MotionStateReader::receive_steering,this,ros::TransportHints().tcpNoDelay(no_delay));
                params_ = nullptr;
            }
            virtual bool hasData() const override
            {
                return odom_initialized_ && steering_initialized_ && acceleration_initialized_;
            }
            virtual bool hasUpdate() const override
            {
                return hasData() && changed_;
            }
            virtual void getData(adore::fun::VehicleMotionState9d& value) override
            {
                value = data_;
            }

        };


        /**
         * ROS specific implementation of AReader for VehicleExtendedState.
         */
        class VehicleExtendedStateReader:public adore::mad::AReader<adore::fun::VehicleExtendedState>
        {
            private:
            ros::Subscriber gear_subscriber_;
            ros::Subscriber steering_subscriber_;
            ros::Subscriber acceleration_subscriber_;
            ros::Subscriber accelerationActive_subscriber_;
            ros::Subscriber leftIndicator_subscriber_;
            ros::Subscriber rightIndicator_subscriber_;
            ros::Subscriber checkpointClearance_subscriber_;
            bool gear_initialized_;
            bool steering_initialized_;
            bool acceleration_initialized_;
            bool accelerationActive_initialized_;
            bool left_indicator_initialized_;
            bool right_indicator_initialized_;
            bool checkpointClearance_initialized_;
            bool changed_;
            adore::fun::VehicleExtendedState data_;

            void receive_gear(std_msgs::Int8ConstPtr msg)
            {
                data_.setGearState((adore::fun::VehicleExtendedState::GearState)msg->data);
                gear_initialized_ = true;
                changed_ = true;
            }
            void receive_steering(std_msgs::BoolConstPtr msg)
            {
                data_.setAutomaticControlSteeringOn(msg->data);
                steering_initialized_ = true;
                changed_ = true;
            }
            void receive_acceleration(std_msgs::BoolConstPtr msg)
            {
                data_.setAutomaticControlAccelerationOn(msg->data);
                acceleration_initialized_ = true;
                changed_ = true;
            }
            void receive_accelerationActive(std_msgs::BoolConstPtr msg)
            {
                data_.setAutomaticControlAccelerationActive(msg->data);
                accelerationActive_initialized_ = true;
                changed_ = true;
            }
            void receive_left_indicator(std_msgs::BoolConstPtr msg)
            {
                data_.setIndicatorLeftOn(msg->data);
                left_indicator_initialized_ = true;
                changed_ = true;
            }
            void receive_right_indicator(std_msgs::BoolConstPtr msg)
            {
                data_.setIndicatorRightOn(msg->data);
                right_indicator_initialized_ = true;
                changed_ = true;
            }
            void receive_checkpointClearance(std_msgs::BoolConstPtr msg)
            {
                data_.setCheckpointClearance(msg->data);
                checkpointClearance_initialized_ = true;
                changed_ = true;
            }

            public:
            VehicleExtendedStateReader(ros::NodeHandle* n,
                                const std::string gear_state_topic,
                                const std::string accelerationTopic,
                                const std::string accelerationActiveTopic,
                                const std::string steeringTopic,
                                const std::string leftIndicatorTopic,
                                const std::string rightIndicatorTopic,
                                const std::string checkpointClearanceTopic,
                                int qsize)
                                :gear_initialized_(false),
                                steering_initialized_(false),
                                acceleration_initialized_(false),
                                accelerationActive_initialized_(false),
                                left_indicator_initialized_(false),
                                right_indicator_initialized_(false),
                                checkpointClearance_initialized_(false),
                                changed_(false)
            {
                bool no_delay;
                n->param("/tcp_no_delay", no_delay, false);
                gear_subscriber_ = n->subscribe(gear_state_topic,qsize,&VehicleExtendedStateReader::receive_gear,this,ros::TransportHints().tcpNoDelay(no_delay));
                acceleration_subscriber_ = n->subscribe(accelerationTopic,qsize,&VehicleExtendedStateReader::receive_acceleration,this,ros::TransportHints().tcpNoDelay(no_delay));
                accelerationActive_subscriber_ = n->subscribe(accelerationActiveTopic,qsize,&VehicleExtendedStateReader::receive_accelerationActive,this,ros::TransportHints().tcpNoDelay(no_delay));
                steering_subscriber_ = n->subscribe(steeringTopic,qsize,&VehicleExtendedStateReader::receive_steering,this,ros::TransportHints().tcpNoDelay(no_delay));
                leftIndicator_subscriber_ = n->subscribe(leftIndicatorTopic,qsize,&VehicleExtendedStateReader::receive_left_indicator,this,ros::TransportHints().tcpNoDelay(no_delay));
                rightIndicator_subscriber_ = n->subscribe(rightIndicatorTopic,qsize,&VehicleExtendedStateReader::receive_right_indicator,this,ros::TransportHints().tcpNoDelay(no_delay));
                checkpointClearance_subscriber_ = n->subscribe(checkpointClearanceTopic,qsize,&VehicleExtendedStateReader::receive_checkpointClearance,this,ros::TransportHints().tcpNoDelay(no_delay));
            }
            virtual bool hasData() const override
            {
                return gear_initialized_ && steering_initialized_ && acceleration_initialized_ && accelerationActive_initialized_ && left_indicator_initialized_ && right_indicator_initialized_ && checkpointClearance_initialized_;
            }
            virtual bool hasUpdate() const override
            {
                return hasData() && changed_;
            }
            virtual void getData(adore::fun::VehicleExtendedState& value) override
            {
                value = data_;
            }

        };


        /**
         * ROS specific implementation of AWriter for adore::fun::VehicleExtendedState.
         * Transmits a adore::fun::VehicleExtendedState by sending several ROS std_msgs.
         */
        class VehicleExtendedStateWriter: public adore::mad::AWriter<adore::fun::VehicleExtendedState>
        {
            private:
            ros::Publisher gearStatePublisher_;
            ros::Publisher accelerationOnPublisher_;
            ros::Publisher accelerationAcivePublisher_;
            ros::Publisher steeringOnPublisher_;
            ros::Publisher leftIndicatorStatePublisher_;
            ros::Publisher rightIndicatorStatePublisher_;
            ros::Publisher checkpointClearancePublisher_;
            public:
            VehicleExtendedStateWriter(ros::NodeHandle* n,
                        const std::string& gearStateTopic,
                        const std::string& accelerationTopic,
                        const std::string& accelerationActiveTopic,
                        const std::string& steeringTopic,
                        const std::string& leftIndicatorTopic,
                        const std::string& rightIndicatorTopic,
                        const std::string& checkpointClearanceTopic,
                        int qsize)
            {
                gearStatePublisher_ = n->advertise<std_msgs::Int8>(gearStateTopic,qsize);
                accelerationOnPublisher_ = n->advertise<std_msgs::Bool>(accelerationTopic,qsize);
                accelerationAcivePublisher_ = n->advertise<std_msgs::Bool>(accelerationActiveTopic,qsize);
                steeringOnPublisher_ = n->advertise<std_msgs::Bool>(steeringTopic,qsize);
                leftIndicatorStatePublisher_ = n->advertise<std_msgs::Bool>(leftIndicatorTopic,qsize);
                rightIndicatorStatePublisher_ = n->advertise<std_msgs::Bool>(rightIndicatorTopic,qsize);
                checkpointClearancePublisher_ = n->advertise<std_msgs::Bool>(checkpointClearanceTopic,qsize);
            }
            virtual bool canWriteMore() const override
            {
                return true;
            }
            ///write sends out data value
            virtual void write(const adore::fun::VehicleExtendedState& value) override
            {
                std_msgs::Int8 msg;
                msg.data = (int)value.getGearState();
                gearStatePublisher_.publish(msg);

                std_msgs::Bool bool_msg1;
                bool_msg1.data = value.getAutomaticControlAccelerationOn();
                accelerationOnPublisher_.publish(bool_msg1);
                
                bool_msg1.data = value.getAutomaticControlAccelerationActive();
                accelerationAcivePublisher_.publish(bool_msg1);

                std_msgs::Bool bool_msg2;
                bool_msg2.data = value.getAutomaticControlSteeringOn();
                steeringOnPublisher_.publish(bool_msg2);

                std_msgs::Bool bool_msg3;
                bool_msg3.data = value.getIndicatorRightOn();
                rightIndicatorStatePublisher_.publish(bool_msg3);

                std_msgs::Bool bool_msg4;
                bool_msg4.data = value.getIndicatorLeftOn();
                leftIndicatorStatePublisher_.publish(bool_msg4);

                std_msgs::Bool bool_msg5;
                bool_msg5.data = value.getCheckpointClearance();
                checkpointClearancePublisher_.publish(bool_msg5);
            }
        };

        /**
         * ROS specific implementation of AWriter for adore::fun::VehicleMotionState9d.
         * Transmits a adore::fun::VehicleMotionState9d by sending nav_msgs::Odometry and two std_msgs::Float32 for steering angle and acceleration.
         */
        class MotionStateWriter:public adore::mad::AWriter<adore::fun::VehicleMotionState9d>
        {
            private:
            ros::Publisher publisher_odom_;
            ros::Publisher publisher_steering_;
            ros::Publisher publisher_acceleration_;
            tf::TransformBroadcaster broadcaster;
            adore::params::APVehicle* params_;

            public:
            MotionStateWriter(ros::NodeHandle* n,const std::string& odom_topic,const std::string& steering_topic,const std::string& acceleration_topic,int qsize)
            {
                publisher_odom_= n->advertise<nav_msgs::Odometry>(odom_topic,qsize);
                publisher_steering_ = n->advertise<std_msgs::Float32>(steering_topic,qsize);
                publisher_acceleration_ = n->advertise<std_msgs::Float32>(acceleration_topic,qsize);
                params_=nullptr;
            }
            virtual bool canWriteMore() const override
            {
                return true;//@TODO
            }
            virtual void write(const adore::fun::VehicleMotionState9d& value)override
            {
                if(params_==nullptr)
                {
                    auto pfactory = adore::params::ParamsFactoryInstance::get();
                    if(pfactory==nullptr)
                    {
                        std::cerr << " WARNING not writing motion state until parameter factory and steeringRatio parameter are available." 
                            << std::endl;
                        return;
                    }
                    else
                    {
                        params_ = pfactory->getVehicle();
                    }
                }                
                ros::Time now;
                now.fromSec(value.getTime());
                nav_msgs::Odometry odom_msg;
                odom_msg.header.stamp=now;
                odom_msg.header.frame_id = "UTMZone";
                odom_msg.child_frame_id = "COR";
                odom_msg.pose.pose.position.x = value.getX();
                odom_msg.pose.pose.position.y = value.getY();
                odom_msg.pose.pose.position.z = value.getZ();
                QuaternionConverter qc;
                qc.setHeading(value.getPSI(),odom_msg.pose.pose);
                odom_msg.twist.twist.linear.x = value.getvx();
                odom_msg.twist.twist.linear.y = value.getvy();
                odom_msg.twist.twist.linear.z = 0.0;
                odom_msg.twist.twist.angular.x =0.0;
                odom_msg.twist.twist.angular.y =0.0;
                odom_msg.twist.twist.angular.z =value.getOmega();
                publisher_odom_.publish(odom_msg);
                std_msgs::Float32 delta;
                delta.data = value.getDelta() *  params_->get_steeringRatio();
                publisher_steering_.publish(delta);
                std_msgs::Float32 acceleration;
                acceleration.data = value.getAx();
                publisher_acceleration_.publish(acceleration);
                broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.0,0.0,0.0)),
                    now,"map","UTMZone"
                    ) 
                );
            }
        };
    }
}