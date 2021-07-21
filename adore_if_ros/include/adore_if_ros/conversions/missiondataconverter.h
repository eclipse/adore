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
#include <adore/fun/missiondata.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/MissionState.h>
#include <std_msgs/UInt8.h>


namespace adore
{
    namespace if_ROS
    {
        /**
         * ROS specific implementation of AReader for MissionData.
         * Reads std_msgs::UInt8 (mission state) and converts to adore::fun::MissionData
         */
        class MissionDataReader:public adore::mad::AReader<adore::fun::MissionData>
        {
            private:
            ros::Subscriber mission_state_subscriber_;
            bool mission_state_initialized_;
            bool changed_;
            adore::fun::MissionData data_;

            void receive_mission_state(std_msgs::UInt8ConstPtr msg)
            {
                data_.setMissionState((adore::fun::MissionData::MissionState)msg->data);
                mission_state_initialized_ = true;
                changed_ = true;
            }

            public:
            MissionDataReader(ros::NodeHandle* n,
                                const std::string mission_state_topic,
                                int qsize)
                                :mission_state_initialized_(false),
                                changed_(false)
            {
                bool no_delay;
                n->param("/tcp_no_delay", no_delay, false);
                mission_state_subscriber_ = n->subscribe(mission_state_topic,qsize,&MissionDataReader::receive_mission_state,this,ros::TransportHints().tcpNoDelay(no_delay));
            }
            virtual bool hasData() const override
            {
                return mission_state_initialized_;
            }
            virtual bool hasUpdate() const override
            {
                return hasData() && changed_;
            }
            virtual void getData(adore::fun::MissionData& value) override
            {
                value = data_;
            }

        };


        /**
         * ROS specific implementation of AWriter for adore::fun::MissionData.
         * Transmits a adore::fun::MissionData by sending ROS std_msgs.
         */
        class MissionDataWriter: public adore::mad::AWriter<adore::fun::MissionData>
        {
            private:
            ros::Publisher mission_state_publisher_;

            public:
            MissionDataWriter(ros::NodeHandle* n,
                        const std::string& mission_state_topic,
                        int qsize)
            {
                mission_state_publisher_ = n->advertise<std_msgs::UInt8>(mission_state_topic,qsize);
            }
            virtual bool canWriteMore() const override
            {
                return true;
            }
            //sends out data value
            virtual void write(const adore::fun::MissionData& value) override
            {
                std_msgs::UInt8 msg;
                msg.data = (uint)value.getMissionState();
                mission_state_publisher_.publish(msg);
            }
        };
    }
}