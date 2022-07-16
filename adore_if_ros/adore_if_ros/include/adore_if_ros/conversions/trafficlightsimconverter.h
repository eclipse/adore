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
 *   Stephan Lapoehn - initial API and implementation
 ********************************************************************************/

#pragma once

#include <adore/fun/afactory.h>
#include <adore/env/tcd/trafficlight.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/TrafficLightSimulation.h>

namespace adore
{
    namespace if_ROS
    {

        /**
         * ROS specific implementation of AWriter for adore::env::SimTrafficLight.
         * Transmits adore::env::SimTrafficLight by sending a ROS TrafficLightSimulation msg.
         */
        class TrafficLightSimWriter:public adore::mad::AWriter<adore::env::SimTrafficLight>
        {
            private:
            ros::Publisher simTLPublisher_;
       
            public:
            TrafficLightSimWriter(ros::NodeHandle* n,const std::string tlSimTopic, int qsize)
            {
                simTLPublisher_ = n->advertise<adore_if_ros_msg::TrafficLightSimulation>(tlSimTopic,qsize);
                
            }
            ///canWriteMore indicates whether more data can be written
            virtual bool canWriteMore() const override
            {
                return true;
            }
            ///write sends out data value
            virtual void write(const adore::env::SimTrafficLight & value) override
            {
                adore_if_ros_msg::TrafficLightSimulation msg_simtl;
                
                //sim data
                msg_simtl.greenDuration = value.green_duration_;
                msg_simtl.redDuration = value.red_duration_;
                msg_simtl.startState = (int) value.start_state_;
                msg_simtl.yellowDuration = value.yellow_duration_;
                msg_simtl.yellowRedDuration = value.yellow_red_duration_;
                msg_simtl.timeToGreen = value.time_to_green_;
                msg_simtl.timeToRed = value.time_to_red_;
                msg_simtl.timeToRedYellow = value.time_to_red_yellow_;
                msg_simtl.timeToYellow = value.time_to_yellow_;
                msg_simtl.tl.tcd.id = value.getID();

                //tl data
                msg_simtl.tl.junctionId = value.intersection_id_;
                msg_simtl.tl.movementId = value.movement_id_;

                // auto 
                const auto status = value.getStatus();

                msg_simtl.tl.tlstate = (int) (status->getCurrentColor());

                msg_simtl.tl.tcd.pose.position.x = value.getCoordinate().m_X;
                msg_simtl.tl.tcd.pose.position.y = value.getCoordinate().m_Y;
                msg_simtl.tl.tcd.pose.position.z = value.getCoordinate().m_Z;

                simTLPublisher_.publish(msg_simtl);
            }
            virtual std::string getDesc() override
            {
                return std::string("Node will write to ROS-topic: ").append(simTLPublisher_.getTopic());
            }
        };
                
        /**
         * ROS specific implementation of AReader for adore::env::SimTrafficLight.
         * Receives ROS adore_if_ros_msg::TrafficLightSimulation and creates adore::env::SimTrafficLight. 
         */
         class TrafficLightSimReader:public adore::mad::AReader<adore::env::SimTrafficLightMap>
        {  
            private:
            adore::env::SimTrafficLightMap data_;

            ros::Subscriber simTLSubscriber_;

            bool changed_;
         
            void receive_simTL(adore_if_ros_msg::TrafficLightSimulation msg)
            {
                adore::env::SimTrafficLight stl;

                stl.intersection_id_ = msg.tl.junctionId;
                stl.movement_id_ = msg.tl.movementId;
                stl.red_duration_ = msg.redDuration;
                stl.green_duration_ = msg.greenDuration;
                stl.yellow_red_duration_ = msg.yellowRedDuration;
                stl.yellow_duration_ = msg.yellowDuration;
                stl.time_to_green_ = msg.timeToGreen;
                stl.time_to_red_ = msg.timeToRed;
                stl.time_to_red_yellow_ = msg.timeToRedYellow;
                stl.time_to_yellow_ = msg.timeToYellow;
                stl.start_state_ = adore::env::TrafficLightColor(msg.startState);
                
                
                adore::env::BorderBased::Coordinate coord; 
                coord.m_X = msg.tl.tcd.pose.position.x;
                coord.m_Y = msg.tl.tcd.pose.position.y;
                coord.m_Z = msg.tl.tcd.pose.position.z;
                
                stl.setCoordinate(coord);
                stl.setType(adore::env::TrafficControlDevice::TRAFFIC_LIGHT);
                stl.getStatus()->setCurrentColor(adore::env::TrafficLightColor(msg.tl.tlstate));

                data_[stl.getCoordinate()] = stl; 
            }
            
            public:
            TrafficLightSimReader(ros::NodeHandle* n,const std::string& topic_tlsim,int qsize)
            :changed_(false)
            {
                bool no_delay;
                n->param("/tcp_no_delay", no_delay, false);
                simTLSubscriber_ = n->subscribe(topic_tlsim,qsize,&TrafficLightSimReader::receive_simTL,this,ros::TransportHints().tcpNoDelay(no_delay));   
            }
            virtual bool hasData() const override
            {
                return true;
            }
            bool hasUpdate() const override
            {
                return hasData() && changed_;
            }
            virtual void getData(adore::env::SimTrafficLightMap& value)  override
            {
                value = data_;
            }
        };
    }
}