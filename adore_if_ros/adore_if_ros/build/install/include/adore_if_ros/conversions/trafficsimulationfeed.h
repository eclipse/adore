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
#include "trafficparticipantconverter.h"
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/TrafficParticipantSetSimulation.h>

namespace adore
{
    namespace if_ROS
    {
        class TrafficSimulationFeed:public adore::mad::AFeed<adore::env::traffic::Participant>
        {
            private:
                std::list<adore::env::traffic::Participant> data_;
                TPSimulationConverter converter_;
                ros::Subscriber single_subscriber_;
                ros::Subscriber multi_subscriber_;
                /// receives single message and puts data element in data_queue
                void receive_single(adore_if_ros_msg::TrafficParticipantSimulationConstPtr msg)
                {
                    data_.emplace_back();
                    converter_(msg,data_.back());
                }
                /// receives aggregates and unpacks into data_ queue
                void receive_multi(adore_if_ros_msg::TrafficParticipantSetSimulationConstPtr msg)
                {
                    for(auto& item:msg->data)
                    {
                        data_.emplace_back();
                        converter_(item,data_.back());
                    }
                }
                public:
                TrafficSimulationFeed(ros::NodeHandle* n,const std::string& single_topic,int single_qsize,
                                                         const std::string& multi_topic,int multi_qsize)
                {
                    bool no_delay;
                    n->param("/tcp_no_delay", no_delay, false);
                    single_subscriber_ = n->subscribe(single_topic,single_qsize,&TrafficSimulationFeed::receive_single,this,ros::TransportHints().tcpNoDelay(no_delay));
                    multi_subscriber_ = n->subscribe(multi_topic,multi_qsize,&TrafficSimulationFeed::receive_multi,this,ros::TransportHints().tcpNoDelay(no_delay));
                }

            public:


            /**
             * hasNext indicates whether there is more data to read
             */
            virtual bool hasNext() const override
            {
                return data_.size()>0;
            }
            /**
             * getNext reads the next data element
             */
            virtual void getNext(adore::env::traffic::Participant& value) override
            {
                value = data_.front();
                data_.pop_front();                
            }
            /**
             * getLatest reads the latest data element and discards all previous
             */
            virtual void getLatest(adore::env::traffic::Participant& value) override
            {
                value = data_.back();
                data_.clear();
            }
        };
    }
}