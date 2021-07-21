/****************************************************************************/
// Eclipse ADORe, Automated Driving Open Research; see https://eclipse.org/adore
// Copyright (C) 2017-2020 German Aerospace Center (DLR).
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    channel2station.h
/// @author  Daniel Heß
/// @contact Daniel.Hess@DLR.de
/// @date 2020/04/02
/// @initialrelease X.X.X
///
// ChannelToStation class handles message hand-over between simulated channel and station v2x topic

#pragma once
#include <ros/ros.h>
#include <string>
#include "channel.h"

namespace v2xsim
{
    /**
     * ChannelToStation class handles message hand-over between simulated channel and station v2x topic
     */
    template<typename TChannelMsg,
            typename TChannelMsgConstPtr,
            typename TStationMsg,
            typename TStationMsgConstPtr>
    class ChannelToStation
    {
        private:
        ros::Publisher to_channel_publisher_;
        ros::Publisher to_station_publisher_;
        ros::Subscriber from_channel_subscriber_;
        ros::Subscriber from_station_subscriber_;
        Station* station_;
        Channel* channel_;
        void station_receive(TStationMsgConstPtr msg)
        {
            TChannelMsg msgout;
            msgout.data=*msg;
            msgout.meta.location.x = station_->getX();
            msgout.meta.location.y = station_->getY();
            msgout.meta.location.z = station_->getZ();
            msgout.meta.time = station_->getT();
            msgout.meta.bytecount = 100;
            msgout.meta.power = 22.0;
            to_channel_publisher_.publish(msgout);
        }
        void channel_receive(TChannelMsgConstPtr msg)
        {
            bool received = false;
            double delay = 0.0;//@TODO implement delay
            channel_->notify(msg->meta,received,delay);
            if(received)to_station_publisher_.publish(msg->data);
            if(received)std::cout<<"Message received"<<std::endl;
            else std::cout<<"Message lost"<<std::endl;
        }
        public:
        ChannelToStation(ros::NodeHandle& n,
                        std::string channel_incoming_topic,
                        std::string channel_outgoing_topic,
                        std::string station_incoming_topic,
                        std::string station_outgoing_topic,
                        Station* station,
                        Channel* channel
                        ):station_(station),channel_(channel)
        {
            from_channel_subscriber_ = n.subscribe(channel_incoming_topic,100,&ChannelToStation::channel_receive,this);
            from_station_subscriber_ = n.subscribe(station_outgoing_topic,100,&ChannelToStation::station_receive,this);
            to_channel_publisher_ = n.advertise<TChannelMsg>(channel_outgoing_topic,100);
            to_station_publisher_ = n.advertise<TStationMsg>(station_incoming_topic,100);
        }
    };
}