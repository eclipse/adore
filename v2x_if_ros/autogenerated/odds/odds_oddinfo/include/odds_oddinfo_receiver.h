/*
 * Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)
 * Institute of Transportation Systems. (https://www.dlr.de/ts/)
 * 
 * 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 * 
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 * 
 * SPDX-License-Identifier: EPL-2.0
 * 
 * 
 * 
 * File automatically generated with DLR Wind v2 (2021)
 * 
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:odds:12.0
 * 
 * Module: ODDInfo {}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 */
#ifndef ODDS_ODDINFO_RECEIVER_H
#define ODDS_ODDINFO_RECEIVER_H

// File name: odds_oddinfo_receiver.h

#include <ros/ros.h>
#include <wind_constants.h>
#include <odds_oddinfo/ODDMSG.h>
#include <odds_oddinfo.h>
#include <odds_oddinfo_translator_wind2ros.h>

namespace wind
{
namespace wind_ros
{
    class Receiver_odds_oddinfo
    {
    public:
        Receiver_odds_oddinfo();
        uint32_t message_id();
        void incoming_message(const void* buf);
    
    private:
    
        ros::NodeHandle node_handle;
        ros::Publisher publisher;
        ros::Publisher mirrorPublisher;
        std::string topic;
        std::string name;
    };
}  // Closing namespace wind_ros
}  // Closing namespace wind

#endif //ODDS_ODDINFO_RECEIVER_H
