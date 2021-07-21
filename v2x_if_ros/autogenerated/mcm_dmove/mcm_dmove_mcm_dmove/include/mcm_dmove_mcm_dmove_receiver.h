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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_dmove:3.0
 * 
 * Module: MCM_DMove {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) mcm(1) version(3)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 */
#ifndef MCM_DMOVE_MCM_DMOVE_RECEIVER_H
#define MCM_DMOVE_MCM_DMOVE_RECEIVER_H

// File name: mcm_dmove_mcm_dmove_receiver.h

#include <ros/ros.h>
#include <wind_constants.h>
#include <mcm_dmove_mcm_dmove/MCM.h>
#include <mcm_dmove_mcm_dmove.h>
#include <mcm_dmove_mcm_dmove_translator_wind2ros.h>

namespace wind
{
namespace wind_ros
{
    class Receiver_mcm_dmove_mcm_dmove
    {
    public:
        Receiver_mcm_dmove_mcm_dmove();
        void incoming_message(const void* buf);
        void start();
        void set_topic(std::string t);
        std::string message_name();
        uint32_t message_id();
    
    private:
    
        ros::NodeHandle node_handle;
        ros::Publisher publisher;
        ros::Publisher mirrorPublisher;
        std::string topic;
    };
}  // Closing namespace wind_ros
}  // Closing namespace wind

#endif //MCM_DMOVE_MCM_DMOVE_RECEIVER_H
