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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_transaid:3.0
 * 
 * Module: MCM_TransAID {version(3)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 */
#ifndef MCM_TRANSAID_MCM_TRANSAID_RECEIVER_H
#define MCM_TRANSAID_MCM_TRANSAID_RECEIVER_H

// File name: mcm_transaid_mcm_transaid_receiver.h

#include <ros/ros.h>
#include <wind_constants.h>
#include <mcm_transaid_mcm_transaid/MCM.h>
#include <mcm_transaid_mcm_transaid.h>
#include <mcm_transaid_mcm_transaid_translator_wind2ros.h>

namespace wind
{
namespace wind_ros
{
    class Receiver_mcm_transaid_mcm_transaid
    {
    public:
        Receiver_mcm_transaid_mcm_transaid();
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

#endif //MCM_TRANSAID_MCM_TRANSAID_RECEIVER_H
