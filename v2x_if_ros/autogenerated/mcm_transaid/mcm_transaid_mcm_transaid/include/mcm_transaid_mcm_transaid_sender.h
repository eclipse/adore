/*
 *
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
 *
 */

#ifndef MCM_TRANSAID_MCM_TRANSAID_SENDER_H
#define MCM_TRANSAID_MCM_TRANSAID_SENDER_H

#include <ros/ros.h>

#include <wind_constants.h>
#include <mcm_transaid_mcm_transaid/MCM.h>   // From ROS
#include <mcm_transaid_mcm_transaid.h>
#include <mcm_transaid_mcm_transaid_translator_ros2wind.h>

#include <udp_sender.h>

namespace wind {
namespace wind_ros {

class Sender_mcm_transaid_mcm_transaid
{
private:
    std::string            name;
    ros::NodeHandle        node_handle;
    ros::Subscriber        subscriber;
    ros::Subscriber        mirrorSubscriber;
    wind::comm::UDPSender* udp_sender;

public:
    Sender_mcm_transaid_mcm_transaid(wind::comm::UDPSender* udp_sender);
    ~Sender_mcm_transaid_mcm_transaid() {};

    std::string            topic;
    std::string            output_ip;
    int                    output_port;

    void start();
    void callback(const mcm_transaid_mcm_transaid::MCM::ConstPtr& msg);
    void mirrorCallback(const mcm_transaid_mcm_transaid::MCM::ConstPtr& msg);
};

} // Closing namespace ros
} // Closing namespace wind

#endif //MCM_TRANSAID_MCM_TRANSAID_SENDER_H
