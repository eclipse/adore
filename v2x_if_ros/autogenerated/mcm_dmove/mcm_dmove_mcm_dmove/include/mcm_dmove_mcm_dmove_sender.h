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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_dmove:3.0
 * 
 * Module: MCM_DMove {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) mcm(1) version(3)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef MCM_DMOVE_MCM_DMOVE_SENDER_H
#define MCM_DMOVE_MCM_DMOVE_SENDER_H

#include <ros/ros.h>

#include <wind_constants.h>
#include <mcm_dmove_mcm_dmove/MCM.h>   // From ROS
#include <mcm_dmove_mcm_dmove.h>
#include <mcm_dmove_mcm_dmove_translator_ros2wind.h>

#include <udp_sender.h>

namespace wind {
namespace wind_ros {

class Sender_mcm_dmove_mcm_dmove
{
private:
    std::string            name;
    ros::NodeHandle        node_handle;
    ros::Subscriber        subscriber;
    ros::Subscriber        mirrorSubscriber;
    wind::comm::UDPSender* udp_sender;

public:
    Sender_mcm_dmove_mcm_dmove(wind::comm::UDPSender* udp_sender);
    ~Sender_mcm_dmove_mcm_dmove() {};

    std::string            topic;
    std::string            output_ip;
    int                    output_port;

    void start();
    void callback(const mcm_dmove_mcm_dmove::MCM::ConstPtr& msg);
    void mirrorCallback(const mcm_dmove_mcm_dmove::MCM::ConstPtr& msg);
};

} // Closing namespace ros
} // Closing namespace wind

#endif //MCM_DMOVE_MCM_DMOVE_SENDER_H
