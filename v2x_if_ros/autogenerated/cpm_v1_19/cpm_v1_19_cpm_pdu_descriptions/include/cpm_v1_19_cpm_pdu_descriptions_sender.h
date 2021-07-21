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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:cpm_v1_19:1.3
 * 
 * Module: CPM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) ts(103324) cpm(1) version1(1)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef CPM_V1_19_CPM_PDU_DESCRIPTIONS_SENDER_H
#define CPM_V1_19_CPM_PDU_DESCRIPTIONS_SENDER_H

#include <ros/ros.h>

#include <wind_constants.h>
#include <cpm_v1_19_cpm_pdu_descriptions/CPM.h>   // From ROS
#include <cpm_v1_19_cpm_pdu_descriptions.h>
#include <cpm_v1_19_cpm_pdu_descriptions_translator_ros2wind.h>

#include <udp_sender.h>

namespace wind {
namespace wind_ros {

class Sender_cpm_v1_19_cpm_pdu_descriptions
{
private:
    std::string            name;
    ros::NodeHandle        node_handle;
    ros::Subscriber        subscriber;
    ros::Subscriber        mirrorSubscriber;
    wind::comm::UDPSender* udp_sender;

public:
    Sender_cpm_v1_19_cpm_pdu_descriptions(wind::comm::UDPSender* udp_sender);
    ~Sender_cpm_v1_19_cpm_pdu_descriptions() {};

    std::string            topic;
    std::string            output_ip;
    int                    output_port;

    void start();
    void callback(const cpm_v1_19_cpm_pdu_descriptions::CPM::ConstPtr& msg);
    void mirrorCallback(const cpm_v1_19_cpm_pdu_descriptions::CPM::ConstPtr& msg);
};

} // Closing namespace ros
} // Closing namespace wind

#endif //CPM_V1_19_CPM_PDU_DESCRIPTIONS_SENDER_H
