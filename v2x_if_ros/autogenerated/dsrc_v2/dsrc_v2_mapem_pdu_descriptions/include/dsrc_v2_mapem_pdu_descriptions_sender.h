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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:dsrc_v2:2.5
 * 
 * Module: MAPEM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(103301) mapem(1) version2(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef DSRC_V2_MAPEM_PDU_DESCRIPTIONS_SENDER_H
#define DSRC_V2_MAPEM_PDU_DESCRIPTIONS_SENDER_H

#include <ros/ros.h>

#include <wind_constants.h>
#include <dsrc_v2_mapem_pdu_descriptions/MAPEM.h>   // From ROS
#include <dsrc_v2_mapem_pdu_descriptions.h>
#include <dsrc_v2_mapem_pdu_descriptions_translator_ros2wind.h>

#include <udp_sender.h>

namespace wind {
namespace wind_ros {

class Sender_dsrc_v2_mapem_pdu_descriptions
{
private:
    std::string            name;
    ros::NodeHandle        node_handle;
    ros::Subscriber        subscriber;
    ros::Subscriber        mirrorSubscriber;
    wind::comm::UDPSender* udp_sender;
    uint32_t               messagesCounter;

public:
    Sender_dsrc_v2_mapem_pdu_descriptions(wind::comm::UDPSender* udp_sender);
    ~Sender_dsrc_v2_mapem_pdu_descriptions() {};

    std::string            topic;
    std::string            output_ip;
    int                    output_port;

    void start();
    void callback(const dsrc_v2_mapem_pdu_descriptions::MAPEM::ConstPtr& msg);
    void mirrorCallback(const dsrc_v2_mapem_pdu_descriptions::MAPEM::ConstPtr& msg);
};

} // Closing namespace ros
} // Closing namespace wind

#endif //DSRC_V2_MAPEM_PDU_DESCRIPTIONS_SENDER_H
