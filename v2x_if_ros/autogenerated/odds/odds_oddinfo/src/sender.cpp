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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:odds:12.0
 * 
 * Module: ODDInfo {}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef WIND_ROS_SENDER_NODE
#define WIND_ROS_SENDER_NODE
#endif

#include <wind_constants.h>
#include <ros/ros.h>
#include <udp_sender.h>
#include <odds_oddinfo_sender.h>

using namespace wind;

class SenderNode
{
public:
    SenderNode() : udp_sender(nullptr) {}

    ~SenderNode() {
        delete udp_sender;
        delete sender_odds_oddinfo;
    }

    void run() {
        if (!getParameters()) return;

        ROS_INFO_STREAM("Starting ROS node " << SENDER_NODE_NAME << ".");

        output_ip = SENDER_REMOTE_IP;
        output_port = SENDER_REMOTE_UDP_PORT;

        std::stringstream port;
        port << output_port;
        ROS_INFO_STREAM("Starting UDP Client to: " << output_ip << ":" << output_port << ".");
        udp_sender = new wind::comm::UDPSender(output_ip, port.str());

        // Modules
        sender_odds_oddinfo = new wind::wind_ros::Sender_odds_oddinfo(udp_sender);
        sender_odds_oddinfo->start();

        ros::spin();
    }

protected:
    bool getParameters() {
        name = ros::this_node::getName();
        return true;
    }

private:
    std::string            name;
    wind::comm::UDPSender* udp_sender;
    std::string            output_ip;
    int                    output_port;

    // Modules
    wind::wind_ros::Sender_odds_oddinfo* sender_odds_oddinfo;
};

//////////////// Main ////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, SENDER_NODE_NAME);
    SenderNode node;
    node.run();
    return 0;
}
