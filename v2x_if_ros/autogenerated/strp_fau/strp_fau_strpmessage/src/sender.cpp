/*
 *
 * Copyright (C) 2017-2020 German Aerospace Center e.V. (https://www.dlr.de)
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
 * File automatically generated with DLR Wind v2 (30.11.2020 11:00:04)
 * 
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:strp_fau:1.0
 * 
 * Module: STRPMessage {}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#ifndef WIND_ROS
#define WIND_ROS
#endif

#define WIND_DEBUG 0

#include <ros/ros.h>
#include <udp_sender.h>
#include <strp_fau_strpmessage_sender.h>

using namespace wind;

class SenderNode
{
public:
    SenderNode() : udp_sender(nullptr) {}

    ~SenderNode() {
        delete udp_sender;
        delete sender_strp_fau_strpmessage;
    }

    void run() {
        if (!getParameters()) return;

        ROS_INFO_STREAM("Starting ROS node " << name << ".");

        output_ip = "127.0.0.1";
        output_port = 1111;

        std::stringstream port;
        port << output_port;
        ROS_INFO_STREAM("Starting UDP Client to: " << output_ip << ":" << output_port << ".");
        udp_sender = new wind::comm::UDPSender(output_ip, port.str());

        // Modules
        sender_strp_fau_strpmessage = new wind::wind_ros::Sender_strp_fau_strpmessage(udp_sender);
        sender_strp_fau_strpmessage->start();

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
    wind::wind_ros::Sender_strp_fau_strpmessage* sender_strp_fau_strpmessage;
};

//////////////// Main ////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "wind_sender");
    SenderNode node;
    node.run();
    return 0;
}
