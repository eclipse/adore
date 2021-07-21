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

#ifndef WIND_ROS_SENDER_NODE
#define WIND_ROS_SENDER_NODE
#endif

#include <wind_constants.h>
#include <ros/ros.h>
#include <udp_sender.h>
#include <mcm_dmove_mcm_dmove_sender.h>

using namespace wind;

class SenderNode
{
public:
    SenderNode() : udp_sender(nullptr) {}

    ~SenderNode() {
        delete udp_sender;
        delete sender_mcm_dmove_mcm_dmove;
    }

    void run() {
        ROS_INFO_STREAM("Starting ROS node " << ros::this_node::getName() << ".");
    
        std::stringstream port;
        port << output_port;
        ROS_INFO_STREAM("Starting UDP Client remoteHost(" << output_ip << ":" << output_port << ")");
        udp_sender = new wind::comm::UDPSender(output_ip, port.str());
    
        // Modules
        sender_mcm_dmove_mcm_dmove = new wind::wind_ros::Sender_mcm_dmove_mcm_dmove(udp_sender);
        sender_mcm_dmove_mcm_dmove->output_ip   = output_ip;
        sender_mcm_dmove_mcm_dmove->output_port = output_port;
        sender_mcm_dmove_mcm_dmove->topic       = topic;
        sender_mcm_dmove_mcm_dmove->start();
    
        ROS_INFO_STREAM("Ready!");
    
        ros::spin();
    }

    void set_output_port(int port) {
        output_port = port;
    }
    
    void set_output_ip(std::string address) {
        output_ip = address;
    }
    
    void set_topic(std::string t) {
        topic = t;
    }

private:
    wind::comm::UDPSender* udp_sender;
    std::string            topic;
    std::string            output_ip;
    int                    output_port;

    // Modules
    wind::wind_ros::Sender_mcm_dmove_mcm_dmove* sender_mcm_dmove_mcm_dmove;
};

//////////////// Main ////////////////
int main(int argc, char** argv)
{
    ROS_INFO_STREAM("*********************************************");
    ROS_INFO_STREAM("****  Starting Wind ROS Client (Sender)  ****");
    ROS_INFO_STREAM("*********************************************");
    ROS_INFO_STREAM("Usage: ./sender [--name nodeName] [--topic topic] [--address ipAddress] [--port portNumber]");
    ROS_INFO_STREAM("Default values in include/wind_constants.h");
    ROS_INFO_STREAM("");

    //Setting default values (defined in wind_constants.h)
    std::string _name  = SENDER_NODE_NAME;
    std::string _topic = SENDER_ROS_TOPIC;
    std::string _ip    = SENDER_REMOTE_IP;
    int         _port  = SENDER_REMOTE_UDP_PORT;

    for (int i = 1; i < argc; ++i)
    {
        if(std::string(argv[i]) == "--name")
            _name = std::string(argv[i+1]);

        if(std::string(argv[i]) == "--port")
            _port = std::stoi(argv[i+1]);

        if(std::string(argv[i]) == "--address")
            _ip = std::string(argv[i+1]);

        if(std::string(argv[i]) == "--topic")
            _topic = std::string(argv[i+1]);
    }

    ROS_INFO_STREAM("+-------------------------------------------------------"); 
    ROS_INFO_STREAM("| Node name      : " << _name);
    ROS_INFO_STREAM("| ROS topic      : " << _topic);
    ROS_INFO_STREAM("| Remote address : " << _ip);
    ROS_INFO_STREAM("| Remote port    : " << _port);
    ROS_INFO_STREAM("+-------------------------------------------------------"); 
    ROS_INFO_STREAM("");
    
    ros::init(argc, argv, _name); 
    SenderNode node;
    node.set_topic(_topic);
    node.set_output_ip(_ip);
    node.set_output_port(_port);
    node.run();

    return 0;
}
