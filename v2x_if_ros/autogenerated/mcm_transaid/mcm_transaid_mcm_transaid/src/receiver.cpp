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

#ifndef WIND_ROS_RECEIVER_NODE
#define WIND_ROS_RECEIVER_NODE
#endif

#include <wind_constants.h>
#include <ros/ros.h>
#include <udp_receiver.h>
#include <mcm_transaid_mcm_transaid_receiver.h>


class ReceiverNode
{
public:
    ReceiverNode() : udp_receiver(nullptr), node_handle("~") { }

    ~ReceiverNode() {
        delete udp_receiver;
        delete receiver_mcm_transaid_mcm_transaid;
    }

    void receiveMsgCallback(const void* buf, const size_t& buf_size)
    {
        ROS_DEBUG("Called receiveMsgViaUDP().");
        ROS_INFO_STREAM(ros::this_node::getName() << ": Incoming Msg (" << buf_size << " bytes)");
    
        uint16_t* endianness = (uint16_t*)buf;
        if(*endianness != 1) {
            ROS_INFO_STREAM("Ignoring incoming message: Wrong endianness");
            return;
        }
    
        if(buf_size == receiver_mcm_transaid_mcm_transaid->message_id()) {
            receiver_mcm_transaid_mcm_transaid->incoming_message(buf);
        }
        else {
            ROS_ERROR_STREAM("Received UDP Msg on Port "
                << input_port << " with no corresponding registered message("
                << buf_size << ")");
        }
    }

    void run()
    {
        ROS_INFO_STREAM("Starting ROS node " << ros::this_node::getName());
    
        receiver_mcm_transaid_mcm_transaid = new wind::wind_ros::Receiver_mcm_transaid_mcm_transaid();
        ROS_INFO_STREAM("Registering message name(" << receiver_mcm_transaid_mcm_transaid->message_name() 
            << ") id(" << receiver_mcm_transaid_mcm_transaid->message_id() << ")");
        receiver_mcm_transaid_mcm_transaid->set_topic(topic);
        receiver_mcm_transaid_mcm_transaid->start();
    
        ROS_INFO_STREAM("Starting UDP server port(" << input_port << ")");
        udp_receiver = new wind::comm::UDPReceiver(input_port);
        udp_receiver->setReceiveCallbackFunction(boost::bind(&ReceiverNode::receiveMsgCallback, this, _1, _2));
        udp_receiver->startReading();
    
        ROS_INFO_STREAM("Ready!");
    
        ros::spin();
    }

    void set_input_port(int port) {
        input_port = port;
    }

    void set_topic(std::string t) {
        topic = t;
    }

private:
    int                      input_port;
    ros::NodeHandle          node_handle;
    wind::comm::UDPReceiver* udp_receiver;
    std::string              topic;

    wind::wind_ros::Receiver_mcm_transaid_mcm_transaid* receiver_mcm_transaid_mcm_transaid;
};

//////////////// Main ////////////////
int main(int argc, char** argv) {
    ROS_INFO_STREAM("*********************************************");
    ROS_INFO_STREAM("**** Starting Wind ROS Client (Receiver) ****");
    ROS_INFO_STREAM("*********************************************");
    ROS_INFO_STREAM("Usage: ./receiver [--name nodeName] [--port portNumber] [--topic topic]");
    ROS_INFO_STREAM("Default values in include/wind_constants.h");
    ROS_INFO_STREAM("");

    //Setting default values (defined in wind_constants.h)
    std::string _name  = RECEIVER_NODE_NAME;
    std::string _topic = RECEIVER_ROS_TOPIC;
    int         _port  = RECEIVER_UDP_PORT;

    for (int i = 1; i < argc; ++i)
    {
        if(std::string(argv[i]) == "--port")
            _port = std::stoi(argv[i+1]);
        
        if(std::string(argv[i]) == "--name")
            _name = std::string(argv[i+1]);
        
        if(std::string(argv[i]) == "--topic")
            _topic = std::string(argv[i+1]);
    }

    ROS_INFO_STREAM("+--------------------------------------------------"); 
    ROS_INFO_STREAM("| Node name : " << _name);
    ROS_INFO_STREAM("| ROS topic : " << _topic);
    ROS_INFO_STREAM("| Input port: " << _port);
    ROS_INFO_STREAM("+--------------------------------------------------"); 
    ROS_INFO_STREAM("");

    ros::init(argc, argv, _name); 

    ReceiverNode node;
    node.set_input_port(_port);
    node.set_topic(_topic);
    node.run();

    return 0;
}
