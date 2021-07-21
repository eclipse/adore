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

#ifndef WIND_ROS_RECEIVER_NODE
#define WIND_ROS_RECEIVER_NODE
#endif

#include <wind_constants.h>
#include <ros/ros.h>
#include <udp_receiver.h>
#include <odds_oddinfo_receiver.h>


class ReceiverNode
{
public:
    ReceiverNode() : udp_receiver(nullptr), node_handle("~") { }

    ~ReceiverNode() {
        delete udp_receiver;
        delete receiver_odds_oddinfo;
    }

    void receiveMsgCallback(const void* buf, const size_t& buf_size)
    {
        ROS_DEBUG("Called receiveMsgViaUDP().");
        ROS_INFO_STREAM(name << ": Incoming Msg (" << buf_size << " bytes)");
    
        uint16_t* endianness = (uint16_t*)buf;
        if(*endianness != 1) {
            ROS_INFO_STREAM("Ignoring incoming message: Wrong endianness.");
            return;
        }
    
        if(buf_size == receiver_odds_oddinfo->message_id()) {
            receiver_odds_oddinfo->incoming_message(buf);
        }
        else {
            ROS_ERROR_STREAM("Received UDP Msg on Port "
                << input_port << " with no corresponding Wind message("
                << buf_size << ")");
        }
    }

    void run()
    {
        if (!getParameters()) return;
    
        ROS_INFO_STREAM("Starting ROS node " << name << ".");
    
        receiver_odds_oddinfo = new wind::wind_ros::Receiver_odds_oddinfo();
        ROS_INFO_STREAM("Instantiating receiver_odds_oddinfo >> message_id -> " << receiver_odds_oddinfo->message_id());
    
        ROS_INFO_STREAM(name << ": Starting UDP server in port: " << input_port);
        udp_receiver = new wind::comm::UDPReceiver(input_port);
        udp_receiver->setReceiveCallbackFunction(boost::bind(&ReceiverNode::receiveMsgCallback, this, _1, _2));
        udp_receiver->startReading();
    
        ros::spin();
    }

    void set_input_port(int port) {
        input_port = port;
    }

protected:
    bool getParameters()
    {
        name = ros::this_node::getName();
        ROS_INFO_STREAM(RECEIVER_NODE_NAME << ": Setting input port: " << RECEIVER_UDP_PORT);
        return true;
    }

private:
    std::string              name;
    int                      input_port;
    ros::NodeHandle          node_handle;
    wind::comm::UDPReceiver* udp_receiver;

    wind::wind_ros::Receiver_odds_oddinfo* receiver_odds_oddinfo;
};

//////////////// Main ////////////////
int main(int argc, char** argv) {
    ros::init(argc, argv, RECEIVER_NODE_NAME); // this value is defined in wind_constants.h
    ReceiverNode node;
    node.set_input_port(RECEIVER_UDP_PORT);    // this value is defined in wind_constants.h
    node.run();
    return 0;
}
