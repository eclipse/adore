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
#include <udp_receiver.h>
#include <strp_fau_strpmessage_receiver.h>


class ReceiverNode
{
public:
    ReceiverNode() : udp_receiver(nullptr), node_handle("~") { }

    ~ReceiverNode() {
        delete udp_receiver;
        delete receiver_strp_fau_strpmessage;
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
    
        if(buf_size == receiver_strp_fau_strpmessage->message_id()) {
            receiver_strp_fau_strpmessage->incoming_message(buf);
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
    
        receiver_strp_fau_strpmessage = new wind::wind_ros::Receiver_strp_fau_strpmessage();
        ROS_INFO_STREAM("Instantiating receiver_strp_fau_strpmessage >> message_id -> " << receiver_strp_fau_strpmessage->message_id());
    
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
        ROS_INFO_STREAM(name << ": Setting input port: " << input_port);
        return true;
    }

private:
    std::string              name;
    int                      input_port;
    ros::NodeHandle          node_handle;
    wind::comm::UDPReceiver* udp_receiver;

    wind::wind_ros::Receiver_strp_fau_strpmessage* receiver_strp_fau_strpmessage;
};

//////////////// Main ////////////////
int main(int argc, char** argv) {
    ros::init(argc, argv, "wind_receiver");
    ReceiverNode node;
    node.set_input_port(2111);
    node.run();
    return 0;
}
