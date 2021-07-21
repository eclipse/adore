/*
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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:denm_v2:1.5
 * 
 * Module: DENM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) denm(1) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 */
#ifndef DENM_V2_DENM_PDU_DESCRIPTIONS_RECEIVER_H
#define DENM_V2_DENM_PDU_DESCRIPTIONS_RECEIVER_H

// File name: denm_v2_denm_pdu_descriptions_receiver.h

#include <ros/ros.h>
#include <wind_constants.h>
#include <denm_v2_denm_pdu_descriptions/DENM.h>
#include <denm_v2_denm_pdu_descriptions.h>
#include <denm_v2_denm_pdu_descriptions_translator_wind2ros.h>
#include <denm_v2_denm_pdu_descriptions_rserial_decoder.h>

namespace wind
{
namespace wind_ros
{
    class Receiver_denm_v2_denm_pdu_descriptions
    {
    public:
        Receiver_denm_v2_denm_pdu_descriptions();
        void incoming_message(const void* buf);
        void start();
        void set_topic(std::string t);
        std::string message_name();
        uint32_t message_id();
    
    private:
    
        ros::NodeHandle node_handle;
        ros::Publisher publisher;
        ros::Publisher mirrorPublisher;
        std::string topic;
        uint32_t   messagesCounter;
    };
}  // Closing namespace wind_ros
}  // Closing namespace wind

#endif //DENM_V2_DENM_PDU_DESCRIPTIONS_RECEIVER_H
