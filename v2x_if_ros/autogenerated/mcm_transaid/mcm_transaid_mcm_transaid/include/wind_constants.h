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
 * Module: DENM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) denm(1) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */
#ifndef WIND_CONSTANTS_H
#define WIND_CONSTANTS_H

#define WIND_DEBUG 0

// Nodes names
#define RECEIVER_NODE_NAME       "mcm_transaid_mcm_transaid_receiver"
#define SENDER_NODE_NAME         "mcm_transaid_mcm_transaid_sender"

// Topics
#define RECEIVER_ROS_TOPIC        "v2x/incoming/MCM"
#define SENDER_ROS_TOPIC          "v2x/outgoing/MCM"
#define RECEIVER_ROS_TOPIC_MIRROR RECEIVER_ROS_TOPIC "_mirror"                          // For testing purposes
#define SENDER_ROS_TOPIC_MIRROR   "/" RECEIVER_NODE_NAME "/" RECEIVER_ROS_TOPIC_MIRROR  // For testing purposes

// Network
#define SENDER_REMOTE_IP          "127.0.0.1"   // Normally you don't need to modify this value
#define SENDER_REMOTE_UDP_PORT    1111          // Normally you don't need to modify this value
#define RECEIVER_UDP_PORT         2111

#endif //WIND_CONSTANTS_H
