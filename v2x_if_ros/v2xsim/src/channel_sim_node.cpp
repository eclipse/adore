/****************************************************************************/
// Eclipse ADORe, Automated Driving Open Research; see https://eclipse.org/adore
// Copyright (C) 2017-2020 German Aerospace Center (DLR).
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    channel_sim_node.cpp
/// @author  Daniel Heß
/// @contact Daniel.Hess@DLR.de
/// @date 2020/04/02
/// @initialrelease X.X.X
///
// channel_sim_node realizes v2x error model and transmission of messages 
// to/from a single station.

#include <ros/ros.h>
#include <v2xsim/SimSPATEM.h>
#include <v2xsim/SimMAPEM.h>
#include <v2xsim/SimMCM.h>
#include <v2xsim/SimDENM.h>


#include "channel2station.h"
// #include <boost/random.hpp>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"channel_sim_node");
    ros::NodeHandle n;
    v2xsim::Station station(n,"odom");
    v2xsim::Channel channel(n,&station,0);
    v2xsim::ChannelToStation<v2xsim::SimSPATEM,
                                 v2xsim::SimSPATEMConstPtr,
                                 dsrc_v2_spatem_pdu_descriptions::SPATEM,
                                 dsrc_v2_spatem_pdu_descriptions::SPATEMConstPtr>
                                channelToStation_spatem(n,
                                    "/SIM/v2x/SPATEM",
                                    "/SIM/v2x/SPATEM",
                                    "v2x/incoming/SPATEM",
                                    "v2x/outgoing/SPATEM",
                                    &station,
                                    &channel
                                    );
    v2xsim::ChannelToStation<v2xsim::SimMAPEM,
                                 v2xsim::SimMAPEMConstPtr,
                                 dsrc_v2_mapem_pdu_descriptions::MAPEM,
                                 dsrc_v2_mapem_pdu_descriptions::MAPEMConstPtr>
                                channelToStation_mapem(n,
                                    "/SIM/v2x/MAPEM",
                                    "/SIM/v2x/MAPEM",
                                    "v2x/incoming/MAPEM",
                                    "v2x/outgoing/MAPEM",
                                    &station,
                                    &channel
                                    );
    v2xsim::ChannelToStation<v2xsim::SimMCM,
                                     v2xsim:: SimMCMConstPtr,
                                    mcm_transaid_mcm_transaid::MCM,
                                    mcm_transaid_mcm_transaid::MCMConstPtr>
                                    channelToStation_mcm(n,
                                        "/SIM/v2x/MCM",
                                        "/SIM/v2x/MCM",
                                        "v2x/incoming/MCM",
                                        "v2x/outgoing/MCM",
                                    &station,
                                    &channel
                                    );
    v2xsim::ChannelToStation<   v2xsim::SimDENM,
                                v2xsim::SimDENMConstPtr,
                                denm_v2_denm_pdu_descriptions::DENM,
                                denm_v2_denm_pdu_descriptions::DENMConstPtr  >
                                channelToStation_denm(n,
                                    "/SIM/v2x/DENM",
                                    "/SIM/v2x/DENM",
                                    "v2x/incoming/DENM",
                                    "v2x/outgoing/DENM",
                                &station,
                                &channel
                                );

    ros::spin();
    return 0;
}