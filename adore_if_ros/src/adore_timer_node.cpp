/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#include <adore_if_ros/simfactory.h>
#include <iostream>
#include <thread>
#include <cstdlib> 

bool paused;
bool terminated;
bool sendpaused;

void kbinput()
{
  while(!terminated)
  {
    int c = std::cin.get();
    if( c == 10)paused=!paused;
  }
}

int main(int argc,char **argv)
{
    paused = false;
    terminated = false;
    sendpaused = false;
    for(int i=0;i<argc;i++)if(strcmp(argv[i],"paused")==0)paused = true;
    for(int i=0;i<argc;i++)if(strcmp(argv[i],"sendpaused")==0)sendpaused = true;
    std::thread kbinput_thread(kbinput);
    ros::init(argc,argv,"adore_timer_node");
    ros::NodeHandle n;
    adore::if_ROS::SIM_Factory sim_factory(&n);
    auto writer = sim_factory.getSimulationTimeWriter();
    double dt = 0.005;
    ros::Rate r(1.0/dt);
    double t = 0.0;
    int i=0;
    while(n.ok() && !ros::isShuttingDown() )
    { 
      if(i++%1000==0)
      {
        std::cout<<"t="<<t<<", \t press enter to (un) pause\n";
      }
      if(!paused)
      {
        t += dt; 
        writer->write(t);
        // auto rt = ros::Time::now();
        // writer->write(rt.toSec());
      }
      else if (sendpaused)
      {
        writer->write(t);
      }
      r.sleep();
    }
    terminated = true;
    // kbinput_thread.join();
    std::cout<<"killall adore_timer_node\n";
    return system("killall adore_timer_node");
}