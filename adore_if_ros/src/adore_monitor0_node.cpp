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

#include <adore_if_ros/envfactory.h>
#include <adore_if_ros/simfactory.h>
#include <adore_if_ros/paramsfactory.h>
#include <adore/apps/monitor0.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"adore_monitor0_node");
    ros::NodeHandle n;
    adore::if_ROS::ENV_Factory env_factory(&n);
    adore::if_ROS::SIM_Factory sim_factory(&n);
    adore::if_ROS::PARAMS_Factory params_factory(n,"");

    
    int simulationID = 0;
    n.getParam("simulationID",simulationID);
    double rate = 40.0;
    {
      ros::NodeHandle np("~");
      np.getParam("rate",rate);
    }
    ros::Rate r(rate);

    adore::apps::Monitor0 monitor(&env_factory,&sim_factory,simulationID);

    while (n.ok())
    {
        ros::spinOnce();
        monitor.run();
        r.sleep();
    }
    return 0;
}