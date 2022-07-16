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
#include <adore_if_ros/funfactory.h>
#include <adore_if_ros/paramsfactory.h>
#include <adore/apps/test_mrm_planner.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"test_mrm_planner");
    ros::NodeHandle n;    
    int simulationID = 0;
    n.getParam("simulationID",simulationID);
    std::stringstream ss;
    ss<<"v"<<simulationID<<"/";
    std::string sid = ss.str();
    adore::if_ROS::ENV_Factory env_factory(&n);
    adore::if_ROS::FUN_Factory fun_factory(&n);
    adore::if_ROS::PARAMS_Factory params_factory(n,"");
    adore::apps::TestMRMPlanner testplanner(&env_factory,&fun_factory,&params_factory);

    ros::Rate r(10);
    while (n.ok())
    {
        ros::spinOnce();
        testplanner.run();
        r.sleep();
    }
}
