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
 *   Robert Markowski - initial API and implementation
 ********************************************************************************/

#include <adore_if_ros/envfactory.h>
#include <adore/apps/navigation.h>
#include <adore_if_ros/paramsfactory.h>
#include <adore/if_xodr/xodr2borderbased.h>
#include <iostream>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"adore_navigation_node");
    ros::NodeHandle n;
    ROS_INFO("adore_navigation_node namespace is: %s",n.getNamespace().c_str());
    adore::if_ROS::ENV_Factory env_factory(&n);
    adore::if_ROS::PARAMS_Factory params_factory(n,"");
    
    /* process track parameter, multiple paths to maps delimited by semicolon, comma separated additional configuration */
    std::string trackConfigs = "";
    n.getParam("PARAMS/track", trackConfigs);
    adore::apps::Navigation::Config config;
    n.getParam("PARAMS/translation_x",config.trans_x_);
    n.getParam("PARAMS/translation_y",config.trans_y_);
    n.getParam("PARAMS/translation_z",config.trans_z_);
    n.getParam("PARAMS/rotation_x",config.rot_x_);
    n.getParam("PARAMS/rotation_y",config.rot_y_);
    n.getParam("PARAMS/rotation_psi",config.rot_psi_);    
    auto navigation = adore::apps::Navigation(&env_factory,&params_factory,trackConfigs, config);

    ros::Rate r(2.0);
    while (n.ok())
    {
        ros::spinOnce();
        navigation.run();
        r.sleep();
    }
    return 0;
}