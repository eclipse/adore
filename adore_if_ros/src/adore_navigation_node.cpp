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
    adore::env::BorderBased::BorderSet globalSet;

    /* process track parameter, multiple paths to maps delimited by semicolon, comma separated additional configuration */
    std::string trackConfigs = "";
    std::string trackConfig = "";
    n.getParam("PARAMS/track", trackConfigs);
    std::stringstream trackstream(trackConfigs);
    
    while(std::getline(trackstream,trackConfig,';'))
    {
        /* reading of single track configuration, comma separated */
        std::stringstream trackConfigStream(trackConfig);
        bool transform = false;
        std::string filename = "";
        std::string token = "";
        while(std::getline(trackConfigStream,token,','))
        {
            if(token.compare("transform")==0)
            {
                transform = true;
            }
            else
            {
                filename = token;
            }
        }
        /* process current file */
        ROS_INFO("loading track: %s",filename.c_str());
        adore::if_xodr::XODR2BorderBasedConverter converter;
        adore::env::BorderBased::BorderSet partialSet;        
        try
        {
            converter.convert(filename.c_str(),&partialSet,transform);
        }
        catch(...)
        {
            ROS_ERROR("Could not parse file %s", filename.c_str());
            return 1;
        }
        /* add partial map to global map */
        auto its = partialSet.getAllBorders();
        for(;its.first!=its.second;its.first++)
        {
            globalSet.insert_border(its.first->second);
        }
        /* global map has responsibility for object/pointers */
        partialSet.setIsOwner(false);
    }
    
    auto navigation = adore::apps::Navigation(&env_factory,&params_factory,&globalSet);

    ros::Rate r(2.0);
    while (n.ok())
    {
        ros::spinOnce();
        navigation.run();
        r.sleep();
    }
    return 0;
}