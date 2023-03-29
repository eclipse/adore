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

#include <adore/apps/map_provider.h>
#include <adore/if_xodr/xodr2borderbased.h>
#include <iostream>
#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>

namespace adore
{
namespace if_ROS
{
class MapProviderNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
{
public:
    adore::apps::MapProvider *mp_;
    MapProviderNode() {}
    void init(int argc, char **argv, double rate, std::string nodename)
    {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        int simulationID = 0;
        getParam("simulationID", simulationID);

        /* process track parameter, multiple paths to maps delimited by semicolon, comma separated additional configuration */
        std::string trackConfigs = "";
        getParam("PARAMS/track", trackConfigs);

        std::string precedenceFile;
        getParam("PARAMS/precedence", precedenceFile);
        ROS_INFO("loading precedence: %s", precedenceFile.c_str());
        adore::env::PrecedenceSet precedenceSet;
        precedenceSet.readFile(precedenceFile);

        adore::apps::MapProvider::Config config;
        getParam("PARAMS/translation_x",config.trans_x_);
        getParam("PARAMS/translation_y",config.trans_y_);
        getParam("PARAMS/translation_z",config.trans_z_);
        getParam("PARAMS/rotation_x",config.rot_x_);
        getParam("PARAMS/rotation_y",config.rot_y_);
        getParam("PARAMS/rotation_psi",config.rot_psi_);
        mp_ = new adore::apps::MapProvider(trackConfigs, &precedenceSet, config);
        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::MapProvider::run, mp_));
        Baseapp::addTimerCallback(run_fcn);
    }
};
} // namespace if_ROS
} // namespace adore

int main(int argc, char **argv)
{
    adore::if_ROS::MapProviderNode mpn;
    mpn.init(argc, argv, 2.0, "adore_mapprovider_node");
    ROS_INFO("adore_mapprovider_node namespace is: %s", mpn.getRosNodeHandle()->getNamespace().c_str());
    mpn.run();
    return 0;
}
