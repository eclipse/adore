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
#include <adore_if_ros/baseapp.h>

namespace adore
{
namespace if_ROS
{
class MapProviderNode : public Baseapp
{
public:
    adore::apps::MapProvider *mp_;
    MapProviderNode() {}
    void init(int argc, char **argv, double rate, std::string nodename)
    {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        int simulationID = 0;
        getParam("simulationID", simulationID);

        // borderset
        adore::env::BorderBased::BorderSet globalSet;

        /* process track parameter, multiple paths to maps delimited by semicolon, comma separated additional configuration */
        std::string trackConfigs = "";
        std::string trackConfig = "";
        getParam("PARAMS/track", trackConfigs);
        std::stringstream trackstream(trackConfigs);

        while (std::getline(trackstream, trackConfig, ';'))
        {
            /* reading of single track configuration, comma separated */
            std::stringstream trackConfigStream(trackConfig);
            bool transform = false;
            std::string filename = "";
            std::string token = "";
            while (std::getline(trackConfigStream, token, ','))
            {
                if (token.compare("transform") == 0)
                {
                    transform = true;
                }
                else
                {
                    filename = token;
                }
            }

            /* process current file */
            ROS_INFO("loading track: %s", filename.c_str());
            adore::if_xodr::XODR2BorderBasedConverter converter;
            adore::env::BorderBased::BorderSet partialSet;
            try
            {
                converter.convert(filename.c_str(), &partialSet, transform);
            }
            catch (...)
            {
                ROS_ERROR("Could not parse file %s", filename.c_str());
                return;
            }
            /* add partial map to global map */
            auto its = partialSet.getAllBorders();
            for (; its.first != its.second; its.first++)
            {
                globalSet.insert_border(its.first->second);
            }
            /* global map has responsibility for object/pointers */
            partialSet.setIsOwner(false);
        }

        // precedences
        std::string precedenceFile;
        getParam("PARAMS/precedence", precedenceFile);
        std::cout << "loading precedence: " << precedenceFile << "\n";
        std::cout << std::flush;
        adore::env::PrecedenceSet precedenceSet;
        precedenceSet.readFile(precedenceFile);

        mp_ = new adore::apps::MapProvider(getFactory<ENV_Factory>(), getParamsFactory(""), &globalSet, &precedenceSet);
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
