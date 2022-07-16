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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#include <adore/apps/speedlimit_provider.h>
#include <adore_if_ros/baseapp.h>

namespace adore
{
namespace if_ROS
{
class SpeedLimitProviderNode : public Baseapp
{
public:
    adore::apps::SpeedLimitProvider *app_;
    SpeedLimitProviderNode() {}
    void init(int argc, char **argv, double rate, std::string nodename)
    {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        int simulationID = 0;
        getParam("simulationID", simulationID);

        std::string speed_limit_file_path = "";
        getParam("PARAMS/speedLimitFile", speed_limit_file_path);

        app_ = new adore::apps::SpeedLimitProvider(speed_limit_file_path);
        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::SpeedLimitProvider::run, app_));
        Baseapp::addTimerCallback(run_fcn);
    }
};
} // namespace if_ROS
} // namespace adore

int main(int argc, char **argv)
{
    adore::if_ROS::SpeedLimitProviderNode app_node;
    app_node.init(argc, argv, 2.0, "adore_speedlimitprovider_node");
    ROS_INFO("adore_speedlimitprovider_node namespace is: %s", app_node.getRosNodeHandle()->getNamespace().c_str());
    app_node.run();
    return 0;
}
