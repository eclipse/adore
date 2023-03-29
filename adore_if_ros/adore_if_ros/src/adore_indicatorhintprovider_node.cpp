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

#include <adore/apps/indicator_hints_provider.h>
#include <adore_if_ros/factorycollection.h>
#include <adore_if_ros_scheduling/baseapp.h>

namespace adore
{
namespace if_ROS
{
class IndicatorHintsProviderNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
{
public:
    adore::apps::IndicatorHintsProvider *app_;
    IndicatorHintsProviderNode() {}
    void init(int argc, char **argv, double rate, std::string nodename)
    {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        int simulationID = 0;
        getParam("simulationID", simulationID);

        std::string indicator_hint_file_path = "";
        getParam("PARAMS/indicatorFile", indicator_hint_file_path);

        app_ = new adore::apps::IndicatorHintsProvider(indicator_hint_file_path);
        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::IndicatorHintsProvider::run, app_));
        Baseapp::addTimerCallback(run_fcn);
    }
};
} // namespace if_ROS
} // namespace adore

int main(int argc, char **argv)
{
    adore::if_ROS::IndicatorHintsProviderNode app_node;
    app_node.init(argc, argv, 2.0, "adore_indicatorhintprovider_node");
    ROS_INFO("adore_indicatorhintprovider_node namespace is: %s", app_node.getRosNodeHandle()->getNamespace().c_str());
    app_node.run();
    return 0;
}
