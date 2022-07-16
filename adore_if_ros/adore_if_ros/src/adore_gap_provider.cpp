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

#include <adore_if_ros/baseapp.h>
#include <adore/apps/gap_provider.h>

namespace adore
{
    namespace if_ROS
    {
        class GapProviderNode : public Baseapp
        {
        public:
            adore::apps::GapProvider *provider_;
            GapProviderNode() {}
            void init(int argc, char **argv, double rate, std::string nodename)
            {
                Baseapp::init(argc, argv, rate, nodename);
                Baseapp::initSim();
                provider_ = new adore::apps::GapProvider();

                // timer callbacks
                std::function<void()> run_fcn(std::bind(&adore::apps::GapProvider::update, provider_));
                Baseapp::addTimerCallback(run_fcn);
            }
        };
    } // namespace if_ROS
} // namespace adore

int main(int argc, char **argv)
{
    auto node = new adore::if_ROS::GapProviderNode();
    node->init(argc, argv, 10.0, "gap_provider");   
    node->run();
    return 0;
}