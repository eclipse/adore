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

#include <adore_if_ros/baseapp.h>
#include <adore/apps/prediction_provider.h>

namespace adore
{
    namespace if_ROS
    {
        class PredictionProviderNode : public Baseapp
        {
        public:
            adore::apps::PredictionProvider *provider_;
            PredictionProviderNode() {}
            void init(int argc, char **argv, double rate, std::string nodename)
            {
                Baseapp::init(argc, argv, rate, nodename);
                Baseapp::initSim();
                provider_ = new adore::apps::PredictionProvider();

                // timer callbacks
                std::function<void()> run_fcn(std::bind(&adore::apps::PredictionProvider::run, provider_));
                Baseapp::addTimerCallback(run_fcn);
            }
        };
    } // namespace if_ROS
} // namespace adore

int main(int argc, char **argv)
{
    auto prediction_node = new adore::if_ROS::PredictionProviderNode();
    prediction_node->init(argc, argv, 10.0, "prediction_provider");   
    prediction_node->run();
    return 0;
}