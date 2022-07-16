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
#include <adore/apps/prediction_filter.h>

namespace adore
{
    namespace if_ROS
    {
        class PredictionFilterNode : public Baseapp
        {
        public:
            adore::apps::PredictionFilter *predictionFilter_;
            PredictionFilterNode() {}
            void init(int argc, char **argv, double rate, std::string nodename,bool worstcase)
            {
                Baseapp::init(argc, argv, rate, nodename);
                Baseapp::initSim();
                predictionFilter_ = new adore::apps::PredictionFilter(worstcase);

                // timer callbacks
                std::function<void()> run_fcn(std::bind(&adore::apps::PredictionFilter::run, predictionFilter_));
                Baseapp::addTimerCallback(run_fcn);
            }
        };
    } // namespace if_ROS
} // namespace adore

int main(int argc, char **argv)
{
    ros::init(argc,argv,"prediction_filter");  // TODO ros init twice, once here, once in baseapp
    ros::NodeHandle nh("~");
    bool worstcase=false;
    if(!nh.getParam("worstcase",worstcase))worstcase=false;
    auto predictionFilter_node = new adore::if_ROS::PredictionFilterNode();
    predictionFilter_node->init(argc, argv, 10.0, "prediction_filter",worstcase);   
    predictionFilter_node->run();
    return 0;
}