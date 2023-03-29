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


#include <adore/apps/test_straight_line_predictor.h>
#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>

namespace adore
{
  namespace if_ROS
  {  
    class TestStraightLinePredictionNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      adore::apps::TestStraightLinePredictor* app_;
      TestStraightLinePredictionNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        app_ = new adore::apps::TestStraightLinePredictor();
        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::TestStraightLinePredictor::run,app_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}
int main(int argc,char **argv)
{
    adore::if_ROS::TestStraightLinePredictionNode node;
    node.init(argc, argv, 100.0, "test_straight_line_prediction_node");
    node.run();
    return 0;
}
