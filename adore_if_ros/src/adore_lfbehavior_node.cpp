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
#include <adore/apps/lane_following_behavior.h>


namespace adore
{
  namespace if_ROS
  {  
    class LaneFollowingBehaviorNode : public Baseapp
    {
      public:
      adore::apps::LaneFollowingBehavior* lf_;
      LaneFollowingBehaviorNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        lf_ = new adore::apps::LaneFollowingBehavior(this->getFactory<ENV_Factory>(),this->getFactory<FUN_Factory>(),this->getParamsFactory(""));

        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::LaneFollowingBehavior::run,lf_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::LaneFollowingBehaviorNode lfn;
    lfn.init(argc, argv, 10.0, "adore_lfbehavior_node");
    lfn.run();
    return 0;
}