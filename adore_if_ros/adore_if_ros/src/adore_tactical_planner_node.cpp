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

#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>
#include <adore/apps/tactical_planner.h>
#include <iostream>

namespace adore
{
  namespace if_ROS
  {  
    class TacticalPlannerNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      adore::apps::TacticalPlanner* tacticalPlanner_;
      TacticalPlannerNode(){}
      void split(const std::string &s, char delim,std::vector<std::string>& result) 
      {
          std::stringstream ss(s);
          std::string item;

          while (std::getline(ss, item, delim)) 
          {
              result.push_back(item);
          }
      }      
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        tacticalPlanner_ = new adore::apps::TacticalPlanner(1.0/rate); 
        std::string objective_names_string;
        std::string objective_weights_string;
        std::vector<std::string> objective_names;
        std::vector<std::string> objective_weights;
        
        ros::NodeHandle np("~");
        np.getParam("objective_names", objective_names_string);
        np.getParam("objective_weights", objective_weights_string);
        std::cout<<"objective_names="<<objective_names_string<<std::endl;
        std::cout<<"objective_weights="<<objective_weights_string<<std::endl;
        split(objective_names_string,',',objective_names);
        split(objective_weights_string,',',objective_weights);
        
        for(int i=0;i<std::min(objective_names.size(),objective_weights.size());i++)
        {
          std::cout<<"adore_tactical_planner_node: adding objective "<<objective_names[i]<<" with weight "<<objective_weights[i]<<std::endl;
          tacticalPlanner_->getEvaluator()->addParameterPair(objective_names[i],objective_weights[i]);
        }

        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::TacticalPlanner::run,tacticalPlanner_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::TacticalPlannerNode lfn;
    lfn.init(argc, argv, 10, "adore_tactical_planner_node");
    lfn.run();
    return 0;
}