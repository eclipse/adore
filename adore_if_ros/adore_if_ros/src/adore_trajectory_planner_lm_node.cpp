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
#include <adore/apps/trajectory_planner_lm.h>
#include <ros/console.h>

namespace adore
{
  namespace if_ROS
  {  
    class TrajectoryPlannerLMNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      adore::apps::TrajectoryPlannerLM* planner_;
      void init(int argc, char **argv, double rate, std::string nodename,bool directionLeft,std::string name,int id)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        planner_ = new adore::apps::TrajectoryPlannerLM(directionLeft,name,id);

        ros::NodeHandle nh("~");
        bool use_scheduler = false;
        nh.getParam("/use_scheduler",use_scheduler);
        if(use_scheduler)
        {
          std::function<void()> run_fcn(std::bind(&adore::apps::TrajectoryPlannerBase::planning_request_handler,planner_));
          Baseapp::addTimerCallback(run_fcn);
        }
        else
        {
          planner_->prime();//node is executed event-based on PlanningRequest, prime sets trigger
        }
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::TrajectoryPlannerLMNode lcn;
    ros::init(argc,argv,"adore_trajectory_planner_lm_node");  // TODO ros init twice, once here, once in baseapp
    ros::NodeHandle nh("~");
    std::string direction;
    if(!nh.getParam("direction",direction)||(direction.compare("left")!=0 && direction.compare("right")!=0))
    {
        ROS_DEBUG("No direction given to adore_trajectory_planner_lm_node node, terminating.");
        ROS_DEBUG("Supply <param name='direction' type='string' value='left' or value='right'/>");
        return 1;
    }
    std::string name;
    if(!nh.getParam("name",name))
    {
        ROS_DEBUG("No name given to adore_trajectory_planner_lm_node node, terminating.");
        ROS_DEBUG("Supply <param name='name' type='string' value='unique name string'/>");
        return 1;
    }
    int id;
    if(!nh.getParam("id",id))
    {
        ROS_DEBUG("No id given to adore_trajectory_planner_lm_node node, terminating.");
        ROS_DEBUG("Supply unique integral value <param name='id' type='int' value='0'/>");
        return 1;
    }
    lcn.init(argc, argv, 100.0, "adore_trajectory_planner_lm_node",direction.compare("left")==0,name,id);
    lcn.run();
    return 0;
}