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
#include <adore/apps/trajectory_planner_lc.h>
#include <ros/console.h>

namespace adore
{
  namespace if_ROS
  {  
    class TrajectoryPlannerLCNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      adore::apps::TrajectoryPlannerLC* planner_;
      void init(int argc, char **argv, double rate, std::string nodename,bool directionLeft,std::string name,int id,double speed_scale,double lateral_i_grid,double const_penalty,bool em_continue_active,bool em_cancel_active)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        planner_ = new adore::apps::TrajectoryPlannerLC(directionLeft,name,id,lateral_i_grid);
        planner_->setSpeedScale(speed_scale);
        planner_->setConstPenalty(const_penalty);
        planner_->setEMContinueActive(em_continue_active);
        planner_->setEMCancelActive(em_cancel_active);


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
    adore::if_ROS::TrajectoryPlannerLCNode lcn;
    ros::init(argc,argv,"adore_trajectory_planner_lc_node");  // TODO ros init twice, once here, once in baseapp
    ros::NodeHandle nh("~");
    std::string direction;
    if(!nh.getParam("direction",direction)||(direction.compare("left")!=0 && direction.compare("right")!=0))
    {
        ROS_DEBUG("No direction given to adore_trajectory_planner_lc_node node, terminating.");
        ROS_DEBUG("Supply <param name='direction' type='string' value='left' or value='right'/>");
        return 1;
    }
    std::string name;
    if(!nh.getParam("name",name))
    {
        ROS_DEBUG("No name given to adore_trajectory_planner_lc_node node, terminating.");
        ROS_DEBUG("Supply <param name='name' type='string' value='unique name string'/>");
        return 1;
    }
    int id;
    if(!nh.getParam("id",id))
    {
        ROS_DEBUG("No id given to adore_trajectory_planner_lc_node node, terminating.");
        ROS_DEBUG("Supply unique integral value <param name='id' type='int' value='0'/>");
        return 1;
    }
    double speed_scale = 1.0;
    if(!nh.getParam("speed_scale",speed_scale))
    {
        ROS_DEBUG("No speed scale given to adore_trajectory_planner_lc_node node.");
        ROS_DEBUG("Supply value speed_scale\\in[0.0,1.0]<param name='speed_scale' type='double' value='1.0'/>");
    }
    double lateral_i_grid = 0.0;
    if(!nh.getParam("lateral_i_grid",lateral_i_grid))
    {
        ROS_DEBUG("No lateral grid index given to adore_trajectory_planner_lc_node node.");
        ROS_DEBUG("Supply lateral_i_grid\\in\\mathbb{R} <param name='lateral_i_grid' type='double' value='0.0'/>");
    }
    double const_penalty = 0.0;
    if(!nh.getParam("const_penalty",const_penalty))
    {
        ROS_DEBUG("No const penalty given to adore_trajectory_planner_lc_node node.");
        ROS_DEBUG("Supply const_penalty\\in\\mathbb{R} <param name='const_penalty' type='double' value='0.1'/> to set const_penalty cost term in planning results to given value.");
    }
    bool em_continue_active = true;
    if(!nh.getParam("em_continue_active",em_continue_active))
    {
        ROS_DEBUG("No em_continue_active penalty given to adore_trajectory_planner_lc_node node.");
        ROS_DEBUG("Optionally supply boolean em_continue_active <param name='em_continue_active' type='bool' value='false'/> to de-activate planning of emergency maneuvers that continue lane change.");
    }
    bool em_cancel_active = false;
    if(!nh.getParam("em_cancel_active",em_cancel_active))
    {
        ROS_DEBUG("No em_cancel_active penalty given to adore_trajectory_planner_lc_node node.");
        ROS_DEBUG("Optionally supply boolean em_cancel_active <param name='em_cancel_active' type='bool' value='true'/> to activate planning of emergency maneuvers that cancel lane change.");
    }
    lcn.init(argc, argv, 100.0, "adore_trajectory_planner_lc_node",direction.compare("left")==0,name,id,speed_scale,lateral_i_grid,const_penalty,em_continue_active,em_cancel_active);
    lcn.run();
    return 0;
}