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
#include <adore/apps/trajectory_planner_lf.h>


namespace adore
{
  namespace if_ROS
  {  
    class TrajectoryPlannerLFNode : public Baseapp
    {
      public:
      adore::apps::TrajectoryPlannerLF* planner_;
      void init(int argc, char **argv, double rate, std::string nodename, std::string name,int id,double speed_scale)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        planner_ = new adore::apps::TrajectoryPlannerLF(id,name);
        planner_->setSpeedScale(speed_scale);

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
    ros::init(argc,argv,"adore_trajectory_planner_lf_node");  // TODO ros init twice, once here, once in baseapp
    ros::NodeHandle nh("~");
    std::string name;
    if(!nh.getParam("name",name))
    {
        ROS_DEBUG("No name given to adore_trajectory_planner_lf_node node, terminating.");
        ROS_DEBUG("Supply <param name='name' type='string' value='unique name string'/>");
        return 1;
    }
    int id;
    if(!nh.getParam("id",id))
    {
        ROS_DEBUG("No id given to adore_trajectory_planner_lf_node node, terminating.");
        ROS_DEBUG("Supply unique integral value <param name='id' type='int' value='0'/>");
        return 1;
    }
    double speed_scale = 1.0;
    if(!nh.getParam("speed_scale",speed_scale))
    {
        ROS_DEBUG("No speed scale given to adore_trajectory_planner_lf_node node, terminating.");
        ROS_DEBUG("Supply value speed_scale\\in[0.0,1.0]<param name='speed_scale' type='double' value='1.0'/>");
        return 1;
    }

    adore::if_ROS::TrajectoryPlannerLFNode lfn;
    lfn.init(argc, argv, 100.0, "adore_trajectory_planner_lf_node",name,id,speed_scale);/* (rate only affects polling when use_scheduler==true) */
    lfn.run();
    return 0;
}