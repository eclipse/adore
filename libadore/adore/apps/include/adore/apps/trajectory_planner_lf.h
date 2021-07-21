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
 *   Daniel Heß - trajectory planner for lane following maneuvers
 ********************************************************************************/


#pragma once

#include <adore/fun/afactory.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/fun/tac/basiclanefollowingplanner.h>
#include <adore/fun/tac/basicmrmplanner.h>
#include <adore/env/borderbased/lanefollowingview.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/traffic/trafficmap.h>
#include <adore/env/navigationgoalobserver.h>
#include <adore/env/tcd/connectionsonlane.h>
#include <adore/fun/safety/setpointrequestswath.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <adore/fun/tac/basicsetpointrequestevaluators.h>
#include <adore/env/traffic/decoupledtrafficpredictionview.h>
#include <adore/apps/trajectory_planner_base.h>
#include <adore/fun/safety/setpointrequestswath.h>


namespace adore
{
  namespace apps
  {
    /**
     * @brief Decoupled trajectory planner, which uses TrajectoryPlannerBase to compute and provide a PlanningResult in the event of a PlanningRequest
     */
    class TrajectoryPlannerLF:public TrajectoryPlannerBase
    {
      private:
      typedef adore::fun::BasicLaneFollowingPlanner<20,5> TNominalPlanner;
      typedef adore::fun::BasicMRMPlanner<20,5> TEmergencyPlanner;
      TNominalPlanner* nominal_planner_;
      TEmergencyPlanner* emergency_planner_;

      adore::params::APVehicle* pvehicle_;
      adore::params::APTacticalPlanner* pTacticalPlanner_;
      adore::params::APTrajectoryGeneration* pTrajectoryGeneration_;
      adore::env::NavigationGoalObserver ngo_;
      adore::env::ControlledConnectionSet4Ego connectionSet_;/**< state of controlled connections in area*/
      adore::env::ControlledConnectionSet4Ego checkPointSet_;/**< state of checkPoints in area*/
      adore::env::ConnectionsOnLane* connectionsOnLane_;/** map controlled connections to lane*/
      adore::env::ConnectionsOnLane* checkPointsOnLane_;/** map controlled connections to lane*/
      adore::env::ThreeLaneViewDecoupled three_lanes_;/**<lane-based representation of environment*/
      adore::env::DecoupledTrafficPredictionView prediction_;/**<collision detection based representation of traffic*/

      /**
       * combined maneuver post-processing constraints
       */
      adore::fun::SPRInvariantCollisionFreedom collision_detection_;/**<collision detection with traffic predictions*/


      int id_;/**<integral id to be written to PlanningResult*/
      std::string plannerName_;/**human readable planner name written to PlanningResult*/
      public:
      virtual ~TrajectoryPlannerLF()
      {
        delete nominal_planner_;
        delete emergency_planner_;
      }
      TrajectoryPlannerLF(int id=0,std::string plannerName = "lane-following"):
           connectionSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getControlledConnectionFeed()),
           checkPointSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getCheckPointFeed()),
           ngo_(adore::env::EnvFactoryInstance::get(),three_lanes_.getCurrentLane(),0,0),
           prediction_(),
           collision_detection_(&prediction_)
      {
        id_ = id;
        plannerName_ = plannerName;
        pvehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
        pTacticalPlanner_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        pTrajectoryGeneration_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
        auto pLongitudinalPlanner = adore::params::ParamsFactoryInstance::get()->getLongitudinalPlanner();
        auto pLateralPlanner = adore::params::ParamsFactoryInstance::get()->getLateralPlanner();
        auto pTacticalPlanner = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        connectionsOnLane_ = new adore::env::ConnectionsOnLane(three_lanes_.getCurrentLane(),&connectionSet_);
        checkPointsOnLane_ = new adore::env::ConnectionsOnLane(three_lanes_.getCurrentLane(),&checkPointSet_);
        //create nominal planner and add additional constraints
        nominal_planner_ = new TNominalPlanner(
                                three_lanes_.getCurrentLane(),
                                &ngo_,
                                connectionsOnLane_,
                                checkPointsOnLane_,
                                pLongitudinalPlanner,
                                pLateralPlanner,
                                pTacticalPlanner,
                                pvehicle_,
                                pTrajectoryGeneration_);

        //create emergency planner 
        emergency_planner_ = new TEmergencyPlanner(three_lanes_.getCurrentLane(),
                          pLateralPlanner,
                          pTacticalPlanner,
                          pvehicle_,
                          pTrajectoryGeneration_);
        emergency_planner_->setJMax(10.0);
        emergency_planner_->setTStall(0.1);
        emergency_planner_->setAStall(-3.0);
        emergency_planner_->setAMin(-3.0);
        emergency_planner_->setSecondAttempt(false);

      }
      void setSpeedScale(double value)
      {
        nominal_planner_->setSpeedScale(value);
      }
      /**
       * @brief update data, views and recompute maneuver
       * 
       */
      virtual void computeTrajectory(const adore::fun::PlanningRequest& planning_request, adore::fun::PlanningResult& planning_result) override
      {
        //document planner result
        planning_result.id = id_; // in lfbehavior, there is only one maneuver
        planning_result.name = plannerName_;
        planning_result.maneuver_type = adore::fun::PlanningResult::NOMINAL_DRIVING;
        planning_result.iteration = planning_request.iteration;
        planning_result.nominal_maneuver_valid = false;
        planning_result.combined_maneuver_valid = false;

        three_lanes_.update();
        auto current = three_lanes_.getCurrentLane();
        ngo_.update();
        connectionSet_.update();
        connectionsOnLane_->update();
        checkPointSet_.update();
        checkPointsOnLane_->update();

        if(!current->isValid())
        {
            planning_result.status_string = "current lane invalid";
            return;
        }

        prediction_.update();

        auto x0=planning_request.initial_state.toMotionState();
        nominal_planner_->compute(x0);
        if(!nominal_planner_->hasValidPlan())
        {
          planning_result.status_string = "nominal maneuver planning failed, "+ nominal_planner_->getStatus();
          return;
        }

        nominal_planner_->getSetPointRequest()->copyTo(planning_result.nominal_maneuver);

        adore::fun::RestartEffort re;  //test whether trajectory is long enough to justify restarting from ~0
        if(!re.isValid(planning_result.nominal_maneuver))
        {
          planning_result.status_string = "not restarting, maneuver too short";
          return;
        }

        planning_result.nominal_maneuver_valid = true;
        auto x0em = nominal_planner_->getSetPointRequest()->interpolateSetPoint(planning_request.t_emergency_start,pvehicle_);
        emergency_planner_->compute(x0em.toMotionState());

        if(!emergency_planner_->hasValidPlan() ) 
        {
          planning_result.status_string += "emergency maneuver planning failed";
          return;
        }

        nominal_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,0);
        planning_result.combined_maneuver.removeAfter(planning_request.t_emergency_start);
        planning_result.combined_maneuver.setPoints.back().tEnd = planning_request.t_emergency_start;
        emergency_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,1);

        double front_buffer_space = 4.0;
        double lateral_precision = 0.05;
        adore::fun::SetPointRequestSwath spr_swath(
            pvehicle_->get_a()+pvehicle_->get_b()+pvehicle_->get_c()+pvehicle_->get_d()+front_buffer_space,
            pvehicle_->get_bodyWidth(),
            pvehicle_->get_d(),//spr reference point at rear axle
            lateral_precision);
        spr_swath.setLonError(0.0);
        spr_swath.setLatError(0.0);
        spr_swath.append_cylinder_swath_linear(planning_result.combined_maneuver,planning_result.combined_maneuver_swath);
        spr_swath.append_cylinder_swath_linear(planning_result.nominal_maneuver,planning_result.nominal_maneuver_swath);



        if(!collision_detection_.isValid(planning_result.combined_maneuver))
        {
            planning_result.status_string = "collision detected for combined maneuver";    
            return;          
        }

        planning_result.combined_maneuver.cropAfterFirstStop(0.1);
        if(planning_result.combined_maneuver.setPoints.size()==0)
        {
          planning_result.status_string = "stopping";
          return;
        }

        planning_result.combined_maneuver_valid = true;
        int laneid = 0;
        adore::fun::SPRNavigationCostOnLane navcostOnLane(&three_lanes_,pTacticalPlanner_,laneid);
        planning_result.objective_values.insert({navcostOnLane.getName(),
                                                 navcostOnLane.getCost(planning_result.nominal_maneuver)});

        adore::fun::SPRNormalizedNavigationCost navcostNormalized(&three_lanes_,pTacticalPlanner_,laneid);
        planning_result.objective_values.insert({navcostNormalized.getName(),
                                                 navcostNormalized.getCost(planning_result.nominal_maneuver)});

        adore::fun::SPRLongitudinalAcceleration2Cost lacccost;
        planning_result.objective_values.insert({lacccost.getName(),
                                                 lacccost.getCost(planning_result.nominal_maneuver)});

        adore::fun::SPRLongitudinalJerk2Cost ljerkcost;
        planning_result.objective_values.insert({ljerkcost.getName(),
                                                 ljerkcost.getCost(planning_result.nominal_maneuver)});

        adore::fun::SPRAverageProgressLoss progressLoss(pTacticalPlanner_);
        planning_result.objective_values.insert({progressLoss.getName(),
                                                 progressLoss.getCost(planning_result.nominal_maneuver)});

      }
    };
  }
}