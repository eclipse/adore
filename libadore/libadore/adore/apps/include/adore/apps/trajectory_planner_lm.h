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
#include <adore/fun/tac/basicmergeplanner.h>
#include <adore/fun/tac/mergemrmplanner.h>
#include <adore/env/navigationgoalobserver.h>
#include <adore/env/tcd/connectionsonlane.h>
#include <adore/fun/safety/setpointrequestswath.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <adore/fun/tac/basicsetpointrequestevaluators.h>
#include <adore/env/traffic/decoupledtrafficpredictionview.h>
#include <adore/apps/trajectory_planner_base.h>
#include <adore/env/traffic/emptygap.h>


namespace adore
{
  namespace apps
  {
    /**
     * @brief Decoupled trajectory planner, which uses TrajectoryPlannerBase to compute and provide a PlanningResult in the event of a PlanningRequest
     * Computes a lane change trajectory
     */
    class TrajectoryPlannerLM:public TrajectoryPlannerBase
    {
      private:
      typedef adore::fun::BasicMergePlanner<20,5> TNominalPlanner;
      typedef adore::fun::MergeMRMPlanner<20,5> TEmergencyPlanner;
      TNominalPlanner* nominal_planner_;
      TEmergencyPlanner* emergency_planner_;//switched direction wrt to lc

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
      std::string plannerName_;/**<human readable planner name written to PlanningResult*/
      bool directionLeft_;/**<true if lane change to the left*/
      public:
      virtual ~TrajectoryPlannerLM()
      {
        delete nominal_planner_;
        delete emergency_planner_;
      }
      TrajectoryPlannerLM(bool directionLeft,std::string name,int id):
           connectionSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getControlledConnectionFeed()),
           checkPointSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getCheckPointFeed()),
           ngo_(adore::env::EnvFactoryInstance::get(),three_lanes_.getCurrentLane(),0,0),
           prediction_(),
           collision_detection_(&prediction_),
           directionLeft_(directionLeft)
      {
        id_ = id;
        plannerName_ = name;
        pvehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
        pTacticalPlanner_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        pTrajectoryGeneration_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
        auto pLongitudinalPlanner = adore::params::ParamsFactoryInstance::get()->getLongitudinalPlanner();
        auto pLateralPlanner = adore::params::ParamsFactoryInstance::get()->getLateralPlanner();
        auto pTacticalPlanner = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        connectionsOnLane_ = new adore::env::ConnectionsOnLane(three_lanes_.getCurrentLane(),&connectionSet_);
        checkPointsOnLane_ = new adore::env::ConnectionsOnLane(three_lanes_.getCurrentLane(),&checkPointSet_);
        auto lcv =  directionLeft
                        ?three_lanes_.getLeftLaneChange()
                        :three_lanes_.getRightLaneChange();
        //create nominal planner and add additional constraints
        nominal_planner_ = new TNominalPlanner(
                                lcv,
                                &ngo_,
                                connectionsOnLane_,
                                checkPointsOnLane_,
                                pLongitudinalPlanner,
                                pLateralPlanner,
                                pTacticalPlanner,
                                pvehicle_,
                                pTrajectoryGeneration_);

        //create emergency planner 
        emergency_planner_ = new TEmergencyPlanner(lcv,
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
      /**
       * @brief update data, views and recompute maneuver
       * 
       */
      virtual void computeTrajectory(const adore::fun::PlanningRequest& planning_request, adore::fun::PlanningResult& planning_result) override
      {
        //document planner result
        planning_result.id = id_; 
        planning_result.name = plannerName_;
        planning_result.maneuver_type = adore::fun::PlanningResult::NOMINAL_DRIVING;
        planning_result.iteration = planning_request.iteration;
        planning_result.nominal_maneuver_valid = false;
        planning_result.combined_maneuver_valid = false;

        three_lanes_.update();
        ngo_.update();
        connectionSet_.update();
        connectionsOnLane_->update();
        checkPointSet_.update();
        checkPointsOnLane_->update();
        auto current = three_lanes_.getCurrentLane();
        auto lcv = directionLeft_
                        ?three_lanes_.getLeftLaneChange()
                        :three_lanes_.getRightLaneChange();
        auto target = lcv->getSourceLane();//(switched direction wrt to lc)
        auto source = lcv->getTargetLane();//(switched direction wrt to lc)

        if(!current->isValid())
        {
            planning_result.status_string = "target lane invalid";
            return;
        }
        if(!source->isValid())
        {
            planning_result.status_string = "source lane invalid";
            return;
        }
        if(lcv->getNavigationCostDifference()<0)
        {
            planning_result.status_string = "lm not required for navigation";
            return;
        }

        prediction_.update();

        nominal_planner_->compute(planning_request.initial_state.toMotionState());
        if(!nominal_planner_->hasValidPlan())
        {
          planning_result.status_string = "nominal maneuver planning failed, "+ nominal_planner_->getStatus();
          return;
        }

        nominal_planner_->getSetPointRequest()->copyTo(planning_result.nominal_maneuver);
        
        adore::fun::RestartEffort re;  
        if(!re.isValid(planning_result.nominal_maneuver))
        {
          planning_result.status_string = "not restarting, maneuver too short";
          return;
        }


        planning_result.nominal_maneuver_valid = true;
        auto x0em = nominal_planner_->getSetPointRequest()->interpolateSetPoint(planning_request.t_emergency_start,pvehicle_);
        bool valid_em_found = false;

        emergency_planner_->compute(x0em.toMotionState());
        if(emergency_planner_->hasValidPlan() )
        {
          nominal_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,0);
          planning_result.combined_maneuver.removeAfter(planning_request.t_emergency_start);
          planning_result.combined_maneuver.setPoints.back().tEnd = planning_request.t_emergency_start;
          emergency_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,1);
          if(collision_detection_.isValid(planning_result.combined_maneuver))
          {
            valid_em_found = true;
          }
          else
          {
            planning_result.combined_maneuver.setPoints.clear();
          }
          
        }

        if(!valid_em_found)
        {
          planning_result.status_string += "emergency maneuver planning failed";
          return;
        }

        planning_result.combined_maneuver.cropAfterFirstStop(pTacticalPlanner_->getTerminateAfterFirstStopThresholdSpeed());
        if(planning_result.combined_maneuver.setPoints.size()==0)
        {
          planning_result.status_string = "stopping";
          return;
        }
        
        planning_result.combined_maneuver_valid = true;
        int laneid = directionLeft_?1:-1;
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