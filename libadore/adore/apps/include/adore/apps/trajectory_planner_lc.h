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
#include <adore/fun/tac/basiclanechangeplanner.h>
#include <adore/fun/tac/cancellcmrmplanner.h>
#include <adore/fun/tac/continuelcmrmplanner.h>
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
#include <adore/env/traffic/emptygap.h>


namespace adore
{
  namespace apps
  {
    /**
     * @brief Decoupled trajectory planner, which uses TrajectoryPlannerBase to compute and provide a PlanningResult in the event of a PlanningRequest
     * Computes a lane change trajectory
     */
    class TrajectoryPlannerLC:public TrajectoryPlannerBase
    {
      private:
      typedef adore::fun::BasicLaneChangePlanner<20,5> TNominalPlanner;
      typedef adore::fun::CancelLCMRMPlanner<20,5> TEmergencyPlanner1;
      typedef adore::fun::ContinueLCMRMPlanner<20,5> TEmergencyPlanner2;
      TNominalPlanner* nominal_planner_;
      TEmergencyPlanner1* emergency_planner1_;
      TEmergencyPlanner2* emergency_planner2_;

      adore::env::ThreeLaneViewDecoupled three_lanes_;/**<lane-based representation of environment*/
      adore::params::APVehicle* pvehicle_;
      adore::params::APTacticalPlanner* pTacticalPlanner_;
      adore::params::APTrajectoryGeneration* pTrajectoryGeneration_;
      adore::env::NavigationGoalObserver ngo_;
      adore::env::ControlledConnectionSet4Ego connectionSet_;/**< state of controlled connections in area*/
      adore::env::ControlledConnectionSet4Ego checkPointSet_;/**< state of checkPoints in area*/
      adore::env::ConnectionsOnLane* connectionsOnLane_;/** map controlled connections to lane*/
      adore::env::ConnectionsOnLane* checkPointsOnTargetLane_;/** map controlled connections to lane*/
      adore::env::ConnectionsOnLane* checkPointsOnSourceLane_;/** map controlled connections to lane*/
      adore::env::DecoupledTrafficPredictionView prediction_;/**<collision detection based representation of traffic*/
      adore::env::EmptyGap gap_;/**<an empty gap for testing*/

      /**
       * combined maneuver post-processing constraints
       */
      adore::fun::SPRInvariantCollisionFreedom collision_detection_;/**<collision detection with traffic predictions*/

      int id_;/**<integral id to be written to PlanningResult*/
      std::string plannerName_;/**<human readable planner name written to PlanningResult*/
      bool directionLeft_;/**<true if lane change to the left*/
      bool gapTrackingActive_;/**<true if gap tracking was active in last iteration*/
      double gapX_;/**<last X position of gap*/
      double gapY_;/**<last Y position of gap*/
      double gapT_;/**<last time of iteration*/
      double gapv_;/**<last observed speed of gap*/
      double gaps_;/**<last s-progress associated with gapX_ and gapY_*/
      public:
      virtual ~TrajectoryPlannerLC()
      {
        delete nominal_planner_;
        delete emergency_planner1_;
        delete emergency_planner2_;
      }
      TrajectoryPlannerLC(bool directionLeft,std::string name,int id):
           connectionSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getControlledConnectionFeed()),
           checkPointSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getCheckPointFeed()),
           ngo_(adore::env::EnvFactoryInstance::get(),
                    directionLeft
                        ?three_lanes_.getLeftLaneChange()->getTargetLane()
                        :three_lanes_.getRightLaneChange()->getTargetLane()
                ,0,0),
           prediction_(),
           collision_detection_(&prediction_),
           directionLeft_(directionLeft),
           gapTrackingActive_(false)
      {
        id_ = id;
        plannerName_ = name;
        pvehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
        pTacticalPlanner_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        pTrajectoryGeneration_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
        auto pLongitudinalPlanner = adore::params::ParamsFactoryInstance::get()->getLongitudinalPlanner();
        auto pLateralPlanner = adore::params::ParamsFactoryInstance::get()->getLateralPlanner();
        auto pTacticalPlanner = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        auto target_lc =  directionLeft
                        ?three_lanes_.getLeftLaneChange()
                        :three_lanes_.getRightLaneChange();
        auto target = target_lc->getTargetLane();
        auto source = target_lc->getSourceLane();
        connectionsOnLane_ = new adore::env::ConnectionsOnLane(target,&connectionSet_);
        checkPointsOnTargetLane_ = new adore::env::ConnectionsOnLane(target,&checkPointSet_);
        checkPointsOnSourceLane_ = new adore::env::ConnectionsOnLane(source,&checkPointSet_);
        //create nominal planner and add additional constraints
        nominal_planner_ = new TNominalPlanner(
                                target_lc,
                                &ngo_,
                                connectionsOnLane_,
                                checkPointsOnSourceLane_,
                                checkPointsOnTargetLane_,
                                pLongitudinalPlanner,
                                pLateralPlanner,
                                pTacticalPlanner,
                                pvehicle_,
                                pTrajectoryGeneration_);
        nominal_planner_->setGap(&gap_);//assign constant empty gap for testing

        //create emergency planner 1
        emergency_planner1_ = new TEmergencyPlanner1(target_lc,
                          pLateralPlanner,
                          pTacticalPlanner,
                          pvehicle_,
                          pTrajectoryGeneration_);
        emergency_planner1_->setJMax(10.0);
        emergency_planner1_->setTStall(0.1);
        emergency_planner1_->setAStall(-3.0);
        emergency_planner1_->setAMin(-3.0);
        emergency_planner1_->setSecondAttempt(false);

        //create emergency planner 2
        emergency_planner2_ = new TEmergencyPlanner2(target_lc,
                          pLateralPlanner,
                          pTacticalPlanner,
                          pvehicle_,
                          pTrajectoryGeneration_);
        emergency_planner2_->setJMax(10.0);
        emergency_planner2_->setTStall(0.1);
        emergency_planner2_->setAStall(-3.0);
        emergency_planner2_->setAMin(-3.0);
        emergency_planner2_->setSecondAttempt(false);
        emergency_planner2_->setGap(&gap_);//assign constant empty gap for testing

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
        planning_result.indicator_left = directionLeft_;
        planning_result.indicator_right = !directionLeft_;

        three_lanes_.update();
        ngo_.update();
        connectionSet_.update();
        connectionsOnLane_->update();
        checkPointSet_.update();
        checkPointsOnTargetLane_->update();
        checkPointsOnSourceLane_->update();
        auto current = three_lanes_.getCurrentLane();
        auto target_lc = directionLeft_
                        ?three_lanes_.getLeftLaneChange()
                        :three_lanes_.getRightLaneChange();
        auto target = target_lc->getTargetLane();

        if(!current->isValid())
        {
            planning_result.status_string = "current lane invalid";
            return;
        }
        if(!target->isValid())
        {
            planning_result.status_string = "target lane invalid";
            return;
        }
        if(target_lc->getNavigationCostDifference()>pTacticalPlanner_->getMaxNavcostLoss())
        {
            planning_result.status_string = "lc not required for navigation";
            return;
        }


        prediction_.update();



        //@TODO: Gap selection
        // - store XY-coordinate for gap identification
        // - remember speed of gap and time
        // - advance XY-coordinate with speed of gap and time to get an estimate of next gap position
        // - relocate XY-coordinate in gap center
        // - compute s-porgress from XY-coordinate to to identify gap to basiclanechangeplanner
        // here: naive gap selection: always use ego coordinate
        gapX_ = planning_request.initial_state.x0ref.getX();
        gapY_ = planning_request.initial_state.x0ref.getY();
        double tmp;
        target->toRelativeCoordinates(gapX_,gapY_,gaps_,tmp);

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

        emergency_planner2_->compute(x0em.toMotionState());
        if(emergency_planner2_->hasValidPlan() )
        {
          nominal_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,0);
          planning_result.combined_maneuver.removeAfter(planning_request.t_emergency_start);
          planning_result.combined_maneuver.setPoints.back().tEnd = planning_request.t_emergency_start;
          emergency_planner2_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,1);
          if(collision_detection_.isValid(planning_result.combined_maneuver))
          {
            valid_em_found = true;
          }
          else
          {
            planning_result.combined_maneuver.setPoints.clear();
          }
          
        }

        // if(!valid_em_found)
        // {
        //   emergency_planner1_->compute(x0em.toMotionState());
        //   if(emergency_planner1_->hasValidPlan() )
        //   {
        //     nominal_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,0);
        //     planning_result.combined_maneuver.removeAfter(planning_request.t_emergency_start);
        //     planning_result.combined_maneuver.setPoints.back().tEnd = planning_request.t_emergency_start;
        //     emergency_planner1_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,1);
        //     if(collision_detection_.isValid(planning_result.combined_maneuver))
        //     {
        //       valid_em_found = true;
        //     }
        //     else
        //     {
        //       planning_result.combined_maneuver.setPoints.clear();
        //     }            
        //   }
        // }

        if(!valid_em_found)
        {
          planning_result.status_string += "emergency maneuver planning failed";
          return;
        }

        planning_result.combined_maneuver.cropAfterFirstStop(0.1);
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