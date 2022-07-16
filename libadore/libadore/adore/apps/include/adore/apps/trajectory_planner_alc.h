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
 *   Daniel Heß - trajectory planner for advanced lane changing maneuvers
 ********************************************************************************/


#pragma once

#include <adore/fun/afactory.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/fun/tac/advancedlanechangeplanner.h>
#include <adore/fun/tac/cancellcmrmplanner.h>
#include <adore/fun/tac/continuelcmrmplanner.h>
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
     * Computes a lane change trajectory using advanced lane change planner
     */
    class TrajectoryPlannerALC:public TrajectoryPlannerBase
    {
      private:
      typedef adore::fun::AdvancedLaneChangePlanner<20,5> TNominalPlanner;
      typedef adore::fun::CancelLCMRMPlanner<20,5> TEmergencyPlanner1;
      typedef adore::fun::ContinueLCMRMPlanner<20,5> TEmergencyPlanner2;
      TNominalPlanner* nominal_planner_;
      TEmergencyPlanner1* emergency_planner1_;
      TEmergencyPlanner2* emergency_planner2_;

      adore::env::ThreeLaneViewDecoupled three_lanes_;/**<lane-based representation of environment*/
      adore::params::APVehicle* pvehicle_;
      adore::params::APTacticalPlanner* pTacticalPlanner_;
      adore::params::APTrajectoryGeneration* pTrajectoryGeneration_;
      adore::params::APEmergencyOperation* pEmergencyOperation_;
      adore::env::NavigationGoalObserver ngo_;
      adore::env::ControlledConnectionSet4Ego connectionSet_;/**< state of controlled connections in area*/
      adore::env::ControlledConnectionSet4Ego checkPointSet_;/**< state of checkPoints in area*/
      adore::env::ConnectionsOnLane* connectionsOnLaneSource_;/** map controlled connections to lane*/
      adore::env::ConnectionsOnLane* connectionsOnLaneTarget_;
      adore::env::ConnectionsOnLane* checkPointsOnTargetLane_;/** map controlled connections to lane*/
      adore::env::ConnectionsOnLane* checkPointsOnSourceLane_;/** map controlled connections to lane*/
      adore::env::DecoupledTrafficPredictionView prediction_;/**<collision detection based representation of traffic*/
      adore::env::EmptyGap gap_;/**<an empty gap for testing*/
      adore::fun::SPRTTCNominal ttcCost_;/**<collision detection based ttc computation*/
      adore::fun::SPRNonCoercive coercion_detection_;/**<collision detection vs expected behavior*/

      /**
       * combined maneuver post-processing constraints
       */
      adore::fun::SPRInvariantCollisionFreedom collision_detection_;/**<collision detection with traffic predictions*/

      int id_;/**<integral id to be written to PlanningResult*/
      std::string plannerName_;/**<human readable planner name written to PlanningResult*/
      bool directionLeft_;/**<true if lane change to the left*/
      bool gapTrackingActive_;/**<true if gap tracking was active in last iteration*/
      double lateral_i_grid_;/**<offset variation index*/
      double const_penalty_;/**penalty, which is always added to cost*/
      double gapX_;/**<last X position of gap*/
      double gapY_;/**<last Y position of gap*/
      double gapT_;/**<last time of iteration*/
      double gapv_;/**<last observed speed of gap*/
      double gaps_;/**<last s-progress associated with gapX_ and gapY_*/
      bool em_continue_active_;/**<de/activate emergency maneuver for continuation of lane change*/
      bool em_cancel_active_;/**de/cativate emergency maenuver for canceling of lane change*/
      public:
      virtual ~TrajectoryPlannerALC()
      {
        delete nominal_planner_;
        delete emergency_planner1_;
        delete emergency_planner2_;
      }
      void setEMContinueActive(bool value)
      {
        em_continue_active_ = value;
      }
      void setEMCancelActive(bool value)
      {
        em_cancel_active_ = value;
      }
      TrajectoryPlannerALC(bool directionLeft,std::string name,int id,double lateral_i_grid):
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
           gapTrackingActive_(false),
           lateral_i_grid_(lateral_i_grid),
           ttcCost_(&prediction_),
           em_continue_active_(true),
           em_cancel_active_(false),
           coercion_detection_(&prediction_)
      {
        id_ = id;
        plannerName_ = name;
        const_penalty_ = 0.0;
        pvehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
        pTacticalPlanner_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        pTrajectoryGeneration_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
        pEmergencyOperation_ = adore::params::ParamsFactoryInstance::get()->getEmergencyOperation();
        auto pLongitudinalPlanner = adore::params::ParamsFactoryInstance::get()->getLongitudinalPlanner();
        auto pLateralPlanner = adore::params::ParamsFactoryInstance::get()->getLateralPlanner();
        auto pTacticalPlanner = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        auto target_lc =  directionLeft
                        ?three_lanes_.getLeftLaneChange()
                        :three_lanes_.getRightLaneChange();
        auto target = target_lc->getTargetLane();
        auto source = target_lc->getSourceLane();
        connectionsOnLaneSource_ = new adore::env::ConnectionsOnLane(source,&connectionSet_);
        connectionsOnLaneTarget_ = new adore::env::ConnectionsOnLane(target,&connectionSet_);
        checkPointsOnTargetLane_ = new adore::env::ConnectionsOnLane(target,&checkPointSet_);
        checkPointsOnSourceLane_ = new adore::env::ConnectionsOnLane(source,&checkPointSet_);
        //create nominal planner and add additional constraints
        nominal_planner_ = new TNominalPlanner(
                                target_lc,
                                &ngo_,
                                connectionsOnLaneSource_,
                                connectionsOnLaneTarget_,
                                checkPointsOnSourceLane_,
                                checkPointsOnTargetLane_,
                                pLongitudinalPlanner,
                                pLateralPlanner,
                                pTacticalPlanner,
                                pvehicle_,
                                pTrajectoryGeneration_,
                                lateral_i_grid);
        nominal_planner_->setGap(&gap_);//assign constant empty gap for testing

        //create emergency planner 1
        emergency_planner1_ = new TEmergencyPlanner1(target_lc,
                          pLateralPlanner,
                          pTacticalPlanner,
                          pvehicle_,
                          pTrajectoryGeneration_);
        emergency_planner1_->setJMax(10.0);
        emergency_planner1_->setTStall(0.1);
        emergency_planner1_->setAStall(-2.0);
        emergency_planner1_->setAMin(-2.0);
        emergency_planner1_->setSecondAttempt(false);

        //create emergency planner 2
        emergency_planner2_ = new TEmergencyPlanner2(target_lc,
                          pLateralPlanner,
                          pTacticalPlanner,
                          pvehicle_,
                          pTrajectoryGeneration_,
                          lateral_i_grid);
        emergency_planner2_->setJMax(10.0);
        emergency_planner2_->setTStall(0.1);
        emergency_planner2_->setAStall(-2.0);
        emergency_planner2_->setAMin(-2.0);
        emergency_planner2_->setSecondAttempt(false);
        emergency_planner2_->setGap(&gap_);//assign constant empty gap for testing

      }
      void setSpeedScale(double value)
      {
        nominal_planner_->setSpeedScale(value);
      }      
      void setConstPenalty(double value)
      {
        const_penalty_ = value;
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
        planning_result.indicator_left = false;
        planning_result.indicator_right = false;

        three_lanes_.update();
        ngo_.update();
        connectionSet_.update();
        connectionsOnLaneSource_->update();
        connectionsOnLaneTarget_->update();
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
            // todo: investigate cost calculation as it seems implausible return;
        }


        prediction_.update();

        //delayed indicator activation
        double s,n;
        target->toRelativeCoordinates(planning_request.initial_state.x0ref.getX(),
                                      planning_request.initial_state.x0ref.getY(),
                                      s,n);
        double sg0 = target_lc->getProgressOfGateOpen();
        double indicator_lookahead = pTacticalPlanner_->getIndicatorLookahead();
        if(s+indicator_lookahead>sg0)
        {
          planning_result.indicator_left = directionLeft_;
          planning_result.indicator_right = !directionLeft_;
        }

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

        if(em_continue_active_)
        {
          emergency_planner2_->setJMax(-pEmergencyOperation_->getEmergencyManeuverJMin());
          emergency_planner2_->setTStall(pEmergencyOperation_->getEmergencyManeuverTStall());
          emergency_planner2_->setAStall(pEmergencyOperation_->getEmergencyManeuverAStall());
          emergency_planner2_->setAMin(pEmergencyOperation_->getEmergencyManeuverAMin());
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
            //planning_result.combined_maneuver.setPoints.clear();
            }
            
          }
        }

        if(em_cancel_active_)
        {
          if(!valid_em_found)
          {
            emergency_planner1_->setJMax(-pEmergencyOperation_->getEmergencyManeuverJMin());
            emergency_planner1_->setTStall(pEmergencyOperation_->getEmergencyManeuverTStall());
            emergency_planner1_->setAStall(pEmergencyOperation_->getEmergencyManeuverAStall());
            emergency_planner1_->setAMin(pEmergencyOperation_->getEmergencyManeuverAMin());
            emergency_planner1_->compute(x0em.toMotionState());
            if(emergency_planner1_->hasValidPlan() )
            {
              nominal_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,0);
              planning_result.combined_maneuver.removeAfter(planning_request.t_emergency_start);
              planning_result.combined_maneuver.setPoints.back().tEnd = planning_request.t_emergency_start;
              emergency_planner1_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,1);
              if(collision_detection_.isValid(planning_result.combined_maneuver))
              {
                valid_em_found = true;
              }
              else
              {
                // planning_result.combined_maneuver.setPoints.clear();
              }            
            }
          }
        }
        
        if(!valid_em_found)
        {
          planning_result.status_string += "emergency maneuver planning failed";
          return;
        }

        double front_buffer_space = pTacticalPlanner_->getCollisionDetectionFrontBufferSpace();
        double lateral_precision = pTacticalPlanner_->getCollisionDetectionLateralPrecision();
        adore::fun::SetPointRequestSwath spr_swath(
            pvehicle_->get_a()+pvehicle_->get_b()+pvehicle_->get_c()+pvehicle_->get_d()+front_buffer_space,
            pvehicle_->get_bodyWidth(),
            pvehicle_->get_d(),//spr reference point at rear axle
            lateral_precision);
        spr_swath.setLonError(pTacticalPlanner_->getCollisionDetectionLongitudinalError());
        spr_swath.setLatError(pTacticalPlanner_->getCollisionDetectionLateralError());
        spr_swath.append_cylinder_swath_linear(planning_result.combined_maneuver,planning_result.combined_maneuver_swath);
        spr_swath.append_cylinder_swath_linear(planning_result.nominal_maneuver,planning_result.nominal_maneuver_swath);

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

        planning_result.objective_values.insert({"const_penalty",const_penalty_});

        //ttc cost
        planning_result.objective_values.insert({ttcCost_.getName(),
                                                 ttcCost_.getCost(planning_result.nominal_maneuver)});


        switch(pTacticalPlanner_->getCoercionPreventionStrategy())
        {
          case 0://turned off
            {
              planning_result.objective_values.insert({coercion_detection_.getName(),0.0});
            }
            break;
          case 1://encode as objective value
            {
              planning_result.objective_values.insert({coercion_detection_.getName(),
                                                        coercion_detection_.isValid(planning_result.nominal_maneuver)
                                                          ? 0.0 : 1.0});
            }
            break;
          case 2://encode as constraint
            {
              bool coercion_detection_passed = coercion_detection_.isValid(planning_result.nominal_maneuver);
              planning_result.combined_maneuver_valid = coercion_detection_passed && planning_result.combined_maneuver_valid;
              if(!coercion_detection_passed)
              {
                planning_result.status_string = "coercion detected for nominal maneuver";    
              }
            }
            break;
        }


        int number_of_time_steps = nominal_planner_->getProgressSolver().lbx().nc();
        auto time_steps = adore::mad::linspace(nominal_planner_->getPlanningHorizon()/number_of_time_steps,nominal_planner_->getPlanningHorizon(),number_of_time_steps);
        planning_result.nominal_maneuver_longitudinal_plan = nominal_planner_->getProgressSolver().result_fun();
        planning_result.nominal_maneuver_longitudinal_lbx.getData() = dlib::join_cols(time_steps,nominal_planner_->getProgressSolver().lbx());
        planning_result.nominal_maneuver_longitudinal_ubx.getData() = dlib::join_cols(time_steps,nominal_planner_->getProgressSolver().ubx());
        planning_result.nominal_maneuver_lateral_plan = nominal_planner_->getOffsetSolver().result_fun();
        planning_result.nominal_maneuver_lateral_lbx.getData() = dlib::join_cols(time_steps,nominal_planner_->getOffsetSolver().lbx());
        planning_result.nominal_maneuver_lateral_ubx.getData() = dlib::join_cols(time_steps,nominal_planner_->getOffsetSolver().ubx());
      }
    };
  }
}