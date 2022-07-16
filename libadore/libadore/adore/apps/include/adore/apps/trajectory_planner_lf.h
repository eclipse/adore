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
#include <adore/env/navigationgoalobserver.h>
#include <adore/env/tcd/connectionsonlane.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <adore/fun/tac/basicsetpointrequestevaluators.h>
#include <adore/env/traffic/decoupledtrafficpredictionview.h>
#include <adore/apps/trajectory_planner_base.h>
#include <adore/fun/safety/setpointrequestswath.h>
#include <adore/env/traffic/decoupledconflictpointview.h>


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
      adore::params::APEmergencyOperation* pEmergencyOperation_;
      adore::params::APPrediction* ppred_;
      adore::env::NavigationGoalObserver ngo_;
      adore::env::ControlledConnectionSet4Ego connectionSet_;/**< state of controlled connections in area*/
      adore::env::ControlledConnectionSet4Ego checkPointSet_;/**< state of checkPoints in area*/
      adore::env::ConnectionsOnLane* connectionsOnLane_;/** map controlled connections to lane*/
      adore::env::ConnectionsOnLane* checkPointsOnLane_;/** map controlled connections to lane*/
      adore::env::ThreeLaneViewDecoupled three_lanes_;/**<lane-based representation of environment*/
      adore::env::DecoupledTrafficPredictionView prediction_;/**<collision detection based representation of traffic*/
      adore::fun::SPRTTCNominal ttcCost_;/**<collision detection based ttc computation*/
      adore::fun::SPRNonCoercive coercion_detection_;/**<collision detection vs expected behavior*/
      adore::env::DecoupledConflictPointView conflicts_;/**cross traffic conflicts*/

      /**
       * combined maneuver post-processing constraints
       */
      adore::fun::SPRInvariantCollisionFreedom collision_detection_;/**<collision detection with traffic predictions*/


      int id_;/**<integral id to be written to PlanningResult*/
      std::string plannerName_;/**human readable planner name written to PlanningResult*/
      double lateral_i_grid_;/**grid index*/
      double const_penalty_;/**penalty, which is always added to cost*/
      public:
      virtual ~TrajectoryPlannerLF()
      {
        delete nominal_planner_;
        delete emergency_planner_;
      }
      TrajectoryPlannerLF(int id=0,std::string plannerName = "lane-following",double lateral_i_grid = 0.0):
           connectionSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getControlledConnectionFeed()),
           checkPointSet_(adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader(),
                          adore::env::EnvFactoryInstance::get()->getCheckPointFeed()),
           ngo_(adore::env::EnvFactoryInstance::get(),three_lanes_.getCurrentLane(),0,0),
           prediction_(),
           coercion_detection_(&prediction_),
           conflicts_(three_lanes_.getCurrentLane()),
           collision_detection_(&prediction_),
           ttcCost_(&prediction_)
      {
        id_ = id;
        lateral_i_grid_ = lateral_i_grid;
        const_penalty_ = 0.0;
        plannerName_ = plannerName;
        pvehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
        pTacticalPlanner_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        pTrajectoryGeneration_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
        pEmergencyOperation_ = adore::params::ParamsFactoryInstance::get()->getEmergencyOperation();
        ppred_ = adore::params::ParamsFactoryInstance::get()->getPrediction();
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
                                pTrajectoryGeneration_,
                                lateral_i_grid);

        //create emergency planner 
        emergency_planner_ = new TEmergencyPlanner(three_lanes_.getCurrentLane(),
                          pLateralPlanner,
                          pTacticalPlanner,
                          pvehicle_,
                          pTrajectoryGeneration_,
                          lateral_i_grid);
        emergency_planner_->setJMax(10.0);
        emergency_planner_->setTStall(0.1);
        emergency_planner_->setAStall(-2.0);
        emergency_planner_->setAMin(-2.0);
        emergency_planner_->setSecondAttempt(false);

      }
      void setConstPenalty(double value)
      {
        const_penalty_ = value;
      }
      void setSpeedScale(double value)
      {
        nominal_planner_->setSpeedScale(value);
      }

      void setStopPoint(int value)
      {
        if(value<0)
        {
          nominal_planner_->setConflictSet(nullptr);
        }
        else
        {
          nominal_planner_->setConflictSet(&conflicts_);
        }

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
        conflicts_.update();


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
        planning_result.nominal_maneuver_valid = true;
        auto x0em = nominal_planner_->getSetPointRequest()->interpolateSetPoint(planning_request.t_emergency_start,pvehicle_);
        emergency_planner_->setJMax(-pEmergencyOperation_->getEmergencyManeuverJMin());
        emergency_planner_->setTStall(pEmergencyOperation_->getEmergencyManeuverTStall());

        // adore::fun::RestartEffort re;  //test whether trajectory is long enough to justify restarting from ~0
        // if(!re.isValid(planning_result.nominal_maneuver))
        // {
        //   planning_result.status_string = "not restarting, maneuver too short";
        //   return;
        // }

        //if emergencyManeuverAMax and AMin are different, compute three maneuvers for {amin, (amin+amax)/2, amax}
        //otherwise compute just one maneuver for amin
        std::vector<double> emergency_acceleration;
        emergency_acceleration.push_back(pEmergencyOperation_->getEmergencyManeuverAMin());
        if(  pEmergencyOperation_->getEmergencyManeuverAMax()
           - pEmergencyOperation_->getEmergencyManeuverAMin() > 0.01 )
        {
          emergency_acceleration.push_back((pEmergencyOperation_->getEmergencyManeuverAMin()
                                          + pEmergencyOperation_->getEmergencyManeuverAMax())*0.5);
          emergency_acceleration.push_back(pEmergencyOperation_->getEmergencyManeuverAMax());
        }

        bool collision_detection_passed = false;

        for(double a_em:emergency_acceleration)
        {
          emergency_planner_->setAStall(a_em);
          emergency_planner_->setAMin(a_em);
          emergency_planner_->compute(x0em.toMotionState());

          if(!emergency_planner_->hasValidPlan() ) 
          {
            continue;
          }

          planning_result.combined_maneuver.setPoints.clear();
          nominal_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,0);
          planning_result.combined_maneuver.removeAfter(planning_request.t_emergency_start);
          planning_result.combined_maneuver.setPoints.back().tEnd = planning_request.t_emergency_start;
          emergency_planner_->getSetPointRequest()->copyTo(planning_result.combined_maneuver,1);


          if(collision_detection_.isValid(planning_result.combined_maneuver))
          {
            collision_detection_passed = true;
            break;
          }
        }


        if(!emergency_planner_->hasValidPlan() ) 
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
        spr_swath.setDuration(ppred_->get_roadbased_prediction_duration());
        spr_swath.append_cylinder_swath_linear(planning_result.combined_maneuver,planning_result.combined_maneuver_swath,true);
        spr_swath.setAccelerationErrorSlow(pTacticalPlanner_->getNominalSwathAccelerationError());
        spr_swath.append_cylinder_swath_linear(planning_result.nominal_maneuver,planning_result.nominal_maneuver_swath,true);

        if(!collision_detection_passed)
        {
          planning_result.status_string = "collision detected for combined maneuver";    
          return;          
        }

        planning_result.combined_maneuver.cropAfterFirstStop(pTacticalPlanner_->getTerminateAfterFirstStopThresholdSpeed());
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

         //indicator activation if hinted at
        double s,n;
        current->toRelativeCoordinates(planning_request.initial_state.x0ref.getX(),
                                      planning_request.initial_state.x0ref.getY(),
                                      s,n);
        // double sg0 = target_lc->getProgressOfGateOpen();
        // double indicator_lookahead = pTacticalPlanner_->getIndicatorLookahead();
        // if(s+indicator_lookahead>sg0)
        {
          planning_result.indicator_left = current->getLeftIndicatorHint(s);
          planning_result.indicator_right = current->getRightIndicatorHint(s);
        }
      
      }
    };
  }
}