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
 *   Daniel Heß - initial implementation
 ********************************************************************************/


#pragma once

#include <adore/fun/afactory.h>
#include <adore/fun/tac/basiclanefollowingplanner.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/env/borderbased/lanefollowingview.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/traffic/trafficmap.h>
#include <adore/env/navigationgoalobserver.h>
#include <adore/env/tcd/connectionsonlane.h>
#include <adore/fun/tac/mrmplanner.h>
#include <adore/fun/safety/setpointrequestswath.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <adore/fun/tac/basicsetpointrequestevaluators.h>
#include <adore/env/traffic/decoupledtrafficpredictionview.h>
#include <adore/apps/trajectory_planner_lf.h>


namespace adore
{
  namespace apps
  {
    /**
     * @brief Decision making and maneuver planning, which realizes lane following only.
     * Basically a wrapper for trajectory_planner_lf, with initial state selection and direct SetPointRequest output to controller.
     * 
     */
    class LaneFollowingBehavior
    {
      private:
      adore::params::APVehicle* pvehicle_;
      adore::params::APTacticalPlanner* pTacticalPlanner_;
      adore::params::APTrajectoryGeneration* pTrajectoryGeneration_;
      adore::mad::AReader<adore::fun::VehicleMotionState9d>* xreader_;
      adore::mad::AReader<adore::fun::VehicleExtendedState>* xxreader_;
      adore::mad::AWriter<adore::fun::SetPointRequest>* sprwriter_;
      adore::mad::AWriter<adore::fun::SetPointRequest>* ntwriter_;
      adore::mad::AWriter<adore::fun::PlanningResult>* prwriter_;
      adore::fun::VehicleMotionState9d x_;
      adore::fun::VehicleExtendedState xx_;
      adore::fun::PlanningResult last_valid_planning_result_;

      TrajectoryPlannerLF trajectory_planner_;


      public:
      virtual ~LaneFollowingBehavior()
      {
        delete pvehicle_;
        delete pTacticalPlanner_;
        delete pTrajectoryGeneration_;
        delete xreader_;
        delete xxreader_;
        delete sprwriter_;
        delete prwriter_;
      }
      LaneFollowingBehavior()
      {
        pvehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
        pTacticalPlanner_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
        pTrajectoryGeneration_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
        xreader_ = adore::fun::FunFactoryInstance::get()->getVehicleMotionStateReader();
        xxreader_ = adore::fun::FunFactoryInstance::get()->getVehicleExtendedStateReader();
        sprwriter_ = adore::fun::FunFactoryInstance::get()->getSetPointRequestWriter();
        prwriter_ = adore::fun::FunFactoryInstance::get()->getPlanningResultWriter();
        ntwriter_ = adore::fun::FunFactoryInstance::get()->getNominalTrajectoryWriter();
      }
      /**
       * @brief select initial state and recompute maneuver
       * 
       */
      void run()
      {
        adore::fun::PlanningRequest request;
        adore::fun::PlanningResult result;
        xreader_->getData(x_);
        xxreader_->getData(xx_);
        request.iteration ++;
        request.initial_state = selectInitialState(x_,xx_,last_valid_planning_result_.combined_maneuver,
                                                        last_valid_planning_result_.combined_maneuver_valid);
        request.t_emergency_start = x_.getTime() + pTrajectoryGeneration_->getEmergencyManeuverDelay();

        trajectory_planner_.computeTrajectory(request,result);

        if(result.combined_maneuver_valid)
        {
          sprwriter_->write(result.combined_maneuver);
          ntwriter_->write(result.nominal_maneuver);
          last_valid_planning_result_ = result;
        }        

        prwriter_->write(result);
      }

      adore::fun::SetPoint selectInitialState(adore::fun::VehicleMotionState9d& x,
                                              adore::fun::VehicleExtendedState& xx,
                                              adore::fun::SetPointRequest& last_maneuver,
                                              bool last_maneuver_valid)
      {
        bool reset = true;
        adore::fun::SetPoint result;
        result.x0ref = adore::fun::PlanarVehicleState10d(x);
        result.tStart = x.getTime();
        result.tEnd = x.getTime();

        if( last_maneuver_valid
            && last_maneuver.setPoints.size()>0 //maneuver exists 
            && xx.getAutomaticControlOn() //< Freigabe im Fahrzeuginterface für Längs und Quer erhalten
            && xx.getAutomaticControlAccelerationActive())// Bestätigung der Freigabe durch Benutzer/Gaspedal erfolgt
        {
          double t = x.getTime();
          if( last_maneuver.isActive(t) )
          {
            auto x_ref = last_maneuver.interpolateReference(t,pvehicle_);
            double dx = x.getX()-x_ref.getX();
            double dy = x.getY()-x_ref.getY();
            double R = pTacticalPlanner_->getResetRadius();
            if(dx*dx+dy*dy<R*R)
            {
              result.x0ref = x_ref;
              reset = false;
            }
            else
            {
              std::cout <<"trajectory cannot be resumed: reset readius exceeded"<<std::endl;
            }
          }
          else
          {
            if(last_maneuver.setPoints.size()>0)
            {
              std::cout <<"trajectory cannot be resumed due to timeout. t= "<<t<<", spr: ["<<last_maneuver.setPoints.front().tStart<<";"<<last_maneuver.setPoints.back().tEnd<<"]"<<std::endl;
            }
          }
        }
        if(reset)std::cout<<"TestTrajectoryPlanner: Reset initial state.\n";
        return result;
      }
    };
  }
}