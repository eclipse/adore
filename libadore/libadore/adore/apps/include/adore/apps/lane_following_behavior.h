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
#include <adore/params/afactory.h>
#include <adore/apps/trajectory_planner_lf.h>
#include <adore/fun/setpointrequest_dispatcher.h>

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
      adore::params::APTrajectoryGeneration* pTrajectoryGeneration_;/**<trajectory generation parameters*/
      adore::fun::PlanningRequest request_;                         /**<the latests request*/
      adore::mad::AWriter<adore::fun::PlanningResult>* prwriter_;   /**<sends planning results*/
      adore::mad::AWriter<adore::fun::PlanningRequest>* reqwriter_; /**<sends planning requests*/
      adore::fun::SetPointRequestDispatcher dispatcher_;            /**<dispatcher_ handles initial state selection and dispatching of setpointrequest*/
      TrajectoryPlannerLF trajectory_planner_;                      /**<trajectory_planner_ plans lane following maneuvers*/


      public:
      virtual ~LaneFollowingBehavior()
      {
        delete pTrajectoryGeneration_;
        delete prwriter_;
        delete reqwriter_;
      }
      LaneFollowingBehavior()
      {
        pTrajectoryGeneration_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
        prwriter_ = adore::fun::FunFactoryInstance::get()->getPlanningResultWriter();
        reqwriter_ = adore::fun::FunFactoryInstance::get()->getPlanningRequestWriter();
      }
      /**
       * @brief select initial state and recompute maneuver
       * 
       */
      void run()
      {
        request_.iteration ++;

        adore::fun::VehicleMotionState9d x0;
        bool x0_available = dispatcher_.getInitialState(x0);
        if(dispatcher_.getStatus()!="")std::cout<<dispatcher_.getStatus()<<std::endl;
        if(!x0_available)return;
        request_.initial_state.x0ref = adore::fun::PlanarVehicleState10d( x0 );
        request_.initial_state.tStart = x0.getTime();
        request_.initial_state.tEnd = x0.getTime();
        request_.t_emergency_start = x0.getTime() + pTrajectoryGeneration_->getEmergencyManeuverDelay();

        reqwriter_->write(request_);//request is published for logging only

        adore::fun::PlanningResult result;
        trajectory_planner_.computeTrajectory(request_,result);

        if(result.combined_maneuver_valid)
        {
            dispatcher_.dispatch(result.combined_maneuver,result.nominal_maneuver);
        }        
        prwriter_->write(result);//result is published for logging and visualization
      }
    };
  }
}