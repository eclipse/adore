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

#include <adore/view/alane.h>
#include <adore/fun/afactory.h>
#include <adore/fun/tac/basiclanefollowingplanner.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/env/borderbased/lanefollowingview.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/traffic/trafficmap.h>

namespace adore
{
  namespace apps
  {
    /**
     * @brief test implementation of a lane following trajectory planner
     * 
     */
    class TestTrajectoryPlanner
    {
      private:
      typedef adore::fun::BasicLaneFollowingPlanner<20,5> TPlanner;
      adore::env::AFactory* envFactory_;
      adore::fun::AFactory* funFactory_;
      adore::params::AFactory* paramsFactory_;
      adore::params::APVehicle* pvehicle_;
      adore::params::APTacticalPlanner* pTacticalPlanner_;
      adore::env::BorderBased::LocalRoadMap roadmap_;
      adore::env::traffic::TrafficMap trafficMap_;
      adore::env::BorderBased::LaneFollowingView lfv_;
      adore::mad::AReader<adore::fun::VehicleMotionState9d>* xreader_;
      adore::mad::AWriter<adore::fun::SetPointRequest>* wwriter_;
      adore::fun::VehicleMotionState9d x_;
      TPlanner* planner_;
      public:
      TestTrajectoryPlanner(adore::env::AFactory* envFactory,
                            adore::fun::AFactory* funFactory,
                            adore::params::AFactory* PARAMS_Factory)
          :envFactory_(envFactory),funFactory_(funFactory),paramsFactory_(PARAMS_Factory),
           roadmap_(envFactory,paramsFactory_),trafficMap_(roadmap_.getBorderSet(),envFactory),
           lfv_(paramsFactory_,&roadmap_,&trafficMap_)
      {
        pvehicle_ = paramsFactory_->getVehicle();
        pTacticalPlanner_ = PARAMS_Factory->getTacticalPlanner();
        planner_ = new TPlanner(&lfv_,nullptr,nullptr,nullptr,
                                PARAMS_Factory->getLongitudinalPlanner(),
                                PARAMS_Factory->getLateralPlanner(),
                                PARAMS_Factory->getTacticalPlanner(),
                                pvehicle_,
                                PARAMS_Factory->getTrajectoryGeneration());
        xreader_ = funFactory->getVehicleMotionStateReader();
        wwriter_ = funFactory->getSetPointRequestWriter();
      }
      /**
       * @brief update function of the trajectory planner
       * 
       */
      void run()
      {
        roadmap_.update();
        trafficMap_.update();
        lfv_.update();
        xreader_->getData(x_);
        auto x_replan = x_;
        bool reset = true;
        if(planner_->hasValidPlan())
        {
          auto spr = planner_->getSetPointRequest();
          double t = x_.getTime();
          if( spr->isActive(t) )
          {
            auto x_ref = spr->interpolateReference(t,pvehicle_);
            double dx = x_.getX()-x_ref.getX();
            double dy = x_.getY()-x_ref.getY();
            if(dx*dx+dy*dy<pTacticalPlanner_->getResetRadius())
            {
              x_replan.setX(x_ref.getX());
              x_replan.setY(x_ref.getY());
              x_replan.setPSI(x_ref.getPSI());
              x_replan.setvx(x_ref.getvx());
              x_replan.setvy(x_ref.getvy());
              x_replan.setOmega(x_ref.getOmega());
              x_replan.setAx(x_ref.getAx());
              x_replan.setDelta(x_ref.getDelta());
              reset = false;
            }
          }
        }
        if(reset)std::cout<<"TestTrajectoryPlanner: Reset initial state.\n";
        planner_->compute(x_replan);
        if(planner_->hasValidPlan())wwriter_->write(*planner_->getSetPointRequest());
      }
      bool hasValidPlan()
      {
        return planner_->hasValidPlan();
      }
      adore::fun::DecoupledLFLCPlanner<20,5>::TProgressSolver& getProgressSolver()
      {
        return planner_->getProgressSolver();
      }
      adore::fun::DecoupledLFLCPlanner<20,5>::TOffsetSolver& getOffsetSolver()
      {
        return planner_->getOffsetSolver();
      }       
      const adore::fun::SetPointRequest* getSetPointRequest()
      {
        return planner_->getSetPointRequest();
      }
      adore::fun::RoadCoordinateConverter& getRoadCoordinateConverter()
      {
        return planner_->getRoadCoordinateConverter();
      }
    };
  }
}