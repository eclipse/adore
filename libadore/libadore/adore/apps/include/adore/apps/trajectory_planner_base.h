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
 *   Daniel Heß - base class for trajectory planners
 ********************************************************************************/


#pragma once

#include <adore/fun/afactory.h>
#include <adore/fun/tac/basicsetpointrequestevaluators.h>


namespace adore
{
  namespace apps
  {
    /**
     * @brief Base class for different trajectory planners: Handles communication w/ decision making module.
     * Reads PlanningRequest and writes PlanningResult. Actual trajectory planning has to occur by overriding virtual computeTrajectory method.
     */
    class TrajectoryPlannerBase
    {
      private:
      adore::mad::AWriter<adore::fun::PlanningResult>* result_writer_;
      adore::mad::AFeedWithCallback<adore::fun::PlanningRequest>* request_listener_;

      public:
      virtual ~TrajectoryPlannerBase()
      {
        delete result_writer_;
        delete request_listener_;
      }
      TrajectoryPlannerBase()
      {
        result_writer_ = adore::fun::FunFactoryInstance::get()->getPlanningResultWriter();
        request_listener_ =  adore::fun::FunFactoryInstance::get()->getPlanningRequestTrigger();
      }
      void prime()
      {
        std::function<void()> handler(std::bind(&adore::apps::TrajectoryPlannerBase::planning_request_handler,this));
        request_listener_->setCallback(handler);
      }
      void planning_request_handler()
      {
        adore::fun::PlanningRequest planning_request;
        adore::fun::PlanningResult planning_result;
        if(request_listener_->hasNext())
        {
            request_listener_->getLatest(planning_request);
            try{
              computeTrajectory(planning_request,planning_result);
              result_writer_->write(planning_result);
            }catch( const std::exception & ex ) {
              std::cerr << ex.what() << std::endl;
            }
        }
      }
      /**
       *@brief override computeTrajectory with actual planning method
       */
      virtual void computeTrajectory(const adore::fun::PlanningRequest& request, adore::fun::PlanningResult& result)=0;
    };
  }
}