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
#include <adore/fun/tac/mrmplanner.h>
#include <adore/fun/tac/basicconstraintsandreferences.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/env/borderbased/lanefollowingview.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/traffic/trafficmap.h>
#include <plotlablib/figurestubfactory.h>

namespace adore
{
  namespace apps
  {
    /**
     * @brief test bench implementation for minimum risk maneuver planner
     * 
     */
    class TestMRMPlanner
    {
        private:
        typedef adore::fun::MRMPlanner<20,5> TPlanner;
        adore::env::AFactory* envFactory_;
        adore::fun::AFactory* funFactory_;
        adore::params::AFactory* paramsFactory_;
        adore::params::APVehicle* pvehicle_;
        adore::params::APTacticalPlanner* pTacticalPlanner_;
        adore::params::APLateralPlanner* plat_;
        adore::env::BorderBased::LocalRoadMap roadmap_;
        adore::env::traffic::TrafficMap trafficMap_;
        adore::env::BorderBased::LaneFollowingView lfv_;
        adore::mad::AReader<adore::fun::VehicleMotionState9d>* xreader_;
        adore::mad::AWriter<adore::fun::SetPointRequest>* wwriter_;
        adore::fun::VehicleMotionState9d x_;
        TPlanner* planner_;

        adore::fun::FollowCenterlineReference* followCenterlineReference_;/**< reference for lateral position: follow the middle of the lane*/
        adore::fun::LateralAccelerationReference* lateralAccelerationReference_;/**< curvature compensation*/
        adore::fun::LateralJerkReference* lateralJerkReference_;/**< lateral jerk compensation*/
        adore::fun::LateralOffsetConstraintLF* lateralOffsetConstraintLFUB_;/**< lateral position bounds governed by lane boundaries*/
        adore::fun::LateralOffsetConstraintLF* lateralOffsetConstraintLFLB_;/**< lateral position bounds governed by lane boundaries*/
        adore::fun::HeadingConstraint* headingConstraintUB_;/**< constraint for deviation from road direction */
        adore::fun::HeadingConstraint* headingConstraintLB_;/**< constraint for deviation from road direction */

        adore::fun::FollowPrecedingVehicle* followVehicleConstraint_;/**< post-processing constraint for longitudinal plan */

        DLR_TS::PlotLab::FigureStubFactory* fig_factory;
        DLR_TS::PlotLab::AFigureStub* figure_lon1;
        DLR_TS::PlotLab::AFigureStub* figure_lon2;
        DLR_TS::PlotLab::AFigureStub* figure_lat1;
        DLR_TS::PlotLab::AFigureStub* figure_lat2;
      public:
      TestMRMPlanner(adore::env::AFactory* envFactory,
                            adore::fun::AFactory* funFactory,
                            adore::params::AFactory* PARAMS_Factory)
          :envFactory_(envFactory),funFactory_(funFactory),paramsFactory_(PARAMS_Factory),
           roadmap_(envFactory,paramsFactory_),trafficMap_(roadmap_.getBorderSet(),envFactory),
           lfv_(paramsFactory_,&roadmap_,&trafficMap_)
      {
        pvehicle_ = paramsFactory_->getVehicle();
        pTacticalPlanner_ = PARAMS_Factory->getTacticalPlanner();
        plat_ = PARAMS_Factory->getLateralPlanner();
        planner_ = new TPlanner(&lfv_,
                                PARAMS_Factory->getLateralPlanner(),
                                pvehicle_,
                                PARAMS_Factory->getTrajectoryGeneration());
        followCenterlineReference_ = new adore::fun::FollowCenterlineReference(&lfv_);
        lateralAccelerationReference_ = new adore::fun::LateralAccelerationReference(&lfv_);
        lateralJerkReference_ = new adore::fun::LateralJerkReference(&lfv_);
        lateralOffsetConstraintLFUB_ = new adore::fun::LateralOffsetConstraintLF(&lfv_,pvehicle_,plat_,adore::fun::ANominalConstraint::UB);
        lateralOffsetConstraintLFLB_ = new adore::fun::LateralOffsetConstraintLF(&lfv_,pvehicle_,plat_,adore::fun::ANominalConstraint::LB);
        headingConstraintUB_ = new adore::fun::HeadingConstraint(plat_,adore::fun::ANominalConstraint::UB);
        headingConstraintLB_ = new adore::fun::HeadingConstraint(plat_,adore::fun::ANominalConstraint::LB);
        followVehicleConstraint_ = new adore::fun::FollowPrecedingVehicle(&lfv_,pvehicle_,pTacticalPlanner_,
                                 PARAMS_Factory->getTrajectoryGeneration());

        planner_->getOffsetSolver().getInformationSet().add(followCenterlineReference_);
        planner_->getOffsetSolver().getInformationSet().add(lateralAccelerationReference_);;
        planner_->getOffsetSolver().getInformationSet().add(lateralJerkReference_);
        planner_->getOffsetSolver().getInformationSet().add(lateralOffsetConstraintLFUB_);
        planner_->getOffsetSolver().getInformationSet().add(lateralOffsetConstraintLFLB_);
        planner_->getOffsetSolver().getInformationSet().add(headingConstraintUB_);
        planner_->getOffsetSolver().getInformationSet().add(headingConstraintLB_);

        planner_->getInformationSet().add(followVehicleConstraint_);
        xreader_ = funFactory->getVehicleMotionStateReader();
        wwriter_ = funFactory->getSetPointRequestWriter();
        init_plot();

      }
      ~TestMRMPlanner()
      {
        delete planner_;
        delete followCenterlineReference_;
        delete lateralAccelerationReference_;
        delete lateralJerkReference_;
        delete lateralOffsetConstraintLFUB_;
        delete lateralOffsetConstraintLFLB_;
        delete headingConstraintUB_;
        delete headingConstraintLB_;
        delete pvehicle_;
        delete pvehicle_;
        delete pTacticalPlanner_;
        delete plat_;
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
        if(planner_->hasValidPlan())
        {
            wwriter_->write(*planner_->getSetPointRequest());
            plot();
        }
      }
      void plot()
      {
        static const int N = 20*5+1;
        double XN[N];
        double YN[N];
        auto& longitudinal_plan = planner_->getLongitudinalPlan().getData();
        auto& lateral_plan = planner_->getLateralPlan().getData();
        adore::mad::copyRowToArray(longitudinal_plan,XN,0);//t
        adore::mad::copyRowToArray(longitudinal_plan,YN,1);//s
        figure_lon1->plot("s",XN,YN,N,"LineColor=0,0,1");
        adore::mad::copyRowToArray(longitudinal_plan,YN,2);//ds
        figure_lon2->plot("ds",XN,YN,N,"LineColor=0,0,1");
        adore::mad::copyRowToArray(longitudinal_plan,YN,3);//dds
        figure_lon2->plot("dds",XN,YN,N,"LineColor=1,0,0");
        adore::mad::copyRowToArray(longitudinal_plan,YN,4);//ddds
        figure_lon2->plot("ddds",XN,YN,N,"LineColor=0,0.5,0");
        //@todo: plot lateral result, if required
      }
      void init_plot()
      {
        fig_factory = new DLR_TS::PlotLab::FigureStubFactory();
        figure_lon1 = fig_factory->createFigureStub(3);
        figure_lon1->setTitle("Longitudinal Plan - s");
        figure_lon1->setXLabel("t (s)");
        figure_lon1->setYLabel("s (m)");
        figure_lon1->showAxis();
        figure_lon1->showGrid();
        figure_lon1->show();
        figure_lon2 = fig_factory->createFigureStub(4);
        figure_lon2->setTitle("Longitudinal Plan - ds(b),dds(g),ddds(r)");
        figure_lon2->setXLabel("t (s)");
        figure_lon2->setYLabel("ds (m/s)");
        figure_lon2->showAxis();
        figure_lon2->showGrid();
        figure_lon2->show();
        //@TODO: uncomment if plotting of lateral result is required
        // figure_lat1 = fig_factory->createFigureStub(5);
        // figure_lat1->setTitle("Lateral Plan - n");
        // figure_lat1->setXLabel("t (s)");
        // figure_lat1->setYLabel("n (m)");
        // figure_lat1->show();
        // figure_lat1->showAxis();
        // figure_lat1->showGrid();
        // figure_lat2 = fig_factory->createFigureStub(6);
        // figure_lat2->setTitle("Lateral Plan - dn(b),ddn(r),dddn(g)");
        // figure_lat2->setXLabel("t (s)");
        // figure_lat2->setYLabel("dn (m/s)");
        // figure_lat2->show();
        // figure_lat2->showAxis();
        // figure_lat2->showGrid();
      }
    };
  }
}