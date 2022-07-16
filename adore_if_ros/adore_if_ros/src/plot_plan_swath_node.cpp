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

#include <adore_if_ros/funfactory.h>
#include <adore/apps/plot_plan_swath.h>
#include <plotlablib/figurestubfactory.h>
#include <adore_if_ros/baseapp.h>
#include <string>

namespace adore
{
  namespace if_ROS
  {  
    class PlotPlanSwathNode : public Baseapp
    {
      public:
      adore::apps::PlotPlanSwath* pb_;
      DLR_TS::PlotLab::AFigureStub* figure_;
      PlotPlanSwathNode()
      {
        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        figure_ = fig_factory.createFigureStub(2);
      }
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        int simulationID = 0;
        getParam("simulationID",simulationID);
        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        pb_ = new adore::apps::PlotPlanSwath(figure_,ss.str());
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotPlanSwath::run,pb_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotPlanSwathNode fbn;    
    fbn.init(argc, argv, 10.0, "plot_plan_swath_node");
    fbn.run();
    return 0;
}