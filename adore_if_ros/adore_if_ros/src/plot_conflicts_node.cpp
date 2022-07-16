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
*   Daniel Heß
*   Thomas Lobig
********************************************************************************/

#include <adore_if_ros/envfactory.h>
#include <adore_if_ros/funfactory.h>
#include <adore/apps/plot_conflict_set.h>
#include <adore/apps/if_plotlab/prediction_config.h>
#include <plotlablib/figurestubfactory.h>
#include <adore_if_ros/baseapp.h>
#include <string>

namespace adore
{
  namespace if_ROS
  {  
    class PlotConflictsNode : public Baseapp
    {
      public:
      adore::apps::PlotConflicts* pb_;
      PlotConflictsNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        auto figure = fig_factory.createFigureStub(2);
        figure->show();
        int simulationID = 0;
 
        getParam("simulationID",simulationID);

        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        pb_ = new adore::apps::PlotConflicts(figure,
                                            ss.str());
        //@TODO activate plotting / management of borderset --> extra visualization module in params
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotConflicts::run,pb_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotConflictsNode node;    
    node.init(argc, argv, 10.0, "plot_conflicts_node");
    node.run();
    return 0;
}