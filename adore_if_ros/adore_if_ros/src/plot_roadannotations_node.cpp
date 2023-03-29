// SPDX-FileCopyrightText: 2019 German Aerospace Center (DLR)
//
// SPDX-License-Identifier: EPL-2.0

#include <adore/apps/plot_roadannotations.h>
#include <plotlablib/figurestubfactory.h>
#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>
// #include <string>

namespace adore
{
  namespace if_ROS
  {  
    class PlotRoadAnnotationsNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      adore::apps::PlotRoadAnnotations* app_;
      PlotRoadAnnotationsNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        auto figure = fig_factory.createFigureStub(2);
        figure->show();
        int simulationID = 0;
        getParam("simulationID",simulationID);
 
        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        app_ = new adore::apps::PlotRoadAnnotations(figure,
                                            ss.str()); //,
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotRoadAnnotations::run,app_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotRoadAnnotationsNode appnode;    
    appnode.init(argc, argv, 10.0, "plot_roadannotations_node");
    appnode.run();
    return 0;
}
