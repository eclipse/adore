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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#include <adore/apps/objectdetectionmodel.h>
#include <adore_if_ros/baseapp.h>
namespace adore
{
  namespace if_ROS
  {  
    class ObjectDetectionModelNode : public Baseapp
    {
      public:
      adore::apps::ObjectDetectionModel* model_;
      ObjectDetectionModelNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();

        int simulationID = 0;
        getParam("simulationID",simulationID);
        model_ = new adore::apps::ObjectDetectionModel(getFactory<SIM_Factory>(),this->getParamsFactory(""),simulationID);

        // timer callbacks
        std::function<void()> fcn(std::bind(&adore::apps::ObjectDetectionModel::run,model_));
        Baseapp::addTimerCallback(fcn);
      }
    };
  }
}


int main(int argc,char **argv)
{
    adore::if_ROS::ObjectDetectionModelNode modeln;
    modeln.init(argc, argv, 100.0, "adore_objectdetectionmodel_node");
    modeln.run();
    return 0;
}


