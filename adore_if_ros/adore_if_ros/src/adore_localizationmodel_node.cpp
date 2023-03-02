/********************************************************************************
 * Copyright (C) 2017-2021 German Aerospace Center (DLR). 
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


#include <adore/apps/localizationmodel.h>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>


namespace adore
{
  namespace if_ROS
  {  
    class LocalizationModelNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      adore::apps::LocalizationModel* lm_;
      LocalizationModelNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        lm_ = new adore::apps::LocalizationModel();
        // timer callbacks
        std::function<void()> callback(std::bind(&adore::apps::LocalizationModel::update,lm_));
        Baseapp::addTimerCallback(callback);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::LocalizationModelNode lmn;
    lmn.init(argc, argv, 50.0, "adore_localizationmodel_node");
    lmn.run();
    return 0;
}
