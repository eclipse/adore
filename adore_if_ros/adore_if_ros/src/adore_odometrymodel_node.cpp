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

#include <adore/apps/odometrymodel.h>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>


namespace adore
{
  namespace if_ROS
  {  
    class OdometryModelNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      adore::apps::OdometryModel* om_;
      OdometryModelNode(){}
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        om_ = new adore::apps::OdometryModel();
        // timer callbacks
        std::function<void()> callback(std::bind(&adore::apps::OdometryModel::update,om_));
        Baseapp::addTimerCallback(callback);
      }
    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::OdometryModelNode omn;
    omn.init(argc, argv, 200.0, "adore_odometrymodel_node");
    omn.run();
    return 0;
}
