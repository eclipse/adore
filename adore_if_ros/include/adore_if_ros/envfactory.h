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

#pragma once

#include <math.h>
#include <ros/ros.h>
# include <adore/env/afactory.h>
#include <adore_if_ros/ros_com_patterns.h>

#include <adore_if_ros_msg/Border.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "conversions/trafficparticipantconverter.h"
#include "conversions/borderconverter.h"
#include "conversions/envvehiclemotionstateconverter.h"
#include "conversions/navigationdataconverter.h"
#include "conversions/navigationgoalconverter.h"
#include "conversions/precedenceruleconverter.h"
#include "conversions/propositionconverter.h"
#include "conversions/precedenceruleconverter.h"


namespace adore
{
  namespace if_ROS
  {
    /**
     *  ROS implementation for data transmission of ENV data.
     */
    class ENV_Factory : public adore::env::AFactory
    {
      ros::NodeHandle* n_;
    public:
      ENV_Factory(ros::NodeHandle* n)
      {
        n_ = n;
      }
      // send updates for borders
      virtual adore::mad::AWriter<adore::env::BorderBased::Border>* getBorderWriter() override
      {
        return new Writer<adore::env::BorderBased::Border, // native type - msg type - converter
                          adore_if_ros_msg::Border,
                          BorderConverter>(n_,"ENV/Border",1000);
      }
      // read current vehicle state
      virtual adore::mad::AReader<adore::env::VehicleMotionState9d>* getVehicleMotionStateReader() override
      {
        return new Reader<adore::env::VehicleMotionState9d, // native type - msg type ConstPtr - converter
                          nav_msgs::OdometryConstPtr,
                          VehicleMotionStateConverter>(n_,"odom",1);
      }
      // read borders as they become visible
      virtual adore::mad::AFeed<adore::env::BorderBased::Border>* getBorderFeed() override
      {
        return new Feed<adore::env::BorderBased::Border,
                        adore_if_ros_msg::BorderConstPtr,
                        BorderConverter>(n_,"ENV/Border",1000);
      }
			// read a set of traffic participants - provides the current, complete set of tracked objects
			virtual adore::env::AFactory::TParticipantSetReader* getTrafficParticipantSetReader() override
      {
        return new Reader<adore::env::traffic::TParticipantSet, // native type - msg type ConstPtr - converter
                          adore_if_ros_msg::TrafficParticipantSetConstPtr,
                          TPSetConverter>(n_,"traffic",1);
      }
      // send navigation data
      virtual adore::mad::AWriter<std::pair<adore::env::BorderBased::BorderID,double>> * getNavigationDataWriter() override
      {
        return new Writer<std::pair<adore::env::BorderBased::BorderID,double>,
                          adore_if_ros_msg::NavigationData,
                          NavigationDataConverter>(n_,"ENV/NavigationData",1000);
      }
      // read navigation goal
      virtual adore::mad::AReader<adore::fun::NavigationGoal>* getNavigationGoalReader() override
      {
        return new Reader<adore::fun::NavigationGoal,
                        adore_if_ros_msg::NavigationGoalConstPtr,
                        NavigationGoalConverter>(n_,"ENV/NavigationGoal",1);
      }
      // read navigation data
			virtual adore::env::AFactory::TNavigationDataFeed* getNavigationDataFeed() override
      {
        return new Feed<std::pair<adore::env::BorderBased::BorderID,double>,
                          adore_if_ros_msg::NavigationDataConstPtr,
                          NavigationDataConverter>(n_,"ENV/NavigationData",1000);
      }
			// write precedence rules
			virtual TPrecedenceRuleWriter* getPrecedenceRuleWriter() override
      {
        return new Writer<adore::env::PrecedenceRule,
                          adore_if_ros_msg::Precedence,
                          PrecedenceRuleConverter>(n_,"ENV/Precedence",100);
      }

			// read precedence rules
			virtual TPrecedenceRuleFeed* getPrecedenceRuleFeed()override
      {
        return new Feed<adore::env::PrecedenceRule,
                        adore_if_ros_msg::PrecedenceConstPtr,
                        PrecedenceRuleConverter>(n_,"ENV/Precedence",100);
      }

			// write propositional logic information
			virtual adore::env::AFactory::TPropositionWriter* getPropositionWriter() override
      {
        return new Writer<adore::env::Proposition,
                          adore_if_ros_msg::Proposition,
                          PropositionConverter>(n_,"ENV/propositions",100);

      }

			// read propositional logic information
			virtual adore::env::AFactory::TPropositionFeed* getPropositionFeed() override
      {
        return new Feed<adore::env::Proposition,
                          adore_if_ros_msg::PropositionConstPtr,
                          PropositionConverter>(n_,"ENV/propositions",100);
     }

    };
  }
}