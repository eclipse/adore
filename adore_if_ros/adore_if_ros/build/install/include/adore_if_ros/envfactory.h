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
#include <adore/env/afactory.h>
#include <adore_if_ros/ros_com_patterns.h>

#include <adore_if_ros_msg/Border.h>
#include <adore_if_ros_msg/LaneGeometry.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "conversions/trafficparticipantconverter.h"
#include "conversions/borderconverter.h"
#include "conversions/bordertypechangeprofileconverter.h"
#include "conversions/envvehiclemotionstateconverter.h"
#include "conversions/navigationdataconverter.h"
#include "conversions/navigationgoalconverter.h"
#include "conversions/precedenceruleconverter.h"
#include "conversions/propositionconverter.h"
#include "conversions/precedenceruleconverter.h"
#include "conversions/tcdconnectionconverter.h"
#include "conversions/occupancyconverter.h"
#include "conversions/lanegeometryconverter.h"
#include "conversions/speedlimitconverter.h"
#include "conversions/areaofeffectconverter.h"
#include "conversions/cooperativeuserspredictionconverter.h"
#include "conversions/gapconverter.h"
#include "conversions/stdconverter.h"
#include "conversions/indicatorhintconverter.h"


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
      adore::mad::AWriter<adore::env::BorderBased::Border>* getBorderWriter() override
      {
        return new Writer<adore::env::BorderBased::Border, // native type - msg type - converter
                          adore_if_ros_msg::Border,
                          BorderConverter>(n_,"ENV/Border",1000);
      }
      // read current vehicle state
      adore::mad::AReader<adore::env::VehicleMotionState9d>* getVehicleMotionStateReader() override
      {
        return new Reader<adore::env::VehicleMotionState9d, // native type - msg type ConstPtr - converter
                          nav_msgs::OdometryConstPtr,
                          VehicleMotionStateConverter>(n_,"localization",1);
      }
      // read borders as they become visible
      adore::mad::AFeed<adore::env::BorderBased::Border>* getBorderFeed() override
      {
        return new Feed<adore::env::BorderBased::Border,
                        adore_if_ros_msg::BorderConstPtr,
                        BorderConverter>(n_,"ENV/Border",1000);
      }
      // write border type change tasks
      virtual adore::mad::AWriter<adore::env::BorderTypeChangeProfile>* getBorderTypeChangeProfileWriter() override
      {
        return new Writer<adore::env::BorderTypeChangeProfile,
                          adore_if_ros_msg::BorderTypeChangeProfile,
                          BorderTypeChangeProfileConverter>(n_,"ENV/BorderTypeChangeProfile",100);
      }

      // read border type change tasks
      virtual adore::mad::AFeed<adore::env::BorderTypeChangeProfile>* getBorderTypeChangeProfileFeed() override
      {
        return new Feed<adore::env::BorderTypeChangeProfile,
                        adore_if_ros_msg::BorderTypeChangeProfileConstPtr,
                        BorderTypeChangeProfileConverter>(n_,"ENV/BorderTypeChangeProfile",100);
      }
			// read a set of traffic participants - provides the current, complete set of tracked objects
      //extension 20220524: read and combine real and virtual traffic objects
			adore::env::AFactory::TParticipantSetReader* getTrafficParticipantSetReader() override
      {
        return new TPSetMultiReader(n_,"traffic","virtual_traffic",1);
      }
      
      // send navigation data
      adore::mad::AWriter<std::pair<adore::env::BorderBased::BorderID,double>> * getNavigationDataWriter() override
      {
        return new Writer<std::pair<adore::env::BorderBased::BorderID,double>,
                          adore_if_ros_msg::NavigationData,
                          NavigationDataConverter>(n_,"ENV/NavigationData",1000);
      }
      // read navigation goal
      adore::mad::AReader<adore::fun::NavigationGoal>* getNavigationGoalReader() override
      {
        return new Reader<adore::fun::NavigationGoal,
                        adore_if_ros_msg::NavigationGoalConstPtr,
                        NavigationGoalConverter>(n_,"ENV/NavigationGoal",1);
      }
      // read navigation data
			adore::env::AFactory::TNavigationDataFeed* getNavigationDataFeed() override
      {
        return new Feed<std::pair<adore::env::BorderBased::BorderID,double>,
                          adore_if_ros_msg::NavigationDataConstPtr,
                          NavigationDataConverter>(n_,"ENV/NavigationData",1000);
      }
			// write precedence rules
			TPrecedenceRuleWriter* getPrecedenceRuleWriter() override
      {
        return new Writer<adore::env::PrecedenceRule,
                          adore_if_ros_msg::Precedence,
                          PrecedenceRuleConverter>(n_,"ENV/Precedence",100);
      }

			// read precedence rules
			TPrecedenceRuleFeed* getPrecedenceRuleFeed()override
      {
        return new Feed<adore::env::PrecedenceRule,
                        adore_if_ros_msg::PrecedenceConstPtr,
                        PrecedenceRuleConverter>(n_,"ENV/Precedence",100);
      }

			// write propositional logic information
			adore::env::AFactory::TPropositionWriter* getPropositionWriter() override
      {
        return new Writer<adore::env::Proposition,
                          adore_if_ros_msg::Proposition,
                          PropositionConverter>(n_,"ENV/propositions",100);
      }

			// read propositional logic information
			adore::env::AFactory::TPropositionFeed* getPropositionFeed() override
      {
        return new Feed<adore::env::Proposition,
                          adore_if_ros_msg::PropositionConstPtr,
                          PropositionConverter>(n_,"ENV/propositions",100);
      }

			// read traffic light information: controlled connection specifies state of a connection, which is controlled by for example a traffic light
			adore::env::AFactory::TControlledConnectionFeed* getControlledConnectionFeed() override
      {
        return new Feed<adore::env::ControlledConnection,
                        adore_if_ros_msg::TCDConnectionStateTraceConstPtr,
                        TCDConnectionConverter>(n_,"ENV/tcd",500);
      }

			// read checkpoint information represented as controlled connections
			adore::env::AFactory::TControlledConnectionFeed* getCheckPointFeed() override
      {
        return new Feed<adore::env::ControlledConnection,
                        adore_if_ros_msg::TCDConnectionStateTraceConstPtr,
                        TCDConnectionConverter>(n_,"ENV/checkpoints",100);
      }
      
			//write checkpoint information represented as controlled connections
			adore::env::AFactory::TControlledConnectionWriter* getCheckPointWriter() override
      {
        return new Writer<adore::env::ControlledConnection,
                          adore_if_ros_msg::TCDConnectionStateTrace,
                          TCDConnectionConverter>(n_,"ENV/checkpoints",100);
      }

			// read the latest prediction set for expected behavior
			adore::env::AFactory::TOCPredictionSetReader* getExpectedPredictionSetReader() override
      {
        return new Reader<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSetConstPtr,
                        OccupancyConverter>(n_,"ENV/Prediction/expected",1);
      }

			// read the latest prediction set for worst-case behavior
			adore::env::AFactory::TOCPredictionSetReader* getWorstCasePredictionSetReader() override
      {
        return new Reader<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSetConstPtr,
                        OccupancyConverter>(n_,"ENV/Prediction/worstcase",1);
      }

			// read the latest prediction set for expected behavior, unfiltered
			adore::env::AFactory::TOCPredictionSetReader* getExpectedRawPredictionSetReader() override
      {
        return new Reader<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSetConstPtr,
                        OccupancyConverter>(n_,"ENV/Prediction/expected_raw",1);
      }

			// read the latest prediction set for worst-case behavior, unfiltered
			adore::env::AFactory::TOCPredictionSetReader* getWorstCaseRawPredictionSetReader() override
      {
        return new Reader<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSetConstPtr,
                        OccupancyConverter>(n_,"ENV/Prediction/worstcase_raw",1);
      }

			// read the latest prediction set for desired behavior
			adore::env::AFactory::TOCPredictionSetReader* getDesiredPredictionSetReader() override
      {
        return new Reader<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSetConstPtr,
                        OccupancyConverter>(n_,"ENV/Prediction/desired",1);
      }

			// read the latest prediction set for static obstacles
			adore::env::AFactory::TOCPredictionSetReader* getStaticObstaclesPredictionSetReader() override
      {
         return new Reader<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSetConstPtr,
                        OccupancyConverter>(n_,"ENV/Prediction/static",1);
      }
      
     	// write the latest prediction set for expected behavior
			adore::env::AFactory::TOCPredictionSetWriter* getExpectedPredictionSetWriter() override
      {
         return new Writer<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSet,
                        OccupancyConverter>(n_,"ENV/Prediction/expected",1);
      }

			// write the latest prediction set for worst-case behavior
			adore::env::AFactory::TOCPredictionSetWriter* getWorstCasePredictionSetWriter() override
      {
        return new Writer<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSet,
                        OccupancyConverter>(n_,"ENV/Prediction/worstcase",1);
      }

			// write the latest prediction set for expected behavior, unfiltered
			adore::env::AFactory::TOCPredictionSetWriter* getExpectedRawPredictionSetWriter() override
      {
         return new Writer<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSet,
                        OccupancyConverter>(n_,"ENV/Prediction/expected_raw",1);
     }

			// write the latest prediction set for worst-case behavior, unfiltered
			adore::env::AFactory::TOCPredictionSetWriter* getWorstCaseRawPredictionSetWriter() override
      {
        return new Writer<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSet,
                        OccupancyConverter>(n_,"ENV/Prediction/worstcase_raw",1);
      }

			// write the latest prediction set for desired behavior
			adore::env::AFactory::TOCPredictionSetWriter* getDesiredPredictionSetWriter() override
      {
        return new Writer<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSet,
                        OccupancyConverter>(n_,"ENV/Prediction/desired",1);
      }
			
      // read the latest conflict set 
			adore::env::AFactory::TOCPredictionSetReader* getConflictSetReader()override
      {
         return new Reader<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSetConstPtr,
                        OccupancyConverter>(n_,"ENV/ConflictSet",1);
      }

			// read the latest conflict set 
			adore::env::AFactory::TOCPredictionSetWriter* getConflictSetWriter()override
      {
        return new Writer<adore::env::OccupancyCylinderPredictionSet,
                        adore_if_ros_msg::OccupancyCylinderPredictionSet,
                        OccupancyConverter>(n_,"ENV/ConflictSet",1);
      }

      // get combined lane geometry feed
			adore::env::AFactory::TLaneGeometryFeed* getLaneGeometryFeed() override
      {
        return new Feed<adore::env::BorderBased::CombinedLaneGeometry,
                          adore_if_ros_msg::LaneGeometryConstPtr,
                          LaneGeometryConverter>(n_,"ENV/lanegeometry",100);
      }

      // write the combined lane geometry
			adore::env::AFactory::TLaneGeometryWriter* getLaneGeometryWriter() override
      {
        return new Writer<adore::env::BorderBased::CombinedLaneGeometry,
                        adore_if_ros_msg::LaneGeometry,
                        LaneGeometryConverter>(n_,"ENV/lanegeometry",100);
      }

      // reads reset signal for lane matching
			adore::env::AFactory::TResetLaneMatchingReader* getResetLaneMatchingReader() override
      {
        return new Reader<bool,std_msgs::BoolPtr,StdConverter>(n_,"ENV/ResetLaneMatching",1);
      }

      // writes reset signal for lane matching
			adore::env::AFactory::TResetLaneMatchingWriter* getResetLaneMatchingWriter() override
      {
        return new Writer<bool,std_msgs::Bool,StdConverter>(n_,"ENV/ResetLaneMatching",1);
      }

      TSpeedLimitWriter* getSpeedLimitWriter() override
      {
        return new Writer<adore::env::SpeedLimit,
                          adore_if_ros_msg::SpeedLimit,
                          SpeedLimitConverter>(n_,"ENV/speedlimits",100);
      }

      // feed for speed limit info
			TSpeedLimitFeed* getSpeedLimitFeed() override
      {
        return new Feed<adore::env::SpeedLimit,
                          adore_if_ros_msg::SpeedLimitConstPtr,
                          SpeedLimitConverter>(n_,"ENV/speedlimits",100);
      }
 			adore::env::AFactory::TAreaOfEffectWriter* getAreaOfEffectWriter() override
      {
        return new Writer<adore::env::AreaOfEffect,
                        adore_if_ros_msg::AreaOfEffect,
                        AreaOfEffectConverter>(n_,"ENV/areaofeffect",1);
      }
 			adore::env::AFactory::TAreaOfEffectWriter* getAreaOfInterestWriter() override
      {
        return new Writer<adore::env::AreaOfEffect,
                        adore_if_ros_msg::AreaOfEffect,
                        AreaOfEffectConverter>(n_,"ENV/areaofinterest",1);
      }
      virtual adore::mad::AReader<adore::env::CooperativeUserPrediction>* getCooperativeUserReader() override
      {
        return new Reader<adore::env::CooperativeUserPrediction, // native type - msg type ConstPtr - converter
                          adore_if_ros_msg::CooperativePlanningConstPtr ,
                          CooperativeUsersPredictionConverter>(n_,"ENV/Cooperation/cooperativeuser",1);
      }   

      virtual adore::mad::AWriter<adore::env::CooperativeUserPrediction>* getCooperativeUserWriter() override
      {
        return new Writer<adore::env::CooperativeUserPrediction, // native type - msg type ConstPtr - converter
                          adore_if_ros_msg::CooperativePlanning ,
                          CooperativeUsersPredictionConverter>(n_,"ENV/Cooperation/cooperativeuser",1);
      }  

      virtual adore::mad::AReader<adore::env::CooperativeUsersList>* getCooperativeUsersListReader() override
      {
        return new Reader<adore::env::CooperativeUsersList, // native type - msg type ConstPtr - converter
                          adore_if_ros_msg::CooperativePlanningSetConstPtr ,
                          CooperativeUsersPredictionConverter>(n_,"ENV/Cooperation/cooperativeuserslist",1);
      }   

      virtual adore::mad::AWriter<adore::env::CooperativeUsersList>* getCooperativeUsersListWriter() override
      {
        return new Writer<adore::env::CooperativeUsersList, // native type - msg type ConstPtr - converter
                          adore_if_ros_msg::CooperativePlanningSet ,
                          CooperativeUsersPredictionConverter>(n_,"ENV/Cooperation/cooperativeuserslist",1);
      }             
     
			// write the gap queue on the left lane
			adore::env::AFactory::TGapQueueWriter* getGapQueueWriterLeftLane()override
      {
        return new Writer<adore::env::GapQueue,
                        adore_if_ros_msg::GapQueue,
                        GapConverter>(n_,"ENV/gaps/leftlane",1);
      }

			// write the gap queue on the right lane
			adore::env::AFactory::TGapQueueWriter* getGapQueueWriterRightLane()override
      {
        return new Writer<adore::env::GapQueue,
                        adore_if_ros_msg::GapQueue,
                        GapConverter>(n_,"ENV/gaps/rightlane",1);
      }

			// read the gap queue on the left lane
			adore::env::AFactory::TGapQueueReader* getGapQueueReaderLeftLane()override
      {
        return new Reader<adore::env::GapQueue,
                        adore_if_ros_msg::GapQueueConstPtr,
                        GapConverter>(n_,"ENV/gaps/leftlane",1);
      }

			// read the gap queue on the right lane
			adore::env::AFactory::TGapQueueReader* getGapQueueReaderRightLane()override
      {
        return new Reader<adore::env::GapQueue,
                        adore_if_ros_msg::GapQueueConstPtr,
                        GapConverter>(n_,"ENV/gaps/rightlane",1);
      }

      // writer for indicator hints
      TIndicatorHintWriter* getIndicatorHintWriter() override 
      {
        return new Writer<adore::env::IndicatorHint,
                          adore_if_ros_msg::IndicatorHint,
                          IndicatorHintConverter>(n_,"ENV/indicatorhints",100);
      }

      // feed for indicator hints
			TIndicatorHintFeed* getIndicatorHintFeed() override
      {
        return new Feed<adore::env::IndicatorHint,
                          adore_if_ros_msg::IndicatorHintConstPtr,
                          IndicatorHintConverter>(n_,"ENV/indicatorhints",100);
      }


      
    };
  }
}