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

#include <adore/fun/afactory.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/NavigationGoal.h>
#include <adore_if_ros_msg/SetPointRequest.h>
#include <adore_if_ros_msg/TerminalRequest.h>
#include <adore_if_ros_msg/WheelSpeed.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include "conversions/navigationgoalconverter.h"
#include "conversions/setpointrequestconverter.h"
#include "conversions/terminalrequestconverter.h"
#include "conversions/gearselectioncommandconverter.h"
#include "conversions/motioncommandconverter.h"
#include "conversions/funvehiclemotionstateconverter.h"
#include "conversions/indicatorcommandconverter.h"
#include "conversions/vehiclebasemeasurementwriter.h"
#include "conversions/planningresultconverter.h"
#include "conversions/planningrequestconverter.h"
#include "conversions/missiondataconverter.h"
#include "conversions/platooninginformationconverter.h"
#include "conversions/stdconverter.h"
namespace adore
{
    namespace if_ROS
    {
        /**
         *  ROS implementation for data transmission of FUN data.
         */
        class FUN_Factory : public adore::fun::AFactory
        {
            private:
                ros::NodeHandle* n_;

            public:
                FUN_Factory(ros::NodeHandle *n) : n_(n)
                {
                }

            public: //overloading virtual AFactory methods:
                //get updates on the navigation goal
                virtual adore::mad::AReader<adore::fun::NavigationGoal>* getNavigationGoalReader() override
                {
                    return new Reader<adore::fun::NavigationGoal,
                                      adore_if_ros_msg::NavigationGoalConstPtr,
                                      NavigationGoalConverter>(n_, "ENV/NavigationGoal", 1);
                }
                //get updates on the setpoint request
                virtual adore::mad::AReader<adore::fun::SetPointRequest>* getSetPointRequestReader() override
                {
                    return new Reader<adore::fun::SetPointRequest,
                                      adore_if_ros_msg::SetPointRequestConstPtr,
                                      SetPointRequestConverter>(n_, "FUN/SetPointRequest", 1);
                }
                //write updates on the setpoint request
                virtual adore::mad::AWriter<adore::fun::SetPointRequest>* getSetPointRequestWriter() override
                {
                    return new Writer<adore::fun::SetPointRequest,
                                      adore_if_ros_msg::SetPointRequest,
                                      SetPointRequestConverter>(n_, "FUN/SetPointRequest", 1);
                }
    			///get updates on the nominal trajectory
                virtual adore::mad::AReader<adore::fun::SetPointRequest>* getNominalTrajectoryReader() override
                {
                    return new Reader<adore::fun::SetPointRequest,
                                      adore_if_ros_msg::SetPointRequestConstPtr,
                                      SetPointRequestConverter>(n_, "FUN/NominalTrajectory", 1);
                }
    			///write updates on the nominal trajectory
                virtual adore::mad::AWriter<adore::fun::SetPointRequest>* getNominalTrajectoryWriter() override
                {
                    return new Writer<adore::fun::SetPointRequest,
                                      adore_if_ros_msg::SetPointRequest,
                                      SetPointRequestConverter>(n_, "FUN/NominalTrajectory", 1);
                }
                ///get updates on the odom setpoint request: this is used by tracking controller, if controller is operating in odom tracking mode
                virtual TSetPointRequestReader* getOdomSetPointRequestReader()override
                {
                    return new Reader<adore::fun::SetPointRequest,
                                      adore_if_ros_msg::SetPointRequestConstPtr,
                                      SetPointRequestConverter>(n_, "FUN/SetPointRequest_odom", 1);
                }
                ///write updates on the odom setpoint request: this is used by planner, if controller is operating in odom tracking mode
                virtual TSetPointRequestWriter* getOdomSetPointRequestWriter()override
                {
                    return new Writer<adore::fun::SetPointRequest,
                                      adore_if_ros_msg::SetPointRequest,
                                      SetPointRequestConverter>(n_, "FUN/SetPointRequest_odom", 1);
                }
                //get updates on the terminal request
                virtual adore::mad::AReader<adore::fun::TerminalRequest>* getTerminalRequestReader() override
                {
                    return new Reader<adore::fun::TerminalRequest,
                                      adore_if_ros_msg::TerminalRequestConstPtr,
                                      TerminalRequestConverter>(n_, "FUN/TerminalRequest", 1);
                }
                //write updates on the terminal request
                virtual adore::mad::AWriter<adore::fun::TerminalRequest>* getTerminalRequestWriter() override
                {
                    return new Writer<adore::fun::TerminalRequest,
                                      adore_if_ros_msg::TerminalRequest,
                                      TerminalRequestConverter>(n_, "FUN/TerminalRequest", 1);
                }
                //write a motion command
                virtual adore::mad::AWriter<adore::fun::MotionCommand>* getMotionCommandWriter() override
                {
                    return new MotionCommandWriter(n_,
                                                   "FUN/MotionCommand/acceleration",
                                                   "FUN/MotionCommand/steeringAngle", 1);
                }
                //read a motion command
                virtual adore::mad::AReader<adore::fun::MotionCommand>* getMotionCommandReader() override
                {
                    return new MotionCommandReader(n_,
                                                   "FUN/MotionCommand/acceleration",
                                                   "FUN/MotionCommand/steeringAngle", 1);
                }

                //write a gear selection command
                virtual adore::mad::AWriter<adore::fun::GearSelectionCommand>* getGearSelectionCommandWriter() override
                {
                    return new Writer<adore::fun::GearSelectionCommand,
                                      std_msgs::Int8,
                                      GearSelectionCommandConverter>(n_, "FUN/GearSelectionCommand", 1);
                }
                //read a gear selection command
                virtual adore::mad::AReader<adore::fun::GearSelectionCommand>* getGearSelectionCommandReader() override
                {
                    return new Reader<adore::fun::GearSelectionCommand,
                                      std_msgs::Int8ConstPtr,
                                      GearSelectionCommandConverter>(n_, "FUN/GearSelectionCommand", 1);
                }
                //write an indicator command
                virtual adore::mad::AWriter<adore::fun::IndicatorCommand>* getIndicatorCommandWriter() override
                {
                    return new IndicatorCommandWriter(n_, "FUN/IndicatorCommand/left", "FUN/IndicatorCommand/right", 1);
                }
                //read an indicator command
                virtual adore::mad::AReader<adore::fun::IndicatorCommand>* getIndicatorCommandReader() override
                {
                    return new IndicatorCommandReader(n_, "FUN/IndicatorCommand/left", "FUN/IndicatorCommand/right", 1);
                }
                //get updates on the vehicle motion state
                virtual adore::fun::AFactory::TMotionStateReader* getVehicleMotionStateReader() override
                {
                    return new MotionStateReader(n_, "localization", "VEH/ax", "VEH/steering_angle_measured", 1);
                }
                ///get updates on the vehicle motion state: best estimate from odometry module
                virtual adore::fun::AFactory::TMotionStateReader* getVehicleOdometryMotionStateReader() override
                {
                    return new MotionStateReader(n_, "odom", "VEH/ax", "VEH/steering_angle_measured", 1);
                }
                ///get updates on the vehicle motion state: best estimate from odometry module
                virtual adore::fun::AFactory::TMotionStateFeed* getVehicleOdometryMotionStateFeed() override
                {
                    return new MotionStateFeed(n_, "odom", "VEH/ax", "VEH/steering_angle_measured", 1);
                }
                ///get updates on the vehicle motion state: best estimate from localization module
                virtual TMotionStateFeed* getVehicleLocalizationMotionStateFeed() override
                {
                    return new MotionStateFeed(n_, "localization", "VEH/ax", "VEH/steering_angle_measured", 1);
                }

                //get updates on the vehicle extended state (buttons, etc.)
                virtual adore::mad::AReader<adore::fun::VehicleExtendedState>* getVehicleExtendedStateReader() override
                {
                    return new VehicleExtendedStateReader(n_, 
                                            "VEH/gear_state",
                                            "VEH/AutomaticControlState/acceleration",
                                            "VEH/AutomaticControlState/accelerationActive",
                                            "VEH/AutomaticControlState/steering",
                                            "VEH/IndicatorState/left",
                                            "VEH/IndicatorState/right",
                                            "VEH/Checkpoints/clearance",                     
                                         1);
                }
                //get updates on the vehicle extended state (buttons, etc.)
                virtual adore::mad::AWriter<adore::fun::VehicleExtendedState>* getVehicleExtendedStateWriter() override
                {
                    return new VehicleExtendedStateWriter(n_, 
                                            "VEH/gear_state",
                                            "VEH/AutomaticControlState/acceleration",
                                            "VEH/AutomaticControlState/accelerationActive",
                                            "VEH/AutomaticControlState/steering",
                                            "VEH/IndicatorState/left",
                                            "VEH/IndicatorState/right",
                                            "VEH/Checkpoints/clearance",                    
                                        1);
                }
                ///writes measurements of base vehicle system into automation system
                virtual adore::mad::AWriter<adore::fun::VehicleBaseMeasurement>* getVehicleBaseMeasurementWriter() override
                {
                    return new VehicleBaseMeasurementWriter(n_,
                                                            "VEH/steering_angle_measured",
                                                            "VEH/wheel_speed",
                                                            "VEH/yaw_rate",
                                                            "VEH/ax",
                                                            "VEH/ay", 1);
                }
                ///writes PlanningResult as general information about decision making and planning performance
                virtual adore::mad::AWriter<adore::fun::PlanningResult>* getPlanningResultWriter() override
                {
                    return new Writer<adore::fun::PlanningResult,
                                      adore_if_ros_msg::PlanningResult,
                                      PlanningResultConverter>(n_, "FUN/PlanningResult", 10 );
                }
                ///writes PlanningResult for maneuver selected for execution
                virtual adore::mad::AWriter<adore::fun::PlanningResult>* getPlanningSelectWriter() override
                {
                    return new Writer<adore::fun::PlanningResult,
                                      adore_if_ros_msg::PlanningResult,
                                      PlanningResultConverter>(n_, "FUN/PlanningSelect", 10 );
                }
                ///reads PlanningResult as general information about decision making and planning performance
                virtual adore::mad::AFeed<adore::fun::PlanningResult>* getPlanningResultFeed() override
                {
                    return new Feed<adore::fun::PlanningResult,
                                      adore_if_ros_msg::PlanningResultConstPtr,
                                      PlanningResultConverter>(n_, "FUN/PlanningResult", 100);
                }    
                virtual adore::mad::AWriter<adore::fun::PlanningRequest>* getPlanningRequestWriter() override
                {
                    return new Writer<adore::fun::PlanningRequest,
                                      adore_if_ros_msg::PlanningRequest,
                                      PlanningRequestConverter>(n_, "FUN/PlanningRequest", 1 );
                }
                virtual adore::mad::AFeedWithCallback<adore::fun::PlanningRequest>* getPlanningRequestTrigger() override
                {
                    return new FeedWithCallback<adore::fun::PlanningRequest,
                                        adore_if_ros_msg::PlanningRequest,
                                        PlanningRequestConverter>(n_,"FUN/PlanningRequest", 1);
                }
                ///write mission data
                virtual adore::mad::AWriter<adore::fun::MissionData>* getMissionDataWriter() override
                {
                    return new MissionDataWriter(n_, "FUN/mission_data/mission_state", 1);

                }
                ///read mission data 
                virtual adore::mad::AReader<adore::fun::MissionData>* getMissionDataReader() override
                {
                    return new MissionDataReader(n_, "FUN/mission_data/mission_state", 1);
                }
                //get updates on the platooning state
                virtual adore::mad::AReader<adore::fun::PlatooningInformation>* getPlatooningStateReader() override
                {
                    return new Reader<adore::fun::PlatooningInformation,
                                      adore_if_ros_msg::CooperativePlanningConstPtr,
                                      PlatooningInformationConverter>(n_, "FUN/PlatooningInformation", 1);
                }
                //write updates on the platooning state 
                virtual adore::mad::AWriter<adore::fun::PlatooningInformation>* getPlatooningStateWriter() override
                {
                    return new Writer<adore::fun::PlatooningInformation,
                                      adore_if_ros_msg::CooperativePlanning,
                                      PlatooningInformationConverter>(n_, "FUN/PlatooningInformation", 1);
                }
                //TODO: instead of seperate packages this should be compounded in a user input message
                //read lanechange suppression msgs
                virtual adore::mad::AReader<bool>* getLanechangeSuppressionReader() override
                {
                    return new Reader<bool,
                                      std_msgs::BoolPtr,
                                      StdConverter>(n_, "FUN/LangeChangeSuppresion", 1);
                }
                //read force lanechange left msgs
                virtual adore::mad::AReader<bool>* getForceLanechangeLeftReader() override
                {
                    return new Reader<bool,
                                      std_msgs::BoolPtr,
                                      StdConverter>(n_, "FUN/ForceLanechange/left", 1);
                }
                //read force lanechange right msgs
                virtual adore::mad::AReader<bool>* getForceLanechangeRightReader() override
                {
                    return new Reader<bool,
                                      std_msgs::BoolPtr,
                                      StdConverter>(n_, "FUN/ForceLanechange/right", 1);
                }
                //read force slow maneuvers msgs
                virtual adore::mad::AReader<bool>* getForceSlowManeuversReader() override
                {
                    return new Reader<bool,
                                      std_msgs::BoolPtr,
                                      StdConverter>(n_, "FUN/ForceSlowManeuvers", 1);
                }
            };          
    } // namespace if_ROS
} // namespace adore