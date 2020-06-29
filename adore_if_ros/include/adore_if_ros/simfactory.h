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


#include <adore/sim/afactory.h>
#include <adore_if_ros/ros_com_patterns.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "conversions/gearselectioncommandconverter.h"
#include "conversions/indicatorcommandconverter.h"
#include "conversions/motioncommandconverter.h"
#include "conversions/funvehiclemotionstateconverter.h"
#include "conversions/simvehicleresetconverter.h"
#include "conversions/stdconverter.h"
#include "conversions/trafficparticipantconverter.h"
#include "conversions/trafficsimulationfeed.h"
#include "conversions/simschedulernotificationconverter.h"
#include "conversions/clocktimeconverter.h"
#include "conversions/trafficlightsimconverter.h"



namespace adore
{
    namespace if_ROS
    {
        /**
         *  ROS implementation for data transmission of SIM data.
         */
        class SIM_Factory:public adore::sim::AFactory
        {
            private:
                ros::NodeHandle* n_;
            public:
                SIM_Factory(ros::NodeHandle* n):n_(n)
                {

                }

            public: //overloaded methods
                //read a motion command
                virtual TMotionCommandReader* getMotionCommandReader()override
                {
                    return new MotionCommandReader(n_,"FUN/MotionCommand/acceleration","FUN/MotionCommand/steeringAngle",1);
                }
                //read a gear selection command
                virtual TGearSelectionCommandReader* getGearSelectionCommandReader()override
                {
                    return new Reader<adore::fun::GearSelectionCommand,
                                     std_msgs::Int8ConstPtr,
                                     GearSelectionCommandConverter>(n_,"FUN/GearSelectionCommand",1);
                }
                //read an indicator command
                virtual TIndicatorCommandReader* getIndicatorCommandReader()override
                {
                    return new IndicatorCommandReader(n_,"FUN/IndicatorCommand/left","FUN/IndicatorCommand/right",1);
                }
                //write updates on the vehicle motion state
                virtual TVehicleMotionStateWriter* getVehicleMotionStateWriter()override
                {
                    return new MotionStateWriter(n_,"odom","VEH/steering_angle_measured","VEH/ax",1);

                }
                //write updates on the vehicle extended state (buttons, etc.)
                virtual TVehicleExtendedStateWriter* getVehicleExtendedStateWriter()override
                {
                    return 0;

                }
                //read the simulation time
                virtual TSimulationTimeReader* getSimulationTimeReader() override
                {
                    return new Reader<double,std_msgs::Float64ConstPtr,StdConverter>(n_,"/SIM/utc",1);
                }

                //write the simulation time
                virtual TSimulationTimeWriter* getSimulationTimeWriter() override
                {
                    return new Writer<double,std_msgs::Float64,StdConverter>(n_,"/SIM/utc",1);
                }

                //write the clock time
                virtual TClockTimeWriter* getClockTimeWriter() override
                {
                    return new Writer<std::pair<uint32_t,uint32_t>,rosgraph_msgs::Clock,ClockTimeConverter>(n_,"/clock",1);
                }

                //read simulation commands for vehicle position and orientation resetting
                virtual TVehiclePoseResetFeed* getVehiclePoseResetFeed() override
                {
                    return new Feed<sim::ResetVehiclePose,
                                    geometry_msgs::PoseConstPtr,
                                    SimVehicleResetConverter>(n_,"SIM/ResetVehiclePose",1);
                }
                //read simulation commands for vehicle speed resetting
                virtual TVehicleTwistResetFeed* getVehicleTwistResetFeed() override
                {
                    return new Feed<sim::ResetVehicleTwist,
                                    geometry_msgs::TwistConstPtr,
                                    SimVehicleResetConverter>(n_,"SIM/ResetVehicleTwist",1);
                }
                ///send simulated sensor data
                virtual TParticipantSetWriter* getParticipantSetWriter() override
                {
                    return new Writer<adore::env::traffic::TParticipantSet,
                                        adore_if_ros_msg::TrafficParticipantSet,
                                        TPSetConverter>(n_,"traffic",1);
                }
                ///send ego state to simulation feed
                virtual TParticipantWriter* getParticipantWriter() override
                {
                    return new Writer<adore::env::traffic::Participant,
                                        adore_if_ros_msg::TrafficParticipantSimulation,
                                        TPSimulationConverter>(n_,"/SIM/traffic",1);
                }
                ///get state updates from all vehicles 
                virtual TParticipantFeed* getParticipantFeed()
                {
                    // return new Feed<adore::env::traffic::Participant,
                    //                 adore_if_ros_msg::TrafficParticipantSimulationConstPtr,
                    //                 TPSimulationConverter>(n_,"/SIM/traffic",1000);
                    return new TrafficSimulationFeed(n_,"/SIM/traffic",1000,"/SIM/traffic/agg",20);
                }

                virtual TSchedulerNotificationFeed* getSchedulerNotificationFeed()
                {
                    return new FeedWithCallback<adore::sim::SchedulerNotification,
                                    adore_if_ros_msg::SchedulerNotification,
                                    SimSchedulerNotificationConverter>(n_,"/SIM/scheduling",1000);
                    //return new Feed<CSA::SIM::
                }
                virtual TSchedulerNotificationWriter* getSchedulerNotificationWriter()
                {
                    //return new Feed<CSA:: return new Writer<double,std_msgs::Float64,StdConverter>(n_,"/SIM/utc",1);
                    return new Writer<adore::sim::SchedulerNotification,
                                    adore_if_ros_msg::SchedulerNotification,
                                    adore::if_ROS::SimSchedulerNotificationConverter> (n_,"/SIM/scheduling",1);

                }

                // send updates for simulated trafficlights
                virtual adore::mad::AWriter<adore::env::SimTrafficLight>* getTrafficLightWriter() override
                {
                    return new TrafficLightSimWriter(n_,"/SIM/SimTl",300);

                
                }
                // get updates for simulated trafficlights
                virtual adore::mad::AReader<adore::env::SimTrafficLightMap>* getTrafficLightReader() override
                {
                    return new TrafficLightSimReader(n_,"/SIM/SimTl",300);
                }
        };
    }
}