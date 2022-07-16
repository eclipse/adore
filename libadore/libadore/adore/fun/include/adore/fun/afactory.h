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

#include <adore/fun/gearselectioncommand.h>
#include <adore/fun/indicatorcommand.h>
#include <adore/fun/missiondata.h>
#include <adore/fun/motioncommand.h>
#include <adore/view/navigationgoal.h>
#include <adore/fun/setpointrequest.h>
#include <adore/fun/terminalrequest.h>
#include <adore/fun/vehiclebasemeasurement.h>
#include <adore/fun/vehicleextendedstate.h>
#include <adore/fun/vehiclemotionstate9d.h>
#include <adore/fun/tac/planning_result.h>
#include <adore/fun/tac/planning_request.h>
#include <adore/mad/com_patterns.h>
#include <adore/fun/platooningInformation.h>

namespace adore
{
	namespace fun
	{
		/**
		 * Abstract factory for adore::fun communication
		 */
		class AFactory
		{
      public:
      typedef adore::mad::AReader<NavigationGoal> TNavigationGoalReader;
      typedef adore::mad::AReader<SetPointRequest> TSetPointRequestReader;
      typedef adore::mad::AWriter<SetPointRequest> TSetPointRequestWriter;
      typedef adore::mad::AReader<VehicleMotionState9d> TMotionStateReader;
      typedef adore::mad::AFeed<VehicleMotionState9d> TMotionStateFeed;
      typedef adore::mad::AReader<TerminalRequest> TTerminalRequestReader;
      typedef adore::mad::AWriter<TerminalRequest> TTerminalRequestWriter;
      typedef adore::mad::AWriter<MotionCommand> TMotionCommandWriter;
      typedef adore::mad::AReader<MotionCommand> TMotionCommandReader;
      typedef adore::mad::AWriter<GearSelectionCommand> TGearSelectionCommandWriter;
      typedef adore::mad::AReader<GearSelectionCommand> TGearSelectionCommandReader;
      typedef adore::mad::AWriter<IndicatorCommand> TIndicatorCommandWriter;
      typedef adore::mad::AReader<IndicatorCommand> TIndicatorCommandReader;
      typedef adore::mad::AReader<VehicleExtendedState> TVehicleExtendedStateReader;
      typedef adore::mad::AWriter<VehicleExtendedState> TVehicleExtendedStateWriter;
      typedef adore::mad::AWriter<VehicleBaseMeasurement> TVehicleBaseMeasurementWriter;
      typedef adore::mad::AWriter<PlanningResult> TPlanningResultWriter;
      typedef adore::mad::AFeed<PlanningResult> TPlanningResultFeed;
      typedef adore::mad::AWriter<PlanningRequest> TPlanningRequestWriter; 
      typedef adore::mad::AFeedWithCallback<PlanningRequest> TPlanningRequestTrigger;
      typedef adore::mad::AWriter<MissionData> TMissionDataWriter;
      typedef adore::mad::AReader<MissionData> TMissionDataReader;
      typedef adore::mad::AReader<PlatooningInformation> TPlatooningStateReader;
      typedef adore::mad::AWriter<PlatooningInformation> TPlatooningStateWriter;
      typedef adore::mad::AReader<bool> TLanechangeSuppressionReader;
      typedef adore::mad::AReader<bool> TForceLanechangeLeftReader; 
      typedef adore::mad::AReader<bool> TForceLanechangeRightReader; 
      typedef adore::mad::AReader<bool> TForceSlowManeuversReader;


		  public:
			///get updates on the navigation goal
			virtual TNavigationGoalReader* getNavigationGoalReader()=0;
			///get updates on the setpoint request
			virtual TSetPointRequestReader* getSetPointRequestReader()=0;
			///write updates on the setpoint request
			virtual TSetPointRequestWriter* getSetPointRequestWriter()=0;
			///get updates on the nominal trajectory
			virtual TSetPointRequestReader* getNominalTrajectoryReader()=0;
			///write updates on the nominal trajectory
			virtual TSetPointRequestWriter* getNominalTrajectoryWriter()=0;
			///get updates on the odom setpoint request: this is used by tracking controller, if controller is operating in odom tracking mode
			virtual TSetPointRequestReader* getOdomSetPointRequestReader()=0;
			///write updates on the odom setpoint request: this is used by planner, if controller is operating in odom tracking mode
			virtual TSetPointRequestWriter* getOdomSetPointRequestWriter()=0;
			///get updates on the terminal request
			virtual TTerminalRequestReader* getTerminalRequestReader()=0;
			///write updates on the terminal request
			virtual TTerminalRequestWriter* getTerminalRequestWriter()=0;
			///write a motion command
			virtual TMotionCommandWriter* getMotionCommandWriter()=0;
			///read a motion command 
			virtual TMotionCommandReader* getMotionCommandReader()=0;
			///write a gear selection command
			virtual TGearSelectionCommandWriter* getGearSelectionCommandWriter()=0;
			///read a gear selection command
			virtual TGearSelectionCommandReader* getGearSelectionCommandReader()=0;
			///write an indicator command
			virtual TIndicatorCommandWriter* getIndicatorCommandWriter()=0;
			///read an indicator command
			virtual TIndicatorCommandReader* getIndicatorCommandReader()=0;
			///get updates on the vehicle motion state: best estimate from localization module
			virtual TMotionStateReader* getVehicleMotionStateReader()=0;
			///get updates on the vehicle motion state: best estimate from odometry module
			virtual TMotionStateReader* getVehicleOdometryMotionStateReader()=0;
			///get updates on the vehicle motion state: best estimate from odometry module
			virtual TMotionStateFeed* getVehicleOdometryMotionStateFeed()=0;
			///get updates on the vehicle motion state: best estimate from localization module
			virtual TMotionStateFeed* getVehicleLocalizationMotionStateFeed()=0;
			///get updates on the vehicle extended state (buttons, etc.)
			virtual TVehicleExtendedStateReader* getVehicleExtendedStateReader()=0;
			///write updates on the vehicle extended state (buttons, etc.)
			virtual TVehicleExtendedStateWriter* getVehicleExtendedStateWriter()=0;
			///writes measurements of base vehicle system into automation system
			virtual TVehicleBaseMeasurementWriter* getVehicleBaseMeasurementWriter()=0;
			///writes PlanningResult as general information about decision making and planning performance
			virtual TPlanningResultWriter* getPlanningResultWriter()=0;
			///reads PlanningResult as general information about decision making and planning performance
			virtual TPlanningResultFeed* getPlanningResultFeed()=0;
			///writes PlanningResult for maneuver selected for execution
			virtual TPlanningResultWriter* getPlanningSelectWriter()=0;
			///writes PlanningRequest to orchestrate multiple trajectory planners
			virtual TPlanningRequestWriter* getPlanningRequestWriter()=0;
      ///allows to trigger planning, when planning request is received
      virtual TPlanningRequestTrigger* getPlanningRequestTrigger()=0;
      ///write mission data
			virtual TMissionDataWriter* getMissionDataWriter()=0;
			///read mission data 
			virtual TMissionDataReader* getMissionDataReader()=0;
      ///get updates on the platooning state
			virtual TPlatooningStateReader* getPlatooningStateReader()=0;
			///write updates on the platooning state
			virtual TPlatooningStateWriter* getPlatooningStateWriter()=0;    
      //read lanechange suppression msgs
      virtual TLanechangeSuppressionReader* getLanechangeSuppressionReader()=0;
      //read force lanechange left  msgs
      virtual TForceLanechangeLeftReader*  getForceLanechangeLeftReader()=0; 
      //read force lanechange right msgs
      virtual TForceLanechangeRightReader* getForceLanechangeRightReader()=0; 
      //read force slow maneuvers msgs
      virtual TForceSlowManeuversReader* getForceSlowManeuversReader()=0; 
    };


        /**
         * @brief Utility class to simplify factory access
         *
         * init() function should be used once per process to set factories
         */
        class FunFactoryInstance final
        {
          private:
            adore::fun::AFactory* funFactory_ = 0;

            /**
             * @brief Function to access singleton instance of the funFactory using magic static
             *
             * @return funFactory& reference on the threadsafe singleton instance
             */
            static FunFactoryInstance& getInstance()
            {
                static FunFactoryInstance instance;
                return instance;
            }

          public:
            static adore::fun::AFactory* get()
            {
                if (getInstance().funFactory_ == 0)
                {
                    std::cerr << " WARNING Accessing singleton funFactory instance without ever running init()"
                              << std::endl;
                }
                return getInstance().funFactory_;
            }

            /**
             * @brief Initialize private members of funFactory
             *
             * This function should ideally run only once per process before the singleton instance is used
             * Makes no guarantees on thread-safety
             *
             * @param funFactory
             */
            static void init(adore::fun::AFactory* funFactory)
            {
                auto& instance = getInstance();
                instance.funFactory_ = funFactory;
            }

          private:
            FunFactoryInstance() = default;
            ~FunFactoryInstance() = default;

            FunFactoryInstance(const FunFactoryInstance&) = delete;
            FunFactoryInstance& operator=(const FunFactoryInstance&) = delete;
            FunFactoryInstance(FunFactoryInstance&&) = delete;
            FunFactoryInstance& operator=(FunFactoryInstance&&) = delete;
        };

    }  // namespace fun
}  // namespace adore
