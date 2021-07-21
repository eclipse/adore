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
			///get updates on the navigation goal
			virtual adore::mad::AReader<NavigationGoal>* getNavigationGoalReader()=0;
			///get updates on the setpoint request
			virtual adore::mad::AReader<SetPointRequest>* getSetPointRequestReader()=0;
			///write updates on the setpoint request
			virtual adore::mad::AWriter<SetPointRequest>* getSetPointRequestWriter()=0;
			///get updates on the nominal trajectory
			virtual adore::mad::AReader<SetPointRequest>* getNominalTrajectoryReader()=0;
			///write updates on the nominal trajectory
			virtual adore::mad::AWriter<SetPointRequest>* getNominalTrajectoryWriter()=0;
			///get updates on the terminal request
			virtual adore::mad::AReader<TerminalRequest>* getTerminalRequestReader()=0;
			///write updates on the terminal request
			virtual adore::mad::AWriter<TerminalRequest>* getTerminalRequestWriter()=0;
			///write a motion command
			virtual adore::mad::AWriter<MotionCommand>* getMotionCommandWriter()=0;
			///read a motion command 
			virtual adore::mad::AReader<MotionCommand>* getMotionCommandReader()=0;
			///write a gear selection command
			virtual adore::mad::AWriter<GearSelectionCommand>* getGearSelectionCommandWriter()=0;
			///read a gear selection command
			virtual adore::mad::AReader<GearSelectionCommand>* getGearSelectionCommandReader()=0;
			///write an indicator command
			virtual adore::mad::AWriter<IndicatorCommand>* getIndicatorCommandWriter()=0;
			///read an indicator command
			virtual adore::mad::AReader<IndicatorCommand>* getIndicatorCommandReader()=0;
			///get updates on the vehicle motion state
			virtual adore::mad::AReader<VehicleMotionState9d>* getVehicleMotionStateReader()=0;
			///get updates on the vehicle extended state (buttons, etc.)
			virtual adore::mad::AReader<VehicleExtendedState>* getVehicleExtendedStateReader()=0;
			///write updates on the vehicle extended state (buttons, etc.)
			virtual adore::mad::AWriter<VehicleExtendedState>* getVehicleExtendedStateWriter()=0;
			///writes measurements of base vehicle system into automation system
			virtual adore::mad::AWriter<VehicleBaseMeasurement>* getVehicleBaseMeasurementWriter()=0;
			///writes PlanningResult as general information about decision making and planning performance
			virtual adore::mad::AWriter<PlanningResult>* getPlanningResultWriter()=0;
			///reads PlanningResult as general information about decision making and planning performance
			virtual adore::mad::AFeed<PlanningResult>* getPlanningResultFeed()=0;
			///writes PlanningRequest to orchestrate multiple trajectory planners
			virtual adore::mad::AWriter<PlanningRequest>* getPlanningRequestWriter()=0;
      ///allows to trigger planning, when planning request is received
      virtual adore::mad::AFeedWithCallback<PlanningRequest>* getPlanningRequestTrigger()=0;
      ///write mission data
			virtual adore::mad::AWriter<MissionData>* getMissionDataWriter()=0;
			///read mission data 
			virtual adore::mad::AReader<MissionData>* getMissionDataReader()=0;
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
