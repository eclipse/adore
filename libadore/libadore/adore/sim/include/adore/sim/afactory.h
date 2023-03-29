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

#include <adore/mad/com_patterns.h>
#include <adore/view/navigationgoal.h>
#include <adore/fun/setpointrequest.h>
#include <adore/fun/terminalrequest.h>
#include <adore/fun/motioncommand.h>
#include <adore/fun/gearselectioncommand.h>
#include <adore/fun/indicatorcommand.h>
#include <adore/fun/vehiclemotionstate9d.h>
#include <adore/fun/vehicleextendedstate.h>
#include <adore/sim/resetvehiclepose.h>
#include <adore/sim/resetvehicletwist.h>
#include <adore/env/traffic/participant.h>

#include <adore/env/tcd/trafficlight.h>

namespace adore 
{
	/**
	 * @brief this namespace contains functionality related to running simulations
	 * 
	 */
	namespace sim
	{
		///abstract factory for adore::sim communication
		class AFactory
		{
		public:
			typedef adore::mad::AReader<adore::fun::MotionCommand> TMotionCommandReader;
			typedef adore::mad::AReader<adore::fun::GearSelectionCommand> TGearSelectionCommandReader;
			typedef adore::mad::AReader<adore::fun::IndicatorCommand> TIndicatorCommandReader;
			typedef adore::mad::AReader<double> TSimulationTimeReader;
			typedef adore::mad::AWriter<double> TSimulationTimeWriter;
			typedef adore::mad::AWriter<adore::fun::VehicleMotionState9d> TVehicleMotionStateWriter;
			typedef adore::mad::AReader<adore::fun::VehicleMotionState9d> TVehicleMotionStateReader;
			typedef adore::mad::AWriter<adore::fun::VehicleExtendedState> TVehicleExtendedStateWriter;
			typedef adore::mad::AFeed<adore::sim::ResetVehiclePose> TVehiclePoseResetFeed;
			typedef adore::mad::AFeed<adore::sim::ResetVehicleTwist> TVehicleTwistResetFeed;
			typedef adore::mad::AWriter<adore::sim::ResetVehiclePose> TVehiclePoseResetWriter;
			typedef adore::mad::AWriter<adore::sim::ResetVehicleTwist> TVehicleTwistResetWriter;
			typedef adore::mad::AFeed<int64_t> TSimulationIDResetFeed;
			typedef adore::mad::AFeed<int64_t> TV2XStationIDResetFeed;
			typedef adore::mad::AWriter<adore::env::traffic::TParticipantSet> TParticipantSetWriter;
			typedef adore::mad::AWriter<adore::env::traffic::Participant> TParticipantWriter;
			typedef adore::mad::AFeed<adore::env::traffic::Participant> TParticipantFeed;
			typedef adore::mad::AWriter<std::pair<uint32_t,uint32_t>> TClockTimeWriter;
			typedef adore::mad::AWriter<adore::env::SimTrafficLight> TSimTrafficLightWriter;
			typedef adore::mad::AReader<adore::env::SimTrafficLightMap> TSimTrafficLightReader;
			typedef adore::mad::AFeedWithCallback<std::string> TSimulationCoordinationFeed;
			typedef adore::mad::AWriter<std::string> TSimulationCoordinationWriter;


			///read a motion command
			virtual TMotionCommandReader* getMotionCommandReader()=0;
			///read a gear selection command
			virtual TGearSelectionCommandReader* getGearSelectionCommandReader()=0;
			///read an indicator command
			virtual TIndicatorCommandReader* getIndicatorCommandReader()=0;
			///read the simulation time
			virtual TSimulationTimeReader* getSimulationTimeReader()=0;
			///write the simulation time
			virtual TSimulationTimeWriter* getSimulationTimeWriter()=0;
			///write updates on the true vehicle motion state
			virtual TVehicleMotionStateWriter* getVehicleMotionStateWriter()=0;
			///write updates on the odometry estimated vehicle motion state
			virtual TVehicleMotionStateWriter* getOdometryEstimatedVehicleStateWriter()=0;
			///write updates on the localization estimated vehicle motion state
			virtual TVehicleMotionStateWriter* getLocalizationEstimatedVehicleStateWriter()=0;
			///read updates on the true vehicle motion state
			virtual TVehicleMotionStateReader* getVehicleMotionStateReader()=0;
			///write updates on the vehicle extended state (buttons, etc.)
			virtual TVehicleExtendedStateWriter* getVehicleExtendedStateWriter()=0;
			///read simulation commands for vehicle position and orientation resetting
			virtual TVehiclePoseResetFeed* getVehiclePoseResetFeed()=0;
			///read simulation commands for vehicle speed resetting
			virtual TVehicleTwistResetFeed* getVehicleTwistResetFeed()=0;
			//send simulation commands for vehicle position and orientation resetting
			virtual TVehiclePoseResetWriter* getVehiclePoseResetWriter()=0;
			///send simulation commands for vehicle speed resetting
			virtual TVehicleTwistResetWriter* getVehicleTwistResetWriter()=0;
			///read simulation commands for simulation id resetting
			virtual TSimulationIDResetFeed* getSimulationIDResetFeed() = 0;
			///read simulation commands for v2x station id resetting
            virtual TV2XStationIDResetFeed* getV2XStationIDResetFeed() = 0;
			///send simulated sensor data
			virtual TParticipantSetWriter* getParticipantSetWriter()=0;
			///send ego state to simulation feed
			virtual TParticipantWriter* getParticipantWriter()=0;
			///get state updates from all vehicles 
			virtual TParticipantFeed* getParticipantFeed()=0;
			///write clock time
			virtual TClockTimeWriter* getClockTimeWriter()=0;
			///send simulated traffic light states  
			virtual TSimTrafficLightWriter* getTrafficLightWriter()=0;
			///receive simulated traffic light states 
			virtual TSimTrafficLightReader* getTrafficLightReader()=0;
			///send id of vehicle to transform
			virtual adore::mad::AWriter<int64_t>* getTransformIDtoAdoreWriter()=0;
		};

		   /**
         * @brief Utility class to simplify factory access
         *
         * init() function should be used once per process to set factories
         */
        class SimFactoryInstance final
        {
          private:
            adore::sim::AFactory* factory_ = 0;
 
            /**
             * @brief Function to access singleton instance of the AllFactory using magic static
             *
             * @return AllFactory& reference on the threadsafe singleton instance
             */
            static SimFactoryInstance& getInstance()
            {
                static SimFactoryInstance instance;
                return instance;
            }

          public:
            static adore::sim::AFactory* get()
            {
              if (getInstance().factory_ == nullptr)
              {
                std::cerr << " WARNING Accessing singleton simFactory instance without ever running init()" 
                      << std::endl;
              }
              return getInstance().factory_;
            }

            /**
             * @brief Initialize private members of AllFactory
             *
             * This function should ideally run only once per process before the singleton instance is used
             * Makes no guarantees on thread-safety
             *
             */
            static void init(adore::sim::AFactory* factory)
            {
                auto& instance = getInstance();
                instance.factory_ = factory;	
            }

          private:
            SimFactoryInstance() = default;
            ~SimFactoryInstance() = default;

            SimFactoryInstance(const SimFactoryInstance&) = delete;
            SimFactoryInstance& operator=(const SimFactoryInstance&) = delete;
            SimFactoryInstance(SimFactoryInstance&&) = delete;
            SimFactoryInstance& operator=(SimFactoryInstance&&) = delete;
        };
	}
}