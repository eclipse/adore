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
#include <adore/fun/navigationgoal.h>
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
#include <adore/sim/schedulernotification.h>
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
			typedef adore::mad::AWriter<adore::fun::VehicleExtendedState> TVehicleExtendedStateWriter;
			typedef adore::mad::AFeed<adore::sim::ResetVehiclePose> TVehiclePoseResetFeed;
			typedef adore::mad::AFeed<adore::sim::ResetVehicleTwist> TVehicleTwistResetFeed;
			typedef adore::mad::AWriter<adore::env::traffic::TParticipantSet> TParticipantSetWriter;
			typedef adore::mad::AWriter<adore::env::traffic::Participant> TParticipantWriter;
			typedef adore::mad::AFeed<adore::env::traffic::Participant> TParticipantFeed;
			typedef adore::mad::AFeedWithCallback<adore::sim::SchedulerNotification> TSchedulerNotificationFeed;
			typedef adore::mad::AWriter<adore::sim::SchedulerNotification> TSchedulerNotificationWriter;
			typedef adore::mad::AWriter<std::pair<uint32_t,uint32_t>> TClockTimeWriter;
			typedef adore::mad::AWriter<adore::env::SimTrafficLight> TSimTrafficLightWriter;
			typedef adore::mad::AReader<adore::env::SimTrafficLightMap> TSimTrafficLightReader;



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
			///write updates on the vehicle motion state
			virtual TVehicleMotionStateWriter* getVehicleMotionStateWriter()=0;
			///write updates on the vehicle extended state (buttons, etc.)
			virtual TVehicleExtendedStateWriter* getVehicleExtendedStateWriter()=0;
			///read simulation commands for vehicle position and orientation resetting
			virtual TVehiclePoseResetFeed* getVehiclePoseResetFeed()=0;
			///read simulation commands for vehicle speed resetting
			virtual TVehicleTwistResetFeed* getVehicleTwistResetFeed()=0;
			///send simulated sensor data
			virtual TParticipantSetWriter* getParticipantSetWriter()=0;
			///send ego state to simulation feed
			virtual TParticipantWriter* getParticipantWriter()=0;
			///get state updates from all vehicles 
			virtual TParticipantFeed* getParticipantFeed()=0;
			///send scheduler notification message
			virtual TSchedulerNotificationWriter* getSchedulerNotificationWriter()=0;
			///get notifications for scheduler
			virtual TSchedulerNotificationFeed* getSchedulerNotificationFeed()=0;
			///send clock time
			virtual TClockTimeWriter* getClockTimeWriter()=0;
			///send simulated traffic light states  
			virtual TSimTrafficLightWriter* getTrafficLightWriter()=0;
			///receive simulated traffic light states 
			virtual TSimTrafficLightReader* getTrafficLightReader()=0;
		};
	}
}