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
#include <adore/fun/vehiclebasemeasurement.h>

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
			///write an indicator command
			virtual adore::mad::AWriter<IndicatorCommand>* getIndicatorCommandWriter()=0;
			///get updates on the vehicle motion state
			virtual adore::mad::AReader<VehicleMotionState9d>* getVehicleMotionStateReader()=0;
			///get updates on the vehicle extended state (buttons, etc.)
			virtual adore::mad::AReader<VehicleExtendedState>* getVehicleExtendedStateReader()=0;
			///write updates on the vehicle extended state (buttons, etc.)
			virtual adore::mad::AWriter<VehicleExtendedState>* getVehicleExtendedStateWriter()=0;
			///writes measurements of base vehicle system into automation system
			virtual adore::mad::AWriter<VehicleBaseMeasurement>* getVehicleBaseMeasurementWriter()=0;

		};
	}
}