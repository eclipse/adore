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
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/

#pragma once

namespace adore
{
	namespace params
	{
		/**
		 * @brief abstract class containing parameters to configure the behaviour of trajectory generation
		 * 
		 */
		class APTrajectoryGeneration
		{

		public:

			///cor to planning point: movement of planning point shall planned by the trajectory planner
			virtual double get_rho()const =0;

			///zero dynamics integration length
			virtual double getZDIntegrationLength()const =0;

			///zero dynamics step size
			virtual double getZDIntegrationStep()const =0;
			
			///number of set points in set-point request
			virtual int getSetPointCount()const =0;

			///time after which emergency maneuver kicks in
			virtual double getEmergencyManeuverDelay()const =0;
		};
	}
}