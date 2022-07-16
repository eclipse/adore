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
#include <adore/params/ap_trajectory_generation.h>

namespace adore
{
	namespace params
	{
		/**
		 * @brief a dummy implementation for testing purposes
		 * 
		 */
		class APTrajectoryGenerationDummy: public APTrajectoryGeneration
		{

		public:

			virtual double get_rho()const
      {
        return 2.7;
      }
			///zero dynamics integration length
			virtual double getZDIntegrationLength()const override
			{
				return 10.0;
			}
			///zero dynamics step size
			virtual double getZDIntegrationStep()const override
			{
				return 0.01;
			}
			virtual int getSetPointCount()const override
			{
				return 100;
			}
			///time after which emergency maneuver kicks in
			virtual double getEmergencyManeuverDelay()const override
			{
				return 0.5;
			}
		};
	}
}