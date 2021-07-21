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

#include <string>
#include <adore/params/ap_traffic_light_sim.h>

#pragma once
namespace adore
{
	namespace params
	{
	/**
		 * @brief a dummy implementation for testing purposes
		 * 
		 */
		class APTrafficLightSimDummy: public APTrafficLightSim
		{

		public:

			virtual int getRedDuration()const override
			{
				return 8000;
			}
            virtual int getGreenDuration()const override
			{
				return 8000;
			}
			virtual int getRedYellowDuration()const override
			{
				return 2000;
			}
			virtual int getYellowDuration()const override
			{
				return 2000;
			}
			virtual std::string getStartState()const override
			{
				return "r";
			}

			virtual bool getIsProbeMode()const override
			{
				return false;
			}
		};

	}
}