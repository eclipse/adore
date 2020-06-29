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

#pragma once
namespace adore
{
	namespace params
	{
		/**
		 * @brief abstract class containing parameters to configure aspects of the map provider
		 * 
		 */
		class APTrafficLightSim
		{
		public:
			//visibility radius of the map provider
			virtual int getRedDuration()const =0;
            virtual int getGreenDuration()const  =0;
			virtual int getRedYellowDuration()const =0;
			virtual int getYellowDuration()const=0;
			virtual std::string getStartState()const =0;
			virtual bool getIsProbeMode()const=0;
		};
	}
}