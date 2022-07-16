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
		 * @brief abstract class containing parameters which configure navigation behaviour
		 * 
		 */
		class APNavigation
		{
		public:
			virtual bool getActivePlottingLocal() = 0;
			virtual bool getActivePlottingGlobal() = 0;
			virtual double getLaneChangePenalty() const = 0;
			virtual bool useScenarioManagerMap()=0;
			virtual double getNagivationGoalStopToleranceLateral()const =0;
		};
	}
}