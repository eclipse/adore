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
#include <adore/params/ap_lane_following_view.h>
namespace adore
{
	namespace params
	{
		/**
		 * @brief a dummy implementation for testing purposes
		 * 
		 */
		class APLaneFollowingViewDummy:public APLaneFollowingView
		{
		public:
			virtual double getLookAhead()const override
			{
				return 150.0;
			}
			virtual double getLookBehind()const override
			{
				return 5.0;
			}

			virtual double getPlanningTime()const override
			{
				return 10.0;
			}

			virtual double getBaselineFitSmoothness()const override
			{
				return 0.05;
			}

		};
	}
}