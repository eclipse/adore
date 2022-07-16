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

#include <adore/fun/planarvehiclestate10d.h>

namespace adore
{
	namespace fun
	{
		/**
		 * A single reference point for the vehicle.
		 * The SetPoint is valid during a time interval [tStart,tEnd].
		 * The reference state is given for tStart.
		 * The desired state at any other time t in [tStart,tEnd] can be attained by forwards integration of x0ref.
		 */
		class SetPoint
		{
		public:
			PlanarVehicleState10d x0ref;/**< reference vehicle state*/

			double tStart;/**< Point of time at which x0ref should be attained.*/
			double tEnd;/**< End of validity of the SetPoint*/
			int maneuverID;/**< ID of the maneuver issuing the SetPoint*/

			SetPoint()
			{
				maneuverID = 0;
			}
			virtual ~SetPoint()
			{
				
			}
			VehicleMotionState9d toMotionState()const
			{
				VehicleMotionState9d x;
				x.copyFromPlanar(x0ref);
				x.setTime(tStart);
				return x;
			}
		};
	}
}