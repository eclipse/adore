/********************************************************************************
 * Copyright (C) 2017-2021 German Aerospace Center (DLR). 
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

namespace adore
{
	namespace params
	{
		/**
		 * @brief abstract class containing parameters which configure odometry state estimation model
		 * 
		 */
		class APOdometryModel
		{
		public:
			virtual double get_k_e_vx()const = 0;
			virtual double get_k_e_vy()const = 0;
			virtual double get_k_e_omega()const = 0;
			virtual double get_k_e_ax()const = 0;
		};
	}
}