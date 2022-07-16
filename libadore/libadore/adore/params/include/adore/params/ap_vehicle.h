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

#include <string>

namespace adore
{
	namespace params
	{

		/**
		 * @brief abstract class for vehicle configuration related paremeters
		 * 
		 */
		class APVehicle
		{

		public:
			///ID of current vehicle
			virtual std::string get_vehicle_id()const =0;

		  	///cog to front axle
			virtual double get_a()const =0;

			///rear axle to cog
			virtual double get_b()const =0;

			///front axle to front border
			virtual double get_c()const =0;

			///rear border to rear axle
			virtual double get_d()const =0;

			///mass
			virtual double get_m()const =0;

			///friction coefficient
			virtual double get_mu()const =0;

			///gravitational constant
			virtual double get_g()const =0;

			///cog height above ground
			virtual double get_h()const =0;

			///front normalized tire stiffness for bicycle model
			virtual double get_cf()const =0;

			///rear normalized tire stiffness for bicycle model
			virtual double get_cr()const =0;

			///rotational inertia around up axis devided by mass
			virtual double get_Iz_m()const =0;

			///track width front
			virtual double get_wf()const =0;

			///track width rear
			virtual double get_wr()const =0;

			virtual double get_bodyWidth()const =0;
			virtual double get_steeringRatio()const =0;
			virtual double get_steeringAngleOffsetMeasured()const =0;
			virtual double get_steeringAngleOffsetCommand()const =0;
			virtual double get_steeringAngleMax()const=0;
			virtual double get_steeringAngleMin()const=0;

			///unnormalized cornering stiffness
			virtual double get_C()const =0;

			///returns the percentage of brake force allocated to the front axle, e.g. 0.6 is a typical value
			virtual double get_brakeBalanceFront()const =0;

			///returns the percentage of acceleration force allocated to the front axle, e.g. 1.0 for front drive
			virtual double get_accelerationBalanceFront()const =0;

			virtual double get_observationPointForPosition()const =0;
			virtual double get_observationPointForVelocity()const =0;
			virtual double get_observationPointForAcceleration()const =0;

			///returns the flag of currently used vehicle
			virtual double get_vehicleFlag()const =0;
		};
	}
}