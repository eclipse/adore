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
		 * @brief abstract class containing parameters to configure the behaviour of the trajactory tracking controller
		 * 
		 */
		class APTrajectoryTracking
		{

		public:
			///lateral control gain for lateral error ey
			virtual double getKey()const =0;

			///lateral control gain for yaw angle error epsi
			virtual double getKepsi()const =0;

			///lateral control gain for yaw rate error eomega
			virtual double getKeomega()const =0;

			///returns I control gain for lateral direction
			virtual double getKIy()const =0;

			///returns I control gain for longitudinal direction
			virtual double getKIx()const =0;

			///returns P control gain for longitudinal direction
			virtual double getK0x()const =0;

			///returns D control gain for longitudinal direction
			virtual double getK1x()const =0;

			///returns factor for maximum tire force requestable by controller, |f_requested|<muCtrlMax * f_max
			virtual double getMuCtrlMax()const =0;

			///hard coded maximum longitudinal acceleration
			virtual double getAxMax()const =0;

			///hard coded minimum longitudinal acceleration
			virtual double getAxMin()const =0;

			///static trajectory tracking offset in longitudinal direction, which should be compensated by tracking controller
			virtual double getExStatic()const =0;

			///static trajectory tracking offset in lateral direction, which should be compensated by tracking controller
			virtual double getEyStatic()const =0;

			///the maximum controllable steering angle
			virtual double getDeltaMax()const=0;

			///the minimum controllable steering angle
			virtual double getDeltaMin()const=0;

			///steering angle: maximum absolute control input change per control update. Maximum steering rate then depends on execution rate of controller.
			virtual double getDDeltaMax()const =0;

			///returns gain for braking torque calculation
			virtual double getBrakingTorqueGain()const =0;

			///returns maxium braking torque rate
			virtual double getDBrakingTorqueMax()const =0;
			
			///reverse controller: control gain for speed error (P)
			virtual double getKPev_r()const =0;

			///reverse controller: control gain for integrated speed error (I)
			virtual double getKIev_r()const =0;

			///reverse controller: control gain for x error (P)
			virtual double getKPex_r()const =0;
			
			///reverse controller: control gain for integrated x error (I)
			virtual double getKIex_r()const =0;

			///reverse controller: control gain for y error (P)
			virtual double getKPey_r()const =0;

			///reverse controller: control gain for psi error (P)
			virtual double getKPepsi_r()const =0;
			
			///reverse controller: control gain for integrated psi error (I)
			virtual double getKIepsi_r()const =0;

			///gain for steering rate limiter
			virtual double getSteeringRateLimiterGain()const =0;
		};
	}
}