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

namespace adore
{
	namespace fun
	{
	   /**
	   *  Control input for vehicle motion.
	   *  Assuming reference acceleration and steering angle are realized by lower level controllers.
	   */
		class MotionCommand
		{
		private:
			double steeringAngle_;
			double acceleration_;
		public:

			MotionCommand()
			{
				steeringAngle_ = 0.0;
				acceleration_ = 0.0;
			}
			double getSteeringAngle() const
			{
				return this->steeringAngle_;
			}
			void setSteeringAngle(double steeringAngle) 
			{
				this->steeringAngle_ = steeringAngle;
			}


			double getAcceleration() const
			{
				return this->acceleration_;
			}
			void setAcceleration(double acceleration) 
			{
				this->acceleration_ = acceleration;
			}


		};
	}
}