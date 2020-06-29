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
#include <adore/fun/terminalrequest.h>
#include <adore/params/ap_emergency_operation.h>
#include <adore/params/ap_vehicle.h>
#include <adore/mad/adoremath.h>
#include <adore/fun/motioncommand.h>
#include <adore/fun/vehiclemotionstate9d.h>

namespace adore
{
	namespace fun
	{
		/**
		 * A simple controller for emergency braking and lateral stabilization during emergency braking.
		 * Main goal of the controller is to stop the vehicle as soon as possible, therefore a high deceleration value is applied.
		 * The controller is provided with a reference in the form of a straight line, around which to stabilize the vehicle during braking.
		 */
		class TerminalController
		{
		private:
			adore::params::APEmergencyOperation* ap_em_;
			adore::params::APVehicle* ap_v_;
		public:
			/**
			 * Constructor
			 * @param ap_em emergency operation parameters
			 * @param ap_v vehicle parameters
			 */
			TerminalController(adore::params::APEmergencyOperation* ap_em,adore::params::APVehicle* ap_v):ap_em_(ap_em),ap_v_(ap_v){}
			/**
			 * computation of controller
			 * @param x vehicle state
			 * @param term terminal request: straight line around which to stabilize during braking
			 * @param u vehicle control input, output of the function
			 */
			void compute_control_input(const VehicleMotionState9d& x, const TerminalRequest& term, MotionCommand& u)
			{
				double cpsi = std::cos(term.getPSI());
				double spsi = std::sin(term.getPSI());
				double dx = cpsi*(x.getX()-term.getX())+spsi*(x.getY()-term.getY());
				double dy =-spsi*(x.getX()-term.getX())+cpsi*(x.getY()-term.getY());
				double cpsiego = std::cos(x.getPSI());
				double spsiego = std::sin(x.getPSI());
				double dpsix = cpsi*cpsiego+spsi*spsiego;
				double dpsiy =-spsi*cpsiego+cpsi*spsiego;
				double dpsi = std::atan2(dpsiy,dpsix);
				if(dx<0.0)
				{
					double a = 0.5*x.getvx()*x.getvx()/dx;
					u.setAcceleration(std::max(a,ap_em_->getamin()));
				}
				else
				{
					u.setAcceleration(ap_em_->getamin());
				}
				if(x.getvx()>0.1)
				{
					//@TODO: use a point different from the COR
					u.setSteeringAngle(adore::mad::bound(ap_em_->getDeltaMin(),-ap_em_->getKy()*dy - ap_em_->getKpsi()*dpsi,ap_em_->getDeltaMax()));
				}
				else
				{
					u.setSteeringAngle(0.0);
				}
			}
		};
	}
}