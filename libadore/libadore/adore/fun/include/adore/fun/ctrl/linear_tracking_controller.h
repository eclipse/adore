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

#include <adore/mad/adoremath.h>
#include <adore/params/ap_vehicle.h>
#include <adore/params/ap_trajectory_tracking.h>
#include <algorithm>
#include <iostream>
#include <adore/fun/vehiclemotionstate9d.h>
#include <adore/fun/planarvehiclestate10d.h>
#include <adore/fun/motioncommand.h>


namespace adore
{
	namespace fun
	{
		/**
		 * An (almost) linear trajectory tracking controller.
		 * The controller computes error states in longitudinal (ex, ev) and lateral direction (ey, epsi, eomega)
		 * and feeds these back linearly via the control inputs ax (acceleration) and delta (steering angle).
		 * Feedback terms are combined with feedforward control inputs axs and deltas.
		 * Nonlinearity occurs as cos, sin in transformation of position error into coordinates of reference trajectory.
		 */
		class LinearTrackingController 
		{
		private:
			adore::params::APVehicle* m_pVehParameters;
			adore::params::APTrajectoryTracking* m_pCtrlParameters;
			double m_ex;///longitudinal tracking error
			double m_ey;///lateral tracking error
			double m_Ix;///integrator in longitudinal direction
			double m_Iy;///integrator in lateral direction
			bool m_use_integrator;
			bool m_reset_integrator;
		public:
			/**
			 * Constructor.
			 * @param pVehicle vehicle parameters
			 * @param pControl feedback control parameters
			 */
			LinearTrackingController(adore::params::APVehicle* pVehicle,adore::params::APTrajectoryTracking* pControl)
				:m_pVehParameters(pVehicle),m_pCtrlParameters(pControl)
			{
				m_ex=0.0;m_ey=0.0;
				m_Ix=0.0;m_Iy=0.0;
				m_use_integrator=false;
				m_reset_integrator=false;
			}
			///returns the longitudinal tracking error
			double get_ex(){return m_ex;}
			///returns the lateral tracking error
			double get_ey(){return m_ey;}

			void setUseIntegrator(bool value){m_use_integrator=value;}

			void resetIntegrator(bool value){m_reset_integrator=value;}

			/**
			 * Compute the controller.
			 * @param x current state
			 * @param xref reference state, xref=tau(t), including feedforward control 
			 * @param u vehicle control input, output of the function
			 */
			void compute_control_input(const VehicleMotionState9d& x, const PlanarVehicleState10d& xref, MotionCommand& u)
			{
				// read the required parameters
				const double k0x = m_pCtrlParameters->getK0x();
				const double k1x = m_pCtrlParameters->getK1x();
				const double key = m_pCtrlParameters->getKey();
				const double kepsi = m_pCtrlParameters->getKepsi();
				const double keomega = m_pCtrlParameters->getKeomega();
				const double kIx = m_pCtrlParameters->getKIx();
				const double kIy = m_pCtrlParameters->getKIy();

				const double ex_static = m_pCtrlParameters->getExStatic();
				const double ey_static = m_pCtrlParameters->getEyStatic();
				
				const double b = m_pVehParameters->get_b();//rear axle to cog

				// extract variables
				const double psi = x.getPSI();
				const double X = x.getX()+std::cos(psi)*(b-m_pVehParameters->get_observationPointForPosition());				//position of cog
				const double Y = x.getY()+std::sin(psi)*(b-m_pVehParameters->get_observationPointForPosition());				//position of cog
				const double vx = x.getvx();
				const double omega = x.getOmega();

				const double psis = xref.getPSI();
				const double Xs = xref.getX()+std::cos(psis)*b;															//set position of cog
				const double Ys = xref.getY()+std::sin(psis)*b;															//set position of cog
				const double vxs = xref.getvx();
				const double omegas = xref.getOmega();
				const double axs = xref.getAx();
				const double deltas = xref.getDelta();

				const double eomega = omega - omegas;
				const double ev = vx-vxs;
				/* angle transformation to prevent angle modulo errors*/
				const double cpsi = (std::cos)(psi);
				const double spsi = (std::sin)(psi);
				const double cpsis = (std::cos)(psis);
				const double spsis = (std::sin)(psis);
				//transformation (negative rotation) into reference coordinates
				const double tcpsi =  cpsis * cpsi + spsis * spsi;
				const double tspsi = -spsis * cpsi + cpsis * spsi;
				const double epsi = (std::atan2)(tspsi,tcpsi);



				if( vxs<0.0 )
				{
					u.setAcceleration(-2.0);
					u.setSteeringAngle( 0.0 );
				}
				else
				{
					//compute error terms
					m_ex = std::cos(psis)*(X-Xs)+std::sin(psis)*(Y-Ys) + ex_static;
					m_ey =-std::sin(psis)*(X-Xs)+std::cos(psis)*(Y-Ys) + ey_static;
					if(m_use_integrator)
					{
						m_Ix += m_ex*0.01;
						m_Iy += m_ey*0.01;
					}

					if(vx<0.05 || m_reset_integrator)
					{
						m_Ix = 0.0;
						m_Iy = 0.0;
					}

					//ouput resulting control:
					double a_out = axs-kIx*m_Ix-k0x*m_ex-k1x*ev;
					double delta_out = deltas -kIy*m_Iy-key*m_ey -kepsi*epsi -keomega*eomega;
					u.setAcceleration( adore::mad::bound(m_pCtrlParameters->getAxMin(),a_out,m_pCtrlParameters->getAxMax()) );
					u.setSteeringAngle(adore::mad::bound(m_pCtrlParameters->getDeltaMin(),delta_out,m_pCtrlParameters->getDeltaMax()));
				}
			}
		};
	}
}