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

namespace adore
{
	namespace fun
	{
		/**
		 * Request the controller to attempt a stop of the vehicle at the specified postion with the specified heading.
		 * If the position can not be hit due to acceleration constraints, maximum deceleration shall be applied, while regulating the lateral error towards the line specified by (X,Y,psi).
		 */
		class TerminalRequest
		{
		private:
			double X,Y,psi,t;
			bool valid;
		public:
			void set(double X,double Y,double psi,double t,bool valid)
			{
				this->X=X;
				this->Y=Y;
				this->psi=psi;
				this->t=t;
				this->valid=valid;
			}
			double getX()const{return X;}
			double getY()const{return Y;}
			double getPSI()const{return psi;}
			double getT()const{return t;}
			bool isValid()const{return valid;}
			bool isActive(double t)const{return valid && t>this->t;}
			adoreMatrix<double,3,1> getStateVector()const
			{
				adoreMatrix<double,3,1> m;
				m(0)=X;
				m(1)=Y;
				m(2)=psi;
				return m;
			}
			void relocate(double new_X0,double new_Y0, double new_PSI0)
			{
				X = new_X0;
				Y = new_Y0;
				psi = new_PSI0;
			}
			void setStartTime(double new_t0)
			{
				t = new_t0;
			}
			TerminalRequest():X(0.0),Y(0.0),psi(0.0),t(0.0),valid(false){}
			TerminalRequest(double X,double Y,double psi,double t,bool valid):X(X),Y(Y),psi(psi),t(t),valid(valid){}
		};
	}
}