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
#include <adore/fun/vehiclemotionstate9d.h>

namespace adore
{
    namespace fun
    {

        /**
		 * A vehicle state used as reference for the vehicle, containing feedforward control inputs.
         * [
         *   pX,
         *   pY,
         *   psi,
         *   vx,
         *   vy,	at COR
         *   omega,
         *   ax,
         *   delta,
		 *   dax,
		 *   ddelta
         * ]
         */
		struct PlanarVehicleState10d
		{
		public:
			PlanarVehicleState10d(){}
			PlanarVehicleState10d(const PlanarVehicleState10d& other)
			{
				this->data = other.data;
			}
			PlanarVehicleState10d(const VehicleMotionState9d& other)
			{	
				this->setX(other.getX());
				this->setY(other.getY());
				this->setPSI(other.getPSI());
				this->setvx(other.getvx());
				this->setvy(other.getvy());
				this->setOmega(other.getOmega());
				this->setAx(other.getAx());
				this->setDelta(other.getDelta());
				this->setDAx(0.0);
				this->setDDelta(0.0);
			}
			adoreMatrix<double, 10, 1> data;
			double getX()const { return data(0, 0); }
			double getY()const { return data(1, 0); }
			double getPSI()const { return data(2, 0); }
			double getvx()const { return data(3, 0); }
			double getvy()const { return data(4, 0); }
			double getOmega()const { return data(5, 0); }
			double getAx()const { return data(6, 0); }
			double getDelta()const { return data(7, 0); }
			double getDAx()const { return data(8, 0); }
			double getDDelta()const { return data(9, 0); }

			void setX(double value) { data(0, 0) = value; }
			void setY(double value) {  data(1, 0) = value; }
			void setPSI(double value) {  data(2, 0) = value; }
			void setvx(double value) {  data(3, 0) = value; }
			void setvy(double value) {  data(4, 0) = value; }
			void setOmega(double value) {  data(5, 0) = value; }
			void setAx(double value) {  data(6, 0) = value; }
			void setDelta(double value) {  data(7, 0) = value; }
			void setDAx(double value) {  data(8, 0) = value; }
			void setDDelta(double value) {  data(9, 0) = value; }
		};
    }
}