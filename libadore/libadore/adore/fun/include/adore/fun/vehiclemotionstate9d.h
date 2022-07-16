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
#include <type_traits>

namespace adore
{
    namespace fun
    {
		struct PlanarVehicleState10d;
		class SetPoint;
        /**
		Structure which contains vehicle motion states and time, used as observed state, input to planning and control modules.
         * [
         *   pX,
         *   pY,
		 *   pZ,
         *   psi,
         *   vx,
         *   vy at COR,
         *   omega,
         *   ax,
         *   delta,
         * ]
         */	
		class VehicleMotionState9d
        {
		public:
			VehicleMotionState9d()
			{
				time_ = 0.0;
				for(int i=0;i<9;i++)data_(i)=0.0;
			}

			double time_;
			adoreMatrix<double, 9, 1> data_;
			/**
			* returns time
			*/
			double getTime()const {return time_;}
			 /**
			  * returns X
			  */
			double getX()const { return data_(0, 0); }
			  /**
			   * returns Y
			   */
			double getY()const { return data_(1, 0); }
			  /**
			   * returns Z
			   */
			double getZ()const { return data_(2, 0); }
			  /**
			   * returns PSI (heading)
			   */
			double getPSI()const { return data_(3, 0); }
			  /**
			   * returns velocity in x coordinate
			   */
			double getvx()const { return data_(4, 0); }
			  /**
			   * returns velocity in y coordinate
			   */
			double getvy()const { return data_(5, 0); }
			  /**
			   * returns omega (yaw rate)
			   */
			double getOmega()const { return data_(6, 0); }
			  /**
			   * returns acceleration in x coordinate
			   */
			double getAx()const { return data_(7, 0); }
			  /**
			   * returns steering angle
			   */
			double getDelta()const { return data_(8, 0); }
			  /**
			   * to set the time
			   *@param value is time
			   */
			void setTime(double value){time_=value;}
			  /**
			   * to set the X
			   *@param value is X
			   */
			void setX(double value) { data_(0, 0) = value; }
			  /**
			   * to set the Y
			   *@param value is Y
			   */
			void setY(double value) {  data_(1, 0) = value; }
			  /**
			   * to set the Z
			   *@param value is Z
			   */
			void setZ(double value) {  data_(2, 0) = value; }
			  /**
			   * to set the PSI (heading)
			   *@param value is PSI
			   */
			void setPSI(double value) {  data_(3, 0) = value; }
			  /**
			   * to set the vx (velocity in x coordinate)
			   *@param value is vx
			   */
			void setvx(double value) {  data_(4, 0) = value; }
			  /**
			   * to set the vy (velocity in y coordinate)
			   *@param value is vy
			   */
			void setvy(double value) {  data_(5, 0) = value; }
			  /**
			   * to set the Omega (yaw rate)
			   *@param value is Omega
			   */
			void setOmega(double value) {  data_(6, 0) = value; }
			  /**
			   * to set the Ax (acceleration in x coordinate)
			   *@param value is Ax
			   */
			void setAx(double value) {  data_(7, 0) = value; }
			  /**
			   * to set the Delta (steering angle)
			   *@param value is Delta
			   */
			void setDelta(double value) {  data_(8, 0) = value; }

			/**
			 * @brief Offers the possibility to copy relevant fields from a PlanarVehicleState10d to VehicleMotionState9d
			 * 
			 * this construct is used to circumvent issues that arise from the mainly header-only nature of the framework
			 * while a copy constructor for the reverse copy order already existed, this new function was not possible to
			 * implement in the same way due to issues with cyclic header dependencies
			 * 
			 * @tparam T 
			 * @tparam std::enable_if_t< std::is_same<T,PlanarVehicleState10d>::value> 
			 * @param other a PlanarVehicleState10d object
			 */
			template<class T, 
				typename = std::enable_if_t< std::is_same<T,PlanarVehicleState10d>::value> >
			void copyFromPlanar(const T& other)
			{		
				this->setX(other.getX());
				this->setY(other.getY());
				// this->setZ(0.0);
				this->setPSI(other.getPSI());
				this->setvx(other.getvx());
				this->setvy(other.getvy());
				this->setOmega(other.getOmega());
				this->setAx(other.getAx());
				this->setDelta(other.getDelta());
			}
		};
    }
}