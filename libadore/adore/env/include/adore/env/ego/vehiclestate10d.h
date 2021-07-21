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
	namespace env
	{
		/**
         * @brief This struct holds the state of the vehicle in 10d
		 * 
		 * The field data_ comprises the following data
         *   pX,
 		 *   pY,
 		 *   psi,
 		 *   vx,
 		 *   vy,
 		 *   omega,
		 *   ax,
 		 *   delta,
 		 * 	 dax,
 		 *   ddelta
         */
		class VehicleState10d
		{
		public:
			adoreMatrix<double, 10, 1> data; /**< matrix that holds the data */
			/**
			 * @brief Get the x-coordinate
			 * 
			 * @return double x-coordinate
			 */
			double getX()const { return data(0, 0); }
			/**
			 * @brief Get the y-coordinate
			 * 
			 * @return double y-coordinate
			 */
			double getY()const { return data(1, 0); }
			/**
			 * @brief Get the heading
			 * 
			 * @return double heading
			 */
			double getPSI()const { return data(2, 0); }
			/**
			 * @brief Get the longitudinal velocity
			 * 
			 * @return double longitudinal velocity
			 */
			double getvx()const { return data(3, 0); }
			/**
			 * @brief Get the lateral velocity
			 * 
			 * @return double lateral velocity
			 */
			double getvy()const { return data(4, 0); }
			/**
			 * @brief Get the yaw rate
			 * 
			 * @return double yaw rate
			 */
			double getOmega()const { return data(5, 0); }
			/**
			 * @brief Get the longitudinal acceleration
			 * 
			 * @return double longitudinal acceleration
			 */
			double getAx()const { return data(6, 0); }
			/**
			 * @brief Get the steering angle
			 * 
			 * @return double steering angle
			 */
			double getDelta()const { return data(7, 0); }
			/**
			 * @brief Get the derivation of the longitudinal acceleration
			 * 
			 * @return double derivation of the longitudinal acceleration
			 */
			double getDAx()const { return data(8, 0); }
			/**
			 * @brief Get the derivation of the steering angle
			 * 
			 * @return double derivation of the steering angle
			 */
			double getDDelta()const { return data(9, 0); }
			
			/**
			 * @brief Set the x-coordinate
			 * 
			 * @param value x-coordinate
			 */
			void setX(double value) { data(0, 0) = value; }
			/**
			 * @brief Set the y-coordinate
			 * 
			 * @param value y-coordinate
			 */
			void setY(double value) {  data(1, 0) = value; }
			/**
			 * @brief Set the heading
			 * 
			 * @param value heading
			 */
			void setPSI(double value) {  data(2, 0) = value; }
			/**
			 * @brief Set the longitudinal velocity
			 * 
			 * @param value longitudinal velocity
			 */
			void setvx(double value) {  data(3, 0) = value; }
			/**
			 * @brief Set the lateral velocity
			 * 
			 * @param value lateral velocity
			 */
			void setvy(double value) {  data(4, 0) = value; }
			/**
			 * @brief Set the yaw rate
			 * 
			 * @param value yaw rate
			 */
			void setOmega(double value) {  data(5, 0) = value; }
			/**
			 * @brief Set the longitudinal acceleration
			 * 
			 * @param value longitudinal acceleration
			 */
			void setAx(double value) {  data(6, 0) = value; }
			/**
			 * @brief Set the steering angle
			 * 
			 * @param value steering angle
			 */
			void setDelta(double value) {  data(7, 0) = value; }
			/**
			 * @brief Set the derivation of the longitudinal acceleration
			 * 
			 * @param value derivation of the longitudinal acceleration
			 */
			void setDAx(double value) {  data(8, 0) = value; }
			/**
			 * @brief Set the derivation of the steering angle
			 * 
			 * @param value derivation of the steering angle
			 */
			void setDDelta(double value) {  data(9, 0) = value; }
		};
	}
}