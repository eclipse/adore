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
         * @brief This struct holds the motion state of the vehicle in 9d
		 * 
		 * The field data_ comprises the following data
         *   pX,
         *   pY,
		 *   pZ,
         *   psi,
         *   vx,
         *   vy,
         *   omega,
         *   ax,
         *   delta
         */
        struct VehicleMotionState9d
        {
		public:
			double time_ = 0; /**< time */
			adoreMatrix<double, 9, 1> data_ = dlib::zeros_matrix<double>(9,1); /**< motion state */
			/**
			 * @brief Get the time
			 * 
			 * @return double time
			 */
			double getTime()const {return time_;}
			/**
			 * @brief Get the x-coordinate
			 * 
			 * @return double x-coordinate
			 */
			double getX()const { return data_(0, 0); }
			/**
			 * @brief Get the y-coordinate
			 * 
			 * @return double y-coordinate
			 */
			double getY()const { return data_(1, 0); }
			/**
			 * @brief Get the z-coordinate
			 * 
			 * @return double z-coordinate
			 */
			double getZ()const { return data_(2, 0); }
			/**
			 * @brief Get the heading
			 * 
			 * @return double heading
			 */
			double getPSI()const { return data_(3, 0); }
			/**
			 * @brief Get the longitudinal velocity
			 * 
			 * @return double longitudinal velocity
			 */
			double getvx()const { return data_(4, 0); }
			/**
			 * @brief Get the lateral velocity
			 * 
			 * @return double lateral velocity
			 */
			double getvy()const { return data_(5, 0); }
			/**
			 * @brief Get the yaw rate
			 * 
			 * @return double yaw rate
			 */
			double getOmega()const { return data_(6, 0); }
			/**
			 * @brief Get the longitudinal acceleration
			 * 
			 * @return double longitudinal acceleration
			 */
			double getAx()const { return data_(7, 0); }
			/**
			 * @brief Get the steering angle
			 * 
			 * @return double steering angle
			 */
			double getDelta()const { return data_(8, 0); }
			
			/**
			 * @brief Set the time
			 * 
			 * @param value time
			 */
			void setTime(double value){time_=value;}
			/**
			 * @brief Set the x-coordinate
			 * 
			 * @param value x-coordinate
			 */
			void setX(double value) { data_(0, 0) = value; }
			/**
			 * @brief Set the y-coordinate
			 * 
			 * @param value y-coordinate
			 */
			void setY(double value) {  data_(1, 0) = value; }
			/**
			 * @brief Set the z-coordinate
			 * 
			 * @param value z-coordinate
			 */
			void setZ(double value) {  data_(2, 0) = value; }
			/**
			 * @brief set the heading
			 * 
			 * @param value heading
			 */
			void setPSI(double value) {  data_(3, 0) = value; }
			/**
			 * @brief set the longitudinal velocity
			 * 
			 * @param value longitudinal velocity
			 */
			void setvx(double value) {  data_(4, 0) = value; }
			/**
			 * @brief set the lateral velocity
			 * 
			 * @param value lateral velocity
			 */
			void setvy(double value) {  data_(5, 0) = value; }
			/**
			 * @brief Set the yaw rate
			 * 
			 * @param value yaw rate
			 */
			void setOmega(double value) {  data_(6, 0) = value; }
			/**
			 * @brief Set the longitudinal acceleration
			 * 
			 * @param value longitudinal acceleration
			 */
			void setAx(double value) {  data_(7, 0) = value; }
			/**
			 * @brief Set the steering angle
			 * 
			 * @param value steering angle
			 */
			void setDelta(double value) {  data_(8, 0) = value; }
		};
    }
}