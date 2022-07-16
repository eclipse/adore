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
  namespace sim
  {
    /**
     * @brief provides encapsulation of values needed to reset the vehicle pose in a simulation
     * 
     */
    struct ResetVehiclePose
    {

      private:
      double X;
      double Y;
      double Z;
      double PSI;

      public:
      double getX() const {
      	return this->X;
      }
      void setX(double X) {
      	this->X = X;
      }


      double getY() const {
        return this->Y;
      }
      void setY(double Y) {
        this->Y = Y;
      }


      double getPSI() const{
        return this->PSI;
      }
      void setPSI(double PSI) {
        this->PSI = PSI;
      }

      double getZ() const{
        return this->Z;
      }
      void setZ(double Z) {
        this->Z = Z;
      }
      
    };
  }
}