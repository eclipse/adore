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
     * @brief provides encapsulation of values needed to reset the vehicle twist (vx,vy and omega) in a simulation
     * 
     */
    struct ResetVehicleTwist
    {
      private:
      double vx;
      double omega;
      double vy;
      public:
      double getVx() const{
        return this->vx;
      }
      void setVx(double vx) {
        this->vx = vx;
      }



      double getVy() const {
        return this->vy;
      }
      void setVy(double vy) {
        this->vy = vy;
      }



      double getOmega() const {
        return this->omega;
      }
      void setOmega(double omega) {
        this->omega = omega;
      }


    };
  }
}