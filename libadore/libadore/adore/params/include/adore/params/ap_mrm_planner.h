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
  namespace params
  {
    /**
     * @brief abstract class containing parameters related to configuring the lateral planner
     * 
     */
    class APMRMPlanner
    {
      public:
      ///getAccLB returns longitudinal acceleration lower bound
      virtual double getAccLB() const=0;

      ///getJerkLB returns longitudinal jerk lower bound
      virtual double getJerkLB() const=0;

      ///getAccStall returns initial deceleration
      virtual double getAccStall() const=0;

      ///length of first phase of maneuver, during which initial deceleration is applied
      virtual double getTStall() const=0;

    };
  }
}