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
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/params/ap_local_road_map.h>

namespace adore
{
  namespace params
  {
    /**
     * @brief a dummy implementation for testing purposes
     * 
     */
    class APLocalRoadMapDummy:public APLocalRoadMap
    {
      public:
      virtual double getDiscardRadius()const override
      {
          return 200.0;
      }
      virtual bool isNavigationActive()const override
      {
          return false;
      }
      virtual double getBorderTraceLength()const override
      {
          return 50.0;
      }
    };
  }
}