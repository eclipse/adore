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
 *   Thomas Lobig - initial implementation
 ********************************************************************************/

#pragma once
#include <string>

namespace adore
{
  namespace PLOT
  {

    class LanePlotConfig
    {
      public:
      std::string border_outer_left_plotoptions;
      std::string border_outer_right_plotoptions;
      std::string lane_fill_driveable_plotoptions;
      std::string lane_fill_emergency_plotoptions;
      std::string lane_fill_other_plotoptions;
      std::string lane_fill_default_plotoptions;
      std::string setpoint_plotoptions;
      bool plot_emergency_lane = false;
      bool plot_other_lane = false;
      bool plot_drive_lane = true;
      bool plot_traffic_lights = false;

      LanePlotConfig()
      { 
      }
    };
  }
}