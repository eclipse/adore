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
 *   Matthias Nichting
 ********************************************************************************/

#pragma once
#include <string>

namespace adore
{
    namespace PLOT
    {
        class ViewPlotterConfig
        {
          public:
            std::string lf_geometry_plotoptions;
            std::string lc_geometry_plotoptions;
            std::string lc_gate_plotoptions;
            std::string lc_sufficient_width_plotoptions;
            double lc_geometry_narrowing;               // narrow the geometry of the lc to avoid overlap with plot of lf geometry
            double lf_geometry_narrowing;               // narrow the geometry of the lf to avoid overlap with plot of lc geometry
            int number_of_samples_per_boundary;
            bool plot_lf_geometry;
            bool plot_lc_geometry;

            ViewPlotterConfig()
            {
            }
            ~ViewPlotterConfig()
            {
            }
        };
    }  // namespace PLOT
}  // namespace adore