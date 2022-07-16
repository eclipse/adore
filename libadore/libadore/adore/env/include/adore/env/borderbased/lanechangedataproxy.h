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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/mad/linearfunctiontypedefs.h>
#include <adore/view/alanechangeview.h>

namespace adore
{
    namespace env
    {
        namespace BorderBased
        {
            /**
             * @brief simple struct to hole data relevant to lane change geometry
             *
             * @see lanechangegeometry.h
             * 
             */
            struct LaneChangeDataProxy
            {
                double gate_s0;
                double gate_s1;
                adore::view::ALaneChangeView::direction direction;
                bool isValid;
                adore::mad::function_type_scalar targetOuterBorderDistance_fct;
                adore::mad::function_type_scalar separatingBorderDistance_fct;
                adore::mad::function_type_scalar sourceOuterBorderDistance_fct;
                double viewingDistance;

                LaneChangeDataProxy()
                  : gate_s0(0.0)
                  , gate_s1(0.0)
                  , isValid(false)
                  , viewingDistance(0.0)
                {
                }
            };
        }  // namespace BorderBased
    }      // namespace env
}  // namespace adore