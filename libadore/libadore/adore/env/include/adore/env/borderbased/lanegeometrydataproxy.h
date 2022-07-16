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

namespace adore
{
    namespace env
    {
        namespace BorderBased
        {
            struct LaneGeometryDataProxy
            {
                adore::mad::function_type_xyz centerSmoothed_fct;
                adore::mad::function_type_scalar leftDistance_fct;
                adore::mad::function_type_scalar rightDistance_fct;
                adore::mad::function_type2d centerNormal_fct;
                adore::mad::function_type_scalar centerSmoothedCurvature_fct;
                adore::mad::function_type_scalar centerSmoothedCurvatureDerivative_fct;
                adore::mad::function_type_scalar navigationCost_fct;
                adore::mad::function_type_scalar speedLimit_fct;
                adore::mad::function_type_scalar left_indicator_hint_fct;
                adore::mad::function_type_scalar right_indicator_hint_fct;

                bool isValid;

                LaneGeometryDataProxy() : isValid(false)
                {
                }
            };
        }  // namespace BorderBased
    }      // namespace env
}  // namespace adore