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
#include <adore/env/borderbased/lanegeometrydataproxy.h>
#include <adore/env/borderbased/lanechangedataproxy.h>
#include <adore/mad/linearfunctiontypedefs.h>

namespace adore
{
    namespace env
    {
        namespace BorderBased
        {
            struct CombinedLaneGeometry
            {
                // private:
                std::shared_ptr<LaneGeometryDataProxy> center;
                std::shared_ptr<LaneGeometryDataProxy> left; // TODO left and right should actually not be needed
                                                            // since lanechange data has the information delta to build
                                                            // left and right from it using only data from center
                std::shared_ptr<LaneChangeDataProxy> leftChange;
                std::shared_ptr<LaneGeometryDataProxy> right;
                std::shared_ptr<LaneChangeDataProxy> rightChange;

              public:
                CombinedLaneGeometry()
                  : center(std::make_shared<LaneGeometryDataProxy>())
                  , left(std::make_shared<LaneGeometryDataProxy>())
                  , leftChange(std::make_shared<LaneChangeDataProxy>())
                  , right(std::make_shared<LaneGeometryDataProxy>())
                  , rightChange(std::make_shared<LaneChangeDataProxy>())
                {
                }
                
            };
        }  // namespace BorderBased
    }      // namespace env
}  // namespace adore