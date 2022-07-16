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
 *    Thomas Lobig - initial implementation and API
 ********************************************************************************/

#pragma once

#include <adore/env/traffic/trafficmap.h>
// #include <unordered_map>
// #include <adore/env/borderbased/borderset.h>
// #include <adore/env/borderbased/lanematchingstrategy.h>
// #include <adore/env/afactory.h>
// #include "participant.h"

namespace adore
{
    namespace env
    {
        namespace traffic
        {
            /**
             * Class to represent traffic data
             *
             */
            class TrafficMapDummy : public TrafficMap
            {
              public:
                TrafficMapDummy() : TrafficMap(nullptr,nullptr)
                {
                }

                void update()
                {
                    std::cout << " traffic dummy talking - update" << std::endl;
                }

                void matchBorders()
                {
                    std::cout << " traffic dummy talking - matchBorders" << std::endl;
                }

            };
        }  // namespace traffic
    }      // namespace env
}  // namespace adore