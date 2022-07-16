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

#include <vector>
#include <string>
#include <map>

namespace adore
{
    namespace env
    {
        /**
         * @brief indicator lights hints valid from startx/y to stopx/y - to be matched with corresponding borders
         *
         */

        typedef int TIndicatorHintID;

        enum IndicatorSide
        {
            none,
            left,
            right,
            both
        };

        struct IndicatorHint
        {
          public:
            IndicatorHint()
            {
                value = IndicatorSide::none;
                startX = 0.0;
                startY = 0.0;
                stopX = 0.0;
                stopY = 0.0;
            }

            constexpr static adore::env::IndicatorSide indicatorSideFromString(std::string_view key)
            {
                if (key.compare("left") == 0)
                {
                    return IndicatorSide::left;
                }
                else if (key.compare("right") == 0)
                {
                    return IndicatorSide::right;
                }
                else if (key.compare("both") == 0)
                {
                    return IndicatorSide::both;
                }
                else // either "none" or unknown
                {
                    return IndicatorSide::none;
                }
            }

            constexpr static std::string_view stringFromIndicatorSide(adore::env::IndicatorSide side)
            {
                switch (side)
                {
                    case IndicatorSide::none:
                        return "none";
                        break;
                    case IndicatorSide::left:
                        return "left";
                        break;
                    case IndicatorSide::right:
                        return "right";
                        break;
                    case IndicatorSide::both:
                        return "both";
                        break;
                    default:
                        return "none";
                }
            }

            IndicatorSide value;
            double startX;
            double startY;
            double stopX;
            double stopY;
            TIndicatorHintID id;
        };

        typedef std::vector<IndicatorHint> TIndicatorHintList;
    }  // namespace env
}  // namespace adore