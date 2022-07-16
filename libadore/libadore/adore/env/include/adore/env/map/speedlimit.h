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

namespace adore
{
    namespace env
    {
        /**
         * @brief speed limit information valid from startx/y to stopx/y - should match with respective lane
         * 
         */

        typedef int TSpeedLimitID;

        struct SpeedLimit
        {
            SpeedLimit()
            {
                value  = 0.0;
                startX = 0.0;
                startY = 0.0;
                stopX  = 0.0;
                stopY  = 0.0;
            }   
            double value;
            double startX;
            double startY;
            double stopX;
            double stopY;
            TSpeedLimitID id;
        };


        typedef std::vector<SpeedLimit> TSpeedLimitBundle;
    }
}