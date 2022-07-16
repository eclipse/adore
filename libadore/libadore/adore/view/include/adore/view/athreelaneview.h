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
#include "alane.h"
#include "alanechangeview.h"

namespace adore
{
    namespace view
    {
        /**
         * AThreeLaneView - A model / data abstraction useful for maneuver planning.
         * For many autonomous driving applications it is sufficient to know the vehicle's current and the two adjacent lanes.
         * AThreeLaneView is an interface, which promises to provide ALane interfaces for current, left and right lane,
         * as well as ALaneChangeView interfaces for transitioning to left and right.
         */
        class AThreeLaneView
        {
            public:
            /**
             * getCurrentLane - return ALane pointer the vehicle's current lane
             */
            virtual ALane* getCurrentLane()=0;
            /**
             * getLeftLaneChange - returns ALaneChangeView pointer to the lane left of the vehicle's current lane.
             */
            virtual ALaneChangeView* getLeftLaneChange()=0;
            /**
             * getRightLaneChange - returns ALaneChangeView pointer to the lane right of the vehicle's current lane.
             */
            virtual ALaneChangeView* getRightLaneChange()=0;
        };
    }
}