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

namespace adore
{
    namespace view
    {
        /**
         * ALaneChangeView - A model / data abstraction required for changing to a target lane.
         * ALaneChangeView couples two ALanes, between which a lane change is possible at a certain longitudinal interval (gate).
         */
        class ALaneChangeView
        {
            public:
            /**
             * getSourceLane - return ALane pointer for the source lane of the lane change
             */
            virtual ALane* getSourceLane()=0; 
            /**
             * getSourceLane - return ALane pointer for the target lane of the lane change
             */
            virtual ALane* getTargetLane()=0; 

            /**
             * lane change direction enum
             */
            enum direction
            {
                LEFT,RIGHT
            };
            /**
             * getLCDirection - return the direction of a lane change leading to target lane
             */
            virtual direction getLCDirection()const =0;
            /**
             * getProgressOfGateOpen - return progress s of the next opening of a gate (distance to end of solid line or otherwise impassable lane border)
             */
            virtual double getProgressOfGateOpen()const =0;
            /**
             * getProgressOfGateClosed - return progress s of the closure of the next gate (distance to beginngin of solid line or otherwise impassable lane border after gate)
             */
            virtual double getProgressOfGateClosed()const =0;
            /**
             * getOffsetOfStartOuterBorder - return lateral offset n of the outer border of the AV's current lane
             */
            virtual double getOffsetOfStartOuterBorder(double s) =0;
            /**
             * getOffsetOfSeparatingBorder - return lateral offset n of the separating border between start and target lane
             */
            virtual double getOffsetOfSeparatingBorder(double s) =0;
            /**
             * getOffsetOfDestinationOuterBorder - return lateral offset n of the outer border of target lane
             */
            virtual double getOffsetOfDestinationOuterBorder(double s) =0;
            /**
             * @return the difference in navigation cost between two lanes: g_target-g_source; if the return value is below 0, the target lane has lower navigation cost 
             */
            virtual double getNavigationCostDifference()=0;
        };
    }
}