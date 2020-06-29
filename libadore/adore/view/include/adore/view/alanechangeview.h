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
         */
        class ALaneChangeView:public ALane
        {
            public:
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
             * getProgressOfWidthOpen - return progress s, where target lane is wide enough to contain AV for the first time
             */
            virtual double getProgressOfWidthOpen()const =0;
            /**
             * getProgressOfWidthClosed - return progress s, where target lane is no longer wide enough to contain AV
             */
            virtual double getProgressOfWidthClosed()const =0;
            /**
             * getOffsetOfStartOuterBorder - return lateral offset n of the outer border of the AV's current lane
             */
            virtual double getOffsetOfStartOuterBorder(double s)const =0;
            /**
             * getOffsetOfSeparatingBorder - return lateral offset n of the separating border between start and target lane
             */
            virtual double getOffsetOfSeparatingBorder(double s) =0;
            /**
             * getOffsetOfDestinationOuterBorder - return lateral offset n of the outer border of target lane
             */
            virtual double getOffsetOfDestinationOuterBorder(double s) =0;
        };
    }
}