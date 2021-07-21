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
 *   Daniel Heß - definition of view
 ********************************************************************************/


#pragma once
namespace adore
{
namespace view
{
    /**
     * Information about the navigation goal
     */
    class ANavigationGoalView
    {
        public:
        /**
         * isNextGoalPointFinal
         * - true, if the vehicle has to stop
         * - false, if the vehicle may continue driving in expectation of the next waypoint
         */
        virtual const bool isNextGoalPointFinal()const=0;
        /**
         * isNextGoalPointInView determines whether the relation of goal-point to lane can be determined
         */
        virtual const bool isNextGoalPointInView()const=0;
        /**
         * is true, if the goal point is on the current lane
         */
        virtual const bool isNextGoalPointOnCurrentLane()const=0;
        /**
         * isNextGoalPointOnLaneToTheLeft returns true, if the goal point is on a neighboring lane to the left
         */
        virtual const bool isNextGoalPointOnLaneToTheLeft()const=0;
        /**
         * isNextGoalPointOnLaneToTheRight returns true, if the goal point is on a neighboring lane to the right
         */
        virtual const bool isNextGoalPointOnLaneToTheRight()const=0;
        /**
         * returns the s-coordinate of the goal point in the current road coordinate system
         */        
        virtual const double getProgress()const=0;
    };
}//end view
}//end adore