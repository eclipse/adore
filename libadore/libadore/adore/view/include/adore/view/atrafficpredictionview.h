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
#include <adore/mad/occupancycylinder.h>

namespace adore
{
    namespace view
    {
        /**
         * ATrafficPredictionView provides different categories of predictions 
         * and allows to test whether a set the ego vehicle wants to occupy conflicts with any of these predictions.
         */
        class ATrafficPredictionView
        {
            public:
            /**
             * overlapsExpectedBehavior returns true if given space overlaps with most likely current or future object positions
             */
            virtual bool overlapsExpectedBehavior(const adore::mad::OccupancyCylinderTree& space)const =0;
            /**
             * overlapsEmergencyBehavior returns true if given space overlaps with current or future object positions belongig to the emergency behavior of these objects
             */
            virtual bool overlapsEmergencyBehavior(const adore::mad::OccupancyCylinderTree& space)const =0;

            /**
             * getExpectedCollisionTime returns minimum time of all occuring collisions or guard if no collisions occur
             */
            virtual double getExpectedCollisionTime(const adore::mad::OccupancyCylinderTree& space,double guard)const =0;

            //@TODO: method for desired behavior: test for all intersections and list trackingids and branchids 


        };
    }
}