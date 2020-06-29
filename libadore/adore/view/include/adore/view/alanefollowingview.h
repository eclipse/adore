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
         * ALaneFollowingView - A model / data abstraction required for following the current lane.
         * Please note that getter functions are not const, as they may internally buffer access in specific implementation
         */
        class ALaneFollowingView:public ALane
        {
            public:
            /**
             *  getHeading - return the heading of the lane at a distance s along the lane
             */
            virtual double getHeading(double s) =0;
            /**
             * getCurvature - return the lane coordinate system's curvature kappa=1/R and its derivatives 1,2,... at a progress s
             * the derivative is given as 1: d kappa / ds, 2: d^2 kappa / ds^2, ...
             * if a derivative is unavailable, 0 will be returned
             */
            virtual double getCurvature(double s, int derivative)  =0;
            /**
             * getOffsetOfLeftBorder - return the lateral offset of the left border at a progress s
             */
            virtual double getOffsetOfLeftBorder(double s)  =0;
            /**
             * getOffsetOfRightBorder - return the lateral offset of the right border at a progress s
             */
            virtual double getOffsetOfRightBorder(double s)  =0;
            /**
             * coordinate transformation from euclidean (xe,ye) to road relative coordinates (s,n)
             */
            virtual void toRelativeCoordinates(double xe,double ye,double& s,double& n)  =0;
            /**
             * coordinate transformation from road relative coordinates (s,n) to euclidean (xe,ye,ze) 
             */
            virtual void toEucledianCoordinates(double s,double n,double& xe,double& ye,double& ze)  =0;
        };
    }
}