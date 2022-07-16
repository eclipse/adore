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
#include "trafficobject.h"
#include "conflictset.h"

namespace adore
{
    namespace view
    {
        /**
         *  ALane - represents traffic and traffic rules for a lane. 
         *  ALane uses a coordinate system (s,n), with s progress along the lane and n lateral offset, perpendicular to s.
         */
        class ALane
        {
          public:
            /**
             * isValid - return true if representation of lane is valid
             */
            virtual bool isValid()const =0;
            /**
             * @return the maximum s parameter of the relative coordinate system
             */
            virtual double getSMax()const=0;
            /**
             * @return the minimum s parameter of the relative coordinate system
             */
            virtual double getSMin()const=0;
            /**
             * @return true if getSMin()<s && s<getSMax()
             */
            bool inSRange(double s)const
            {
                return getSMin()<s && s<getSMax();
            }
            /**
             * getProgressOfWidthOpen - return progress s, where target lane is wide enough to contain AV for the first time
             */
            virtual double getProgressOfWidthOpen()const =0;
            /**
             * getProgressOfWidthClosed - return progress s, where target lane is no longer wide enough to contain AV
             */
            virtual double getProgressOfWidthClosed()const =0;
            /**
             * getOnLaneTraffic - return queue of traffic objects moving on lane, ordered by progress of objects on lane
             */
            virtual const TrafficQueue& getOnLaneTraffic()const =0;
            /**
             * getConflictSet - return set of conflict zones, ordered by occurance along lane
             */
            virtual const ConflictSet& getConflictSet()const =0;
            /**
             * getSpeedLimit - return the speed limit at a certain distance s along the lane
             */
            virtual double getSpeedLimit(double s) =0;
            /**
             * getLeftIndicatorHint - return left indicator light hint at s along the lane
             */
            virtual double getLeftIndicatorHint(double s) =0;
            /**
             * getRightIndicatorHint - return right indicator light hint at s along the lane
             */
            virtual double getRightIndicatorHint(double s) =0;
            /**
             * hasSpeedRecommendation - return true, if a speed recommendation is available (GLOSA or other infrastructure advice) at a certain distance s along the lane
             */
            virtual bool hasSpeedRecommendation(double s)const =0;
            /**
             * getSpeedRecommendation - return a speed recommendation at a certain distance s along the lane
             */
            virtual double getSpeedRecommendation(double s)const =0;
            /**
             * getNavigationCost - return remaining navigation cost at a certain distance s along the lane
             */
            virtual double getNavigationCost(double s) =0;
            /**
             * boundNavigationCost - return bounds for navigation-cost on a distance s interval along the lane
             * @param cmin the minimum cost on the interval is returned
             * @param cmax the maximum cost on the interval is returned
             */
            virtual void boundNavigationCost(double s0,double s1,double& cmin,double& cmax) =0;
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