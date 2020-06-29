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
         *  ALane - represents traffic and traffic rules for a lane. Geometry is specific to subtype of ALane.
         *  ALane and subclasses use a coordinate system (s,n), with s progress along the lane and n lateral offset, perpendicular to s.
         */
        class ALane
        {
          public:
            /**
             * isValid - return true if representation of lane is valid
             */
            virtual bool isValid()const =0;
            /**
             * getViewingDistance - returns how far to the horizon the model of the lane extends, given as maximum progress along lane 
             */
            virtual double getViewingDistance()const =0;
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
            virtual double getSpeedLimit(double s)const =0;
            /**
             * hasSpeedRecommendation - return true, if a speed recommendation is available (GLOSA or other infrastructure advice) at a certain distance s along the lane
             */
            virtual bool hasSpeedRecommendation(double s)const =0;
            /**
             * getSpeedRecommendation - return a speed recommendation at a certain distance s along the lane
             */
            virtual double getSpeedRecommendation(double s)const =0;
        };
    }
}