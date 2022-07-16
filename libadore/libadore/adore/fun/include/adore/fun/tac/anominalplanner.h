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

#include <adore/fun/setpointrequest.h>
#include <adore/fun/vehiclemotionstate9d.h>

namespace adore
{
  namespace fun
  {
    /**
     * Interface for typical motion planning for automated driving.
     */
    class ANominalPlanner
    {
      public:
        /**
         * compute - try to compute a trajectory according to given constraints and objective
         */
        virtual void compute(const VehicleMotionState9d& initial_state) = 0;
        /**
         *  hasValidPlan - return true, if a trajectory was computed, which satisfies given constraints
         */
        virtual bool hasValidPlan()const = 0;
        /**
         * getSetPointRequest - return computed trajectory in the form of a SetPointRequest
         */
        virtual const SetPointRequest* getSetPointRequest()const =0;
        /**
         *  getCPUTime - return the time require for trajectory planning in seconds
         */
        virtual double getCPUTime()const  = 0;
    };

    /**
     * Interface for constriants specified in road-relative coordinates.
     */
    class ANominalConstraint
    {
      public:
        enum ConstraintDirection
        {
          LB,UB
        };
        /**
         * return the value of the constraint at a certain time and position
         * @param t the time
         * @param s progress along the road-relative coordinate system
         * @param ds derivative of s
         * @return value of the constraint (e.g. vmax for a speed limit)
         */
        virtual double getValue(double t,double s,double ds)const =0;
        /**
         * Refresh values of the constraint object.
         * Allows parameters and precomputations to buffered for one planning cycle.
         * @param t0 start time for planning
         * @param s0 start progress in road-relative coordinate system
         * @param ds0 derivative of s at t0
         */
        virtual void update(double t0,double s0,double ds0)=0;
        /**
         * Determine whether this constraint is an upper or lower bound.
         * @return lb or ub
         */
        virtual ConstraintDirection getDirection()=0;
        /**
         * Determine for which dimension this constraint is responsible (e.g. longitudinal, lateral, etc.)
         * @return applicable to which dimension, depends on setup of planner
         */
        virtual int getDimension()=0;
        /**
         * Determine for which derivative of given dimension this constraint is responsible.
         * @return applicable to which state derivative
         */
        virtual int getDerivative()=0;
    };

    /**
     * Interface for reference values specified in road-relative coordinates.
     * The term reference is here applied as in convex optimization, where the deviation from the reference is penalized.
     */
    class ANominalReference
    {
      public:
        /**
         * Determine whether a reference is available and retrieve its value.
         * @param t time
         * @param s progress along coordinate system 
         * @return true, if a reference is available
         */
        virtual bool getValueIfAvailable(double t, double s, double ds,double & ref)const =0;
        /**
         * Refresh values of the reference object.
         * Allows parameters and precomputations to buffered for one planning cycle.
         * @param t0 start time for planning
         * @param s0 start progress in road-relative coordinate system
         * @param ds0 derivative of s at t0
         */
        virtual void update(double t0,double s0,double ds0)=0;
        /**
         * Determine for which dimension this reference is responsible (e.g. longitudinal, lateral, etc.)
         * @return applicable to which dimension, depends on setup of planner
         */
        virtual int getDimension()=0;
        /**
         * Determine for which derivative of given dimension this reference is responsible.
         * @return applicable to which state derivative
         */
        virtual int getDerivative()=0;
    };
  }
}