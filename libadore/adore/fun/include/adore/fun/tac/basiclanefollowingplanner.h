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
#include "decoupled_lflc_planner.h"
#include "basicconstraintsandreferences.h"
#include <adore/view/alanefollowingview.h>
#include <adore/params/ap_longitudinal_planner.h>
#include <adore/params/ap_lateral_planner.h>
#include <adore/params/ap_tactical_planner.h>


namespace adore
{
  namespace fun
  {

    /**
     * Plans lane change maneuvers.
     * Specialization of DecoupledLFLCPlanner for lane change maneuver planning.
     * K number of control points for planning.
     * P interpolation points per planning step.
     */
    template<int K,int P>
    class BasicLaneFollowingPlanner:public DecoupledLFLCPlanner<K,P>
    {
      private:
        NominalReferenceSpeed nominalReferenceSpeed_;/**< reference for vehicle speed*/
        BreakAtHorizon breakAtHorizon_;/**< plan to reduce speed to standstill inside visible range*/
        CurvatureSpeedLimit curvatureSpeedLimit_;/**< do not exceed lateral acceleration constraints in curve */
        LFVSpeedLimit lfvSpeedLimit_;/**< speed limit*/
        DontDriveBackwards dontDriveBackwards_;/**< limit maneuver to movement in prescribed lane direction*/
        LongitudinalAccelerationConstraint longitudinalAccelerationConstraintUB_;/**< constraint for ax*/
        LongitudinalAccelerationConstraint longitudinalAccelerationConstraintLB_;/**< constraint for ax*/
        FollowPrecedingVehicle followPrecedingVehicle_;
        LaneWidthSpeedLimitLFV laneWidthSpeedLimitLFV_;/**< constraint enforcing slow movement through narrow gaps */
        StopAtBottleneckLFV stopAtBottleneckLFV_;/**< constraint enforcing vehicle to stop, if road is too narrow */

        FollowCenterlineReference followCenterlineReference_;/**< reference for lateral position: follow the middle of the lane*/
        LateralAccelerationReference lateralAccelerationReference_;/**< curvature compensation*/
        LateralJerkReference lateralJerkReference_;/**< lateral jerk compensation*/
        LateralOffsetConstraintLF lateralOffsetConstraintLFUB_;/**< lateral position bounds governed by lane boundaries*/
        LateralOffsetConstraintLF lateralOffsetConstraintLFLB_;/**< lateral position bounds governed by lane boundaries*/
        HeadingConstraint headingConstraintUB_;/**< constraint for deviation from road direction */
        HeadingConstraint headingConstraintLB_;/**< constraint for deviation from road direction */
        LateralAccelerationConstraint lateralAccelerationConstraintUB_;/**< constraint for ay*/
        LateralAccelerationConstraint lateralAccelerationConstraintLB_;/**< constraint for ay*/
      public:
        /**
         * Constructor.
         * Initializes references and constraints by linking them to required data abstraction views and parameters.
         * @param lfv view for lane following
         * @param plon longitudinal planning paramters
         * @param plat lateral planning parameters
         * @param pveh vehicle parameters
         * @param ptrajectory trajectory generation parameters
         */
        BasicLaneFollowingPlanner(adore::view::ALaneFollowingView* lfv,
                                  adore::params::APLongitudinalPlanner* plon,
                                  adore::params::APLateralPlanner* plat,
                                  adore::params::APTacticalPlanner* ptac,
                                  adore::params::APVehicle* pveh,
                                  adore::params::APTrajectoryGeneration* ptrajectory)
              : DecoupledLFLCPlanner<K,P>(lfv,plon,plat,pveh,ptrajectory),
                nominalReferenceSpeed_(lfv,plon,ptac),
                breakAtHorizon_(lfv),
                curvatureSpeedLimit_(lfv,plon),
                lfvSpeedLimit_(lfv,ptac),
                dontDriveBackwards_(),
                longitudinalAccelerationConstraintUB_(plon,ANominalConstraint::UB),
                longitudinalAccelerationConstraintLB_(plon,ANominalConstraint::LB),
                followPrecedingVehicle_(lfv,pveh,ptac,ptrajectory),
                laneWidthSpeedLimitLFV_(lfv,plon),
                stopAtBottleneckLFV_(lfv,plon,pveh,ptrajectory,ptac),
                followCenterlineReference_(),
                lateralAccelerationReference_(lfv),
                lateralJerkReference_(lfv),
                lateralOffsetConstraintLFUB_(lfv,pveh,plat,ANominalConstraint::UB),
                lateralOffsetConstraintLFLB_(lfv,pveh,plat,ANominalConstraint::LB),
                headingConstraintUB_(plat,ANominalConstraint::UB),
                headingConstraintLB_(plat,ANominalConstraint::LB),
                lateralAccelerationConstraintUB_(lfv,plat,ANominalConstraint::UB),
                lateralAccelerationConstraintLB_(lfv,plat,ANominalConstraint::LB)
        {
          this->info_.add(&nominalReferenceSpeed_);
          this->info_.add(&breakAtHorizon_);
          this->info_.add(&curvatureSpeedLimit_);
          this->info_.add(&lfvSpeedLimit_);
          this->info_.add(&dontDriveBackwards_);
          this->info_.add(&longitudinalAccelerationConstraintUB_);
          this->info_.add(&longitudinalAccelerationConstraintLB_);
          this->info_.add(&followPrecedingVehicle_);
          this->info_.add(&laneWidthSpeedLimitLFV_);
          this->info_.add(&stopAtBottleneckLFV_);
          this->info_.add(&followCenterlineReference_);
          this->info_.add(&lateralAccelerationReference_);
          this->info_.add(&lateralJerkReference_);
          this->info_.add(&lateralOffsetConstraintLFUB_);
          this->info_.add(&lateralOffsetConstraintLFLB_);
          this->info_.add(&headingConstraintUB_);
          this->info_.add(&headingConstraintLB_);
          this->info_.add(&lateralAccelerationConstraintUB_);
          this->info_.add(&lateralAccelerationConstraintLB_);
        }

        /**
         *  setSpeedScale - define reference speed to be a certain percentage of the 
         */ 
        void setSpeedScale(double value)
        {
          nominalReferenceSpeed_.setSpeedScale(value);
        }
    };
  }
}