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
#include <adore/view/alane.h>
#include <adore/params/ap_longitudinal_planner.h>
#include <adore/params/ap_lateral_planner.h>
#include <adore/params/ap_tactical_planner.h>


namespace adore
{
  namespace fun
  {

    /**
     * Plans lane change maneuvers.
     * Specialization of DecoupledLFLCPlanner for lane following maneuver planning.
     * K number of control points for planning.
     * P interpolation points per planning step.
     */
    template<int K,int P>
    class BasicLaneFollowingPlanner:public DecoupledLFLCPlanner<K,P>
    {
      private:
        NominalReferenceSpeed nominalReferenceSpeed_;/**< reference for vehicle speed*/
        BreakAtHorizon breakAtHorizon_;/**< plan to reduce speed to standstill inside visible range*/
        DistanceToLaneEndConstraint keepDistanceToLaneEnd_;/**< plan to reduce speed to standstill before end of lane*/
        CurvatureSpeedLimitPredict curvatureSpeedLimit_;/**< do not exceed lateral acceleration constraints in curve */
        LFVSpeedLimit lfvSpeedLimit_;/**< speed limit*/
        DontDriveBackwards dontDriveBackwards_;/**< limit maneuver to movement in prescribed lane direction*/
        LongitudinalAccelerationConstraint longitudinalAccelerationConstraintUB_;/**< constraint for ax*/
        LongitudinalAccelerationConstraint longitudinalAccelerationConstraintLB_;/**< constraint for ax*/
        // FollowPrecedingVehicle followPrecedingVehicle_; /**< uses time gap*/
        FollowPrecedingVehicle_BreakingCurve followPrecedingVehicle_;/**< uses breaking curve*/
        LowerBoundSGapToPrecedingVehicle lowerBoundSGapToPrecedingVehicle_;
        LaneWidthSpeedLimitLFV laneWidthSpeedLimitLFV_;/**< constraint enforcing slow movement through narrow gaps */
        StopAtBottleneckLFV stopAtBottleneckLFV_;/**< constraint enforcing vehicle to stop, if road is too narrow */
        StopAtNextGoalPoint stopAtNextGoalPoint_;/**< constraint enforcing vehicle to stop, if goal point is reached */
        AdhereToNextLimitLine stopAtRedLight_;/**< constraint enforcing vehicle to stop at next controlled connection, if it is switched to red*/
        AdhereToNextLimitLine stopAtCheckPoint_;/**< constraint enforcing vehicle to stop at next checkpoint*/
        StopAtNextConflictPoint stopAtNextConflictPoint_;/**< */

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
         * @param lfv view for lane following: must be valid pointer
         * @param goalview set to nullptr to ignore. Set to valid ANavigationGoalView in order to stop vehicle at goal
         * @param controlledConnection limit line for red-light on lane, set to nullptr to deactivate
         * @param checkpoint checkpoint on lane, set to nullptr to deactivate
         * @param plon longitudinal planning paramters
         * @param plat lateral planning parameters
         * @param pveh vehicle parameters
         * @param ptrajectory trajectory generation parameters
         * @param lateral_i_grid lateral grid index
         */
        BasicLaneFollowingPlanner(adore::view::ALane* lfv,
                                  adore::view::ANavigationGoalView* goalview,
                                  adore::view::ALimitLineEnRoute* controlledConnection,
                                  adore::view::ALimitLineEnRoute* checkpoint,
                                  adore::params::APLongitudinalPlanner* plon,
                                  adore::params::APLateralPlanner* plat,
                                  adore::params::APTacticalPlanner* ptac,
                                  adore::params::APVehicle* pveh,
                                  adore::params::APTrajectoryGeneration* ptrajectory,
                                  double lateral_i_grid = 0.0)
              : DecoupledLFLCPlanner<K,P>(lfv,plon,plat,pveh,ptrajectory),
                nominalReferenceSpeed_(lfv,plon,ptac),
                breakAtHorizon_(lfv,pveh,ptrajectory),
                keepDistanceToLaneEnd_(lfv,pveh,ptrajectory,ptac),
                curvatureSpeedLimit_(lfv,plon),
                lfvSpeedLimit_(lfv,ptac),
                dontDriveBackwards_(),
                longitudinalAccelerationConstraintUB_(plon,ANominalConstraint::UB),
                longitudinalAccelerationConstraintLB_(plon,ANominalConstraint::LB),
                followPrecedingVehicle_(lfv,pveh,ptac,ptrajectory),
                lowerBoundSGapToPrecedingVehicle_(lfv,pveh,ptac,ptrajectory),
                laneWidthSpeedLimitLFV_(lfv,plon),
                stopAtBottleneckLFV_(lfv,plon,pveh,ptrajectory,ptac),
                stopAtNextGoalPoint_(goalview),
                stopAtNextConflictPoint_(nullptr),
                stopAtRedLight_(controlledConnection),
                stopAtCheckPoint_(checkpoint),
                followCenterlineReference_(lfv,lateral_i_grid),
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
          this->info_.add(&keepDistanceToLaneEnd_);
          this->info_.add(&curvatureSpeedLimit_);
          this->info_.add(&lfvSpeedLimit_);
          this->info_.add(&dontDriveBackwards_);
          this->info_.add(&longitudinalAccelerationConstraintUB_);
          this->info_.add(&longitudinalAccelerationConstraintLB_);
          this->info_.add(&followPrecedingVehicle_);
          this->info_.add(&lowerBoundSGapToPrecedingVehicle_);
          this->info_.add(&stopAtNextGoalPoint_);
          this->info_.add(&stopAtRedLight_);
          this->info_.add(&stopAtCheckPoint_);          
          this->info_.add(&followCenterlineReference_);
          this->info_.add(&lateralAccelerationReference_);
          this->info_.add(&lateralJerkReference_);
          this->info_.add(&lateralOffsetConstraintLFUB_);
          this->info_.add(&lateralOffsetConstraintLFLB_);
          this->info_.add(&headingConstraintUB_);
          this->info_.add(&headingConstraintLB_);
          this->info_.add(&lateralAccelerationConstraintUB_);
          this->info_.add(&lateralAccelerationConstraintLB_);
          this->info_.add(&stopAtNextConflictPoint_);
        }

        void addConstraint(ANominalConstraint* constraint)
        {
          this->info_.add(constraint);
        }

        void addReference(ANominalReference* reference)
        {
          this->info_.add(reference);
        }
        /**
         *  setSpeedScale - define reference speed to be a certain percentage of the 
         */ 
        void setSpeedScale(double value)
        {
          nominalReferenceSpeed_.setSpeedScale(value);
        }
        void setConflictSet(adore::view::AConflictPointSet* conflicts)
        {
          stopAtNextConflictPoint_.setView(conflicts);
        }
    };
  }
}