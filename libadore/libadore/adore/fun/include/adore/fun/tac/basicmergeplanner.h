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
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/


#pragma once
#include "decoupled_lflc_planner.h"
#include "basicconstraintsandreferences.h"
#include "lanechangeconstraintsandreferences.h"
#include <adore/view/alane.h>
#include <adore/params/ap_longitudinal_planner.h>
#include <adore/params/ap_lateral_planner.h>
#include <adore/params/ap_tactical_planner.h>


namespace adore
{
  namespace fun
  {
    /**
     * Plans merge maneuvers, which finalize a lane change.
     * Specialization of DecoupledLFLCPlanner for merge maneuver planning.
     * The merge planner uses the lateral reference of lane following, but the constraints of the lane change: 
     * When the view switches lanes in the middle of the lane change, the vehicle may thus enter the new center lane.
     */
    template<int K,int P>
    class BasicMergePlanner:public DecoupledLFLCPlanner<K,P>
    {
      private:
        NominalReferenceSpeed nominalReferenceSpeed_;/**< speed reference*/
        BreakAtHorizon breakAtHorizon_;/**< plan to reduce speed to standstill inside visible range*/
        CurvatureSpeedLimitPredict curvatureSpeedLimit_;/**< do not exceed lateral acceleration constraints in curve*/
        LFVSpeedLimit lfvSpeedLimit_;/**< speed limit*/
        DontDriveBackwards dontDriveBackwards_;/**< only drive forwards*/
        LongitudinalAccelerationConstraint longitudinalAccelerationConstraintUB_;/**< constraint for ax*/
        LongitudinalAccelerationConstraint longitudinalAccelerationConstraintLB_;/**< constraint for ax*/
        StopAtNextGoalPoint stopAtNextGoalPoint_;/**< constraint enforcing vehicle to stop, if goal point is reached */
        AdhereToNextLimitLine stopAtRedLight_;/**< constraint enforcing vehicle to stop at next controlled connection, if it is switched to red*/
        AdhereToNextLimitLine stopAtCheckPoint_;/**< constraint enforcing vehicle to stop at next checkpoint*/
        FollowPrecedingVehicle_BreakingCurve followPrecedingVehicleOnSource_;/**< constraint enforcing vehicle to follow next vehicle on source lane*/
        FollowPrecedingVehicle_BreakingCurve followPrecedingVehicleOnTarget_;/**< constraint enforcing vehicle to follow next vehicle on target lane*/

        LateralAccelerationReference lateralAccelerationReference_;/**< curvature supression*/
        LateralJerkReference lateralJerkReference_;/**< jerk supression*/
        HeadingConstraint headingConstraintUB_;/**< constraint for yaw angle deviation from road direction*/
        HeadingConstraint headingConstraintLB_;/**< constraint for yaw angle deviation from road direction*/
        LateralAccelerationConstraint lateralAccelerationConstraintUB_;/**< constraint for ay*/
        LateralAccelerationConstraint lateralAccelerationConstraintLB_;/**< constraint for ay*/
        LateralOffsetConstraintLM lateralOffsetConstraint_left_;/**< constraint for staying in lane boundaries */
        LateralOffsetConstraintLM lateralOffsetConstraint_right_;/**< constraint for staying in lane boundaries */
        FollowCenterlineReference followCenterlineReference_;/** reference for vehicle lateral position */
      public:

        /**
         * Constructor.
         * Initializes references and constraints by linking them to required data abstraction views and parameters.
         * @param lcv view for lane changing (determines direction of lane change, as well as source and target lane)
         * @param plon longitudinal planning paramters
         * @param plat lateral planning parameters
         * @param pveh vehicle parameters
         * @param ptrajectory trajectory generation parameters
         */
        BasicMergePlanner(   adore::view::ALaneChangeView* lcv,
                                  adore::view::ANavigationGoalView* goalview,
                                  adore::view::ALimitLineEnRoute* controlledConnection,
                                  adore::view::ALimitLineEnRoute* checkpoint,
                                  adore::params::APLongitudinalPlanner* plon,
                                  adore::params::APLateralPlanner* plat,
                                  adore::params::APTacticalPlanner* ptac,
                                  adore::params::APVehicle* pveh,
                                  adore::params::APTrajectoryGeneration* ptrajectory)
              : DecoupledLFLCPlanner<K,P>(lcv->getSourceLane(),plon,plat,pveh,ptrajectory),
                nominalReferenceSpeed_(lcv->getSourceLane(),plon,ptac),
                breakAtHorizon_(lcv->getSourceLane(),pveh,ptrajectory),
                curvatureSpeedLimit_(lcv->getSourceLane(),plon),
                lfvSpeedLimit_(lcv->getSourceLane(),ptac),
                dontDriveBackwards_(),
                longitudinalAccelerationConstraintUB_(plon,ANominalConstraint::UB),
                longitudinalAccelerationConstraintLB_(plon,ANominalConstraint::LB),
                lateralAccelerationReference_(lcv->getSourceLane()),
                lateralJerkReference_(lcv->getSourceLane()),
                followPrecedingVehicleOnSource_(lcv->getSourceLane(),pveh,ptac,ptrajectory),
                followPrecedingVehicleOnTarget_(lcv->getTargetLane(),pveh,ptac,ptrajectory),
                stopAtNextGoalPoint_(goalview),
                stopAtRedLight_(controlledConnection),
                stopAtCheckPoint_(checkpoint),
                headingConstraintUB_(plat,ANominalConstraint::UB),
                headingConstraintLB_(plat,ANominalConstraint::LB),
                lateralAccelerationConstraintUB_(lcv->getSourceLane(),plat,ANominalConstraint::UB),
                lateralAccelerationConstraintLB_(lcv->getSourceLane(),plat,ANominalConstraint::LB),
                lateralOffsetConstraint_left_(lcv,pveh,plat,ANominalConstraint::UB),
                lateralOffsetConstraint_right_(lcv,pveh,plat,ANominalConstraint::LB),
                followCenterlineReference_(lcv->getSourceLane())
        {
          this->info_.add(&nominalReferenceSpeed_);
          this->info_.add(&breakAtHorizon_);
          this->info_.add(&curvatureSpeedLimit_);
          this->info_.add(&lfvSpeedLimit_);
          this->info_.add(&dontDriveBackwards_);
          this->info_.add(&longitudinalAccelerationConstraintUB_);
          this->info_.add(&longitudinalAccelerationConstraintLB_);
          this->info_.add(&followPrecedingVehicleOnSource_);
          this->info_.add(&followPrecedingVehicleOnTarget_);
          this->info_.add(&stopAtNextGoalPoint_);
          this->info_.add(&stopAtRedLight_);
          this->info_.add(&stopAtCheckPoint_);
          this->info_.add(&lateralAccelerationReference_);
          this->info_.add(&lateralJerkReference_);
          this->info_.add(&headingConstraintUB_);
          this->info_.add(&headingConstraintLB_);
          this->info_.add(&lateralAccelerationConstraintUB_);
          this->info_.add(&lateralAccelerationConstraintLB_);
          this->info_.add(&lateralOffsetConstraint_left_);
          this->info_.add(&lateralOffsetConstraint_right_);
          this->info_.add(&followCenterlineReference_);
        }

    };
  }
}