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
     * Plans lane change maneuvers. The maneuver is limited in that front vehicle may not be overtaken during maneuver.
     * Specialization of DecoupledLFLCPlanner for lane change maneuver planning.
     */
    template<int K,int P>
    class BasicLaneChangePlanner:public DecoupledLFLCPlanner<K,P>
    {
      private:
        NominalReferenceSpeed nominalReferenceSpeed_;/**< speed reference*/
        BreakAtHorizon breakAtHorizon_;/**< plan to reduce speed to standstill inside visible range*/
        CurvatureSpeedLimitPredict curvatureSpeedLimit_;/**< do not exceed lateral acceleration constraints in curve*/
        LFVSpeedLimit lfvSpeedLimit_onTarget_;/**< speed limit*/
        LFVSpeedLimit lfvSpeedLimit_onSource_;/**< speed limit*/
        DontDriveBackwards dontDriveBackwards_;/**< only drive forwards*/
        LongitudinalAccelerationConstraint longitudinalAccelerationConstraintUB_;/**< constraint for ax*/
        LongitudinalAccelerationConstraint longitudinalAccelerationConstraintLB_;/**< constraint for ax*/
        LaneWidthSpeedLimitLFV laneWidthSpeedLimitLFV_;/**< constraint enforcing slow movement through narrow gaps */
        StopAtBottleneckLFV stopAtBottleneckLFV_;/**<  @TODO: rework this for lcv, constraint enforcing vehicle to stop, if road is too narrow */
        StopAtNextGoalPoint stopAtNextGoalPoint_;/**< constraint enforcing vehicle to stop, if goal point is reached */
        AdhereToNextLimitLine stopAtRedLightOnSource_;/**< constraint enforcing vehicle to stop at next controlled connection, if it is switched to red*/
        AdhereToNextLimitLine stopAtRedLightOnTarget_;/**< constraint enforcing vehicle to stop at next controlled connection, if it is switched to red*/
        AdhereToNextLimitLine stopAtCheckPointOnSource_;/**< constraint enforcing vehicle to stop at next checkpoint_on_source*/
        AdhereToNextLimitLine stopAtCheckPointOnTarget_;/**< constraint enforcing vehicle to stop at next checkpoint_on_target*/
        FollowPrecedingVehicle_BreakingCurve followPrecedingVehicleOnSource_;/**< constraint enforcing vehicle to follow next vehicle on source lane*/
        FollowPrecedingVehicle_BreakingCurve followPrecedingVehicleOnTarget_;/**< constraint enforcing vehicle to follow next vehicle on target lane*/
        LowerBoundSGapToPrecedingVehicle lowerBoundSGapToPrecedingVehicleOnTarget_;

        LateralAccelerationReference lateralAccelerationReference_;/**< curvature supression*/
        LateralJerkReference lateralJerkReference_;/**< jerk supression*/
        HeadingConstraint headingConstraintUB_;/**< constraint for yaw angle deviation from road direction*/
        HeadingConstraint headingConstraintLB_;/**< constraint for yaw angle deviation from road direction*/
        LateralAccelerationConstraint lateralAccelerationConstraintUB_;/**< constraint for ay*/
        LateralAccelerationConstraint lateralAccelerationConstraintLB_;/**< constraint for ay*/
        LateralOffsetConstraintLC lateralOffsetConstraint_left_;/**< constraint for staying in lane boundaries, which are selected according to gap state*/
        LateralOffsetConstraintLC lateralOffsetConstraint_right_;/**< constraint for staying in lane boundaries, which are selected according to gap state*/
        LaneChangeIntoGapReference laneChangeIntoGapReference_;/** reference for vehicle lateral position */
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
        BasicLaneChangePlanner(   adore::view::ALaneChangeView* lcv,
                                  adore::view::ANavigationGoalView* goalview,
                                  adore::view::ALimitLineEnRoute* controlledConnectionOnSource,
                                  adore::view::ALimitLineEnRoute* controlledConnectionOnTarget,
                                  adore::view::ALimitLineEnRoute* checkpoint_on_source,
                                  adore::view::ALimitLineEnRoute* checkpoint_on_target,
                                  adore::params::APLongitudinalPlanner* plon,
                                  adore::params::APLateralPlanner* plat,
                                  adore::params::APTacticalPlanner* ptac,
                                  adore::params::APVehicle* pveh,
                                  adore::params::APTrajectoryGeneration* ptrajectory,
                                  double lateral_i_grid = 0.0)
              : DecoupledLFLCPlanner<K,P>(lcv->getTargetLane(),plon,plat,pveh,ptrajectory),
                nominalReferenceSpeed_(lcv->getTargetLane(),plon,ptac),
                breakAtHorizon_(lcv->getTargetLane(),pveh,ptrajectory),
                curvatureSpeedLimit_(lcv->getTargetLane(),plon),
                lfvSpeedLimit_onTarget_(lcv->getTargetLane(),ptac),
                lfvSpeedLimit_onSource_(lcv->getSourceLane(),ptac),
                dontDriveBackwards_(),
                longitudinalAccelerationConstraintUB_(plon,ANominalConstraint::UB),
                longitudinalAccelerationConstraintLB_(plon,ANominalConstraint::LB),
                lateralAccelerationReference_(lcv->getTargetLane()),
                lateralJerkReference_(lcv->getTargetLane()),
                followPrecedingVehicleOnSource_(lcv->getSourceLane(),pveh,ptac,ptrajectory),
                followPrecedingVehicleOnTarget_(lcv->getTargetLane(),pveh,ptac,ptrajectory),
                lowerBoundSGapToPrecedingVehicleOnTarget_(lcv->getTargetLane(),pveh,ptac,ptrajectory),
                laneWidthSpeedLimitLFV_(lcv->getTargetLane(),plon),
                stopAtBottleneckLFV_(lcv->getTargetLane(),plon,pveh,ptrajectory,ptac),
                stopAtNextGoalPoint_(goalview),
                stopAtRedLightOnSource_(controlledConnectionOnSource),
                stopAtRedLightOnTarget_(controlledConnectionOnTarget),
                stopAtCheckPointOnSource_(checkpoint_on_source),
                stopAtCheckPointOnTarget_(checkpoint_on_target),
                headingConstraintUB_(plat,ANominalConstraint::UB),
                headingConstraintLB_(plat,ANominalConstraint::LB),
                lateralAccelerationConstraintUB_(lcv->getTargetLane(),plat,ANominalConstraint::UB),
                lateralAccelerationConstraintLB_(lcv->getTargetLane(),plat,ANominalConstraint::LB),
                lateralOffsetConstraint_left_(lcv,pveh,plat,ANominalConstraint::UB),
                lateralOffsetConstraint_right_(lcv,pveh,plat,ANominalConstraint::LB),
                laneChangeIntoGapReference_(lcv,pveh,plat,lateral_i_grid)                
        {
          this->info_.add(&nominalReferenceSpeed_);
          this->info_.add(&breakAtHorizon_);
          this->info_.add(&stopAtCheckPointOnSource_);
          this->info_.add(&stopAtCheckPointOnTarget_);
          this->info_.add(&stopAtRedLightOnSource_);
          this->info_.add(&stopAtRedLightOnTarget_);          
          this->info_.add(&stopAtNextGoalPoint_);
          this->info_.add(&curvatureSpeedLimit_);
          this->info_.add(&lfvSpeedLimit_onTarget_);
          this->info_.add(&lfvSpeedLimit_onSource_);
          this->info_.add(&dontDriveBackwards_);
          this->info_.add(&longitudinalAccelerationConstraintUB_);
          this->info_.add(&longitudinalAccelerationConstraintLB_);
          // this->info_.add(&laneWidthSpeedLimitLFV_);
          this->info_.add(&followPrecedingVehicleOnSource_);
          this->info_.add(&followPrecedingVehicleOnTarget_);
          this->info_.add(&lowerBoundSGapToPrecedingVehicleOnTarget_);

          this->info_.add(&laneChangeIntoGapReference_);
          this->info_.add(&lateralAccelerationReference_);
          this->info_.add(&lateralJerkReference_);

          this->info_.add(&lateralOffsetConstraint_left_);
          this->info_.add(&lateralOffsetConstraint_right_);
          this->info_.add(&headingConstraintUB_);
          this->info_.add(&headingConstraintLB_);
          this->info_.add(&lateralAccelerationConstraintUB_);
          this->info_.add(&lateralAccelerationConstraintLB_);
        }
        void setGap(adore::view::AGap* gap)
        {
          lateralOffsetConstraint_left_.setGap(gap);
          lateralOffsetConstraint_right_.setGap(gap);
          laneChangeIntoGapReference_.setGap(gap);
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