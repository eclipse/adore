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
#include "mrmplanner.h"
#include "basicconstraintsandreferences.h"
#include <adore/view/alane.h>
#include <adore/params/ap_longitudinal_planner.h>
#include <adore/params/ap_lateral_planner.h>
#include <adore/params/ap_tactical_planner.h>
#include "lanechangeconstraintsandreferences.h"


namespace adore
{
  namespace fun
  {

    /**
     * @brief Plans minimum risk maneuver to cancel lane change
     */
    template<int K,int P>
    class ContinueLCMRMPlanner:public MRMPlanner<K,P>
    {
      private:
        ContinueLCReference lateralReference_;
        LateralAccelerationReference lateralAccelerationReference_;/**< curvature compensation*/
        LateralJerkReference lateralJerkReference_;/**< lateral jerk compensation*/
        LateralOffsetConstraintLC lateralOffsetConstraint_left_;/**< constraint for staying in lane boundaries, which are selected according to gap state*/
        LateralOffsetConstraintLC lateralOffsetConstraint_right_;/**< constraint for staying in lane boundaries, which are selected according to gap state*/
        HeadingConstraint headingConstraintUB_;/**< constraint for deviation from road direction */
        HeadingConstraint headingConstraintLB_;/**< constraint for deviation from road direction */
        LateralAccelerationConstraint lateralAccelerationConstraintUB_;/**< constraint for ay*/
        LateralAccelerationConstraint lateralAccelerationConstraintLB_;/**< constraint for ay*/
        LaneChangeIntoGapReference LaneChangeIntoGapReference_;/** reference for vehicle lateral position */

        FollowPrecedingVehicle followVehicleConstraint_;/**< post-processing constraint for longitudinal plan */

      public:
        /**
         * Constructor.
         * Initializes references and constraints by linking them to required data abstraction views and parameters.
         * @param lfv view for lane changes
         * @param plat lateral planning parameters
         * @param pveh vehicle parameters
         * @param ptrajectory trajectory generation parameters
         */
        ContinueLCMRMPlanner(adore::view::ALaneChangeView* lcv,
                                  adore::params::APLateralPlanner* plat,
                                  adore::params::APTacticalPlanner* ptac,
                                  adore::params::APVehicle* pveh,
                                  adore::params::APTrajectoryGeneration* ptrajectory,
                                  double lateral_i_grid = 0.0)
              : MRMPlanner<K,P>(lcv->getTargetLane(),plat,pveh,ptrajectory),            
                followVehicleConstraint_(lcv->getSourceLane(),pveh,ptac,ptrajectory),
                lateralReference_(lcv),
                LaneChangeIntoGapReference_(lcv,pveh,plat,lateral_i_grid),                
                lateralAccelerationReference_(lcv->getTargetLane()),
                lateralJerkReference_(lcv->getTargetLane()),
                lateralOffsetConstraint_left_(lcv,pveh,plat,ANominalConstraint::UB),
                lateralOffsetConstraint_right_(lcv,pveh,plat,ANominalConstraint::LB),
                headingConstraintUB_(plat,ANominalConstraint::UB),
                headingConstraintLB_(plat,ANominalConstraint::LB),
                lateralAccelerationConstraintUB_(lcv->getTargetLane(),plat,ANominalConstraint::UB),
                lateralAccelerationConstraintLB_(lcv->getTargetLane(),plat,ANominalConstraint::LB)
        {
          // this->getInformationSet().add(&followVehicleConstraint_);
          // this->getOffsetSolver().getInformationSet().add(&lateralReference_);
          this->getOffsetSolver().getInformationSet().add(&LaneChangeIntoGapReference_);
          this->getOffsetSolver().getInformationSet().add(&lateralAccelerationReference_);
          this->getOffsetSolver().getInformationSet().add(&lateralJerkReference_);
          this->getOffsetSolver().getInformationSet().add(&lateralOffsetConstraint_left_);
          this->getOffsetSolver().getInformationSet().add(&lateralOffsetConstraint_right_);
          this->getOffsetSolver().getInformationSet().add(&headingConstraintUB_);
          this->getOffsetSolver().getInformationSet().add(&headingConstraintLB_);
          this->getOffsetSolver().getInformationSet().add(&lateralAccelerationConstraintUB_);
          this->getOffsetSolver().getInformationSet().add(&lateralAccelerationConstraintLB_);
        }

        void setGap(adore::view::AGap* gap)
        {
          lateralOffsetConstraint_left_.setGap(gap);
          lateralOffsetConstraint_right_.setGap(gap);
          LaneChangeIntoGapReference_.setGap(gap);
        }

        void addConstraint(ANominalConstraint* constraint)
        {
          this->info_.add(constraint);
        }

        void addReference(ANominalReference* reference)
        {
          this->info_.add(reference);
        }
    };
  }
}