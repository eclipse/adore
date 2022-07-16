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


namespace adore
{
  namespace fun
  {

    /**
     * @brief Plans minimum risk maneuvers on given lane. 
     * K number of control points for planning.
     * P interpolation points per planning step.
     */
    template<int K,int P>
    class BasicMRMPlanner:public MRMPlanner<K,P>
    {
      private:
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
         * @param plat lateral planning parameters
         * @param pveh vehicle parameters
         * @param ptrajectory trajectory generation parameters
         * @param lateral_i_grid offset variation
         */
        BasicMRMPlanner(adore::view::ALane* lfv,
                                  adore::params::APLateralPlanner* plat,
                                  adore::params::APTacticalPlanner* ptac,
                                  adore::params::APVehicle* pveh,
                                  adore::params::APTrajectoryGeneration* ptrajectory,
                                  double lateral_i_grid = 0.0)
              : MRMPlanner<K,P>(lfv,plat,pveh,ptrajectory),
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
          this->getOffsetSolver().getInformationSet().add(&followCenterlineReference_);
          this->getOffsetSolver().getInformationSet().add(&lateralAccelerationReference_);
          this->getOffsetSolver().getInformationSet().add(&lateralJerkReference_);
          this->getOffsetSolver().getInformationSet().add(&lateralOffsetConstraintLFUB_);
          this->getOffsetSolver().getInformationSet().add(&lateralOffsetConstraintLFLB_);
          this->getOffsetSolver().getInformationSet().add(&headingConstraintUB_);
          this->getOffsetSolver().getInformationSet().add(&headingConstraintLB_);
          this->getOffsetSolver().getInformationSet().add(&lateralAccelerationConstraintUB_);
          this->getOffsetSolver().getInformationSet().add(&lateralAccelerationConstraintLB_);
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