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
#include <string>
#include <map>
#include <adore/fun/setpointrequest.h>
#include <adore/fun/terminalrequest.h>
#include <adore/mad/occupancycylinder.h>

namespace adore
{
    namespace fun
    {
        /**
         * PlanningResult describes result of the planning process for a single maneuver in detail.
         */
        struct PlanningResult
        {
            int id;           /**< numerical planner id*/
            int iteration;    /**< determines to which planning iteration the planning result contributes*/
            std::string name; /**< human-readable identifier for or type of maneuver, e.g. "lane following"*/
            SetPointRequest nominal_maneuver; /**< the nominal maneuver optimzed towards horizon: not to be executed but
                                                 rather for HMI and decision making*/
            SetPointRequest combined_maneuver; /**< combined maneuver starting with nominal_maneuver and transitioning
                                                  to emergency_maneuver ending in safe state*/
            TerminalRequest terminal_maneuver; /**< full-break/standstill after execution of combined maneuver*/
            adore::mad::OccupancyCylinderTree nominal_maneuver_swath;  /**< space-time volume expected to be covered by
                                                                          nominal_maneuver*/
            adore::mad::OccupancyCylinderTree combined_maneuver_swath; /**< space-time volume expected to be covered by
                                                                          combined_maneuver*/

            adore::mad::LLinearPiecewiseFunctionM<double, 4> nominal_maneuver_longitudinal_plan; /**< dim 0: t; dim 1:
                                                                                                  s; dim 2: ds; dim 3:
                                                                                                  dds */
            adore::mad::LLinearPiecewiseFunctionM<double, 3> nominal_maneuver_longitudinal_lbx;  /**< lower bounds for
                                                                                                    planning dim 0: t;
                                                                                                    dim 1: s; dim 2: ds;
                                                                                                    dim 3: dds */
            adore::mad::LLinearPiecewiseFunctionM<double, 3> nominal_maneuver_longitudinal_ubx;  /**< upper bounds for
                                                                                                  planning dim 0: t;
                                                                                                 dim 1: s; dim 2: ds;
                                                                                                 dim 3: dds */
            adore::mad::LLinearPiecewiseFunctionM<double, 4> nominal_maneuver_lateral_plan; /**< dim 0: t; dim 1: n; dim
                                                                                             2: dn; dim 3: ddn */
            adore::mad::LLinearPiecewiseFunctionM<double, 3> nominal_maneuver_lateral_lbx;  /**< lower bounds for
                                                                                             planning  dim 0: t; dim 1:
                                                                                             n;  dim  2: dn; dim 3: ddn */
            adore::mad::LLinearPiecewiseFunctionM<double, 3> nominal_maneuver_lateral_ubx;  /**< upper bounds for
                                                                                             planning  dim 0: t; dim 1:
                                                                                             n;  dim  2: dn; dim 3: ddn */

            bool nominal_maneuver_valid;
            bool combined_maneuver_valid;
            std::string status_string; /**< string explaining for example why constraint evaluation failed, e.g.
                                          "collision with prediction of object 17, branch 3, at t=0.58" or "I/O lin
                                          inversion fail"*/
            std::unordered_map<std::string, double> objective_values;   /**< set of individual objective values, to be
                                                                           weighted by decision making. e.g. acceleration,
                                                                           jerk, time-loss/navigation-cost*/
            std::unordered_map<std::string, double> performance_values; /**< cpu-time measurements, etc. */
            int maneuver_type;                                          /**< */
            static const int COLLISION_MITIGATION = 0;                  /**< */
            static const int EMERGENCY_BREAKING = 1;                    /**< */
            static const int NOMINAL_DRIVING = 2;                       /**< */
            bool indicator_left;
            bool indicator_right;
            PlanningResult()
              : nominal_maneuver_valid(false)
              , combined_maneuver_valid(false)
              , iteration(0)
              , indicator_left(false)
              , indicator_right(false)
            {
            }
            /**
             * @return objective value corresponding to objective name, if entry exists. Otherwise return bound.
             */
            double getObjectiveValue(std::string name,double bound)
            {
                auto it = objective_values.find(name);
                if( it == objective_values.end() ) return bound;
                else return it->second;
            }
        };


        typedef std::vector<PlanningResult> TPlanningResultSet;
    }  // namespace fun
}  // namespace adore
