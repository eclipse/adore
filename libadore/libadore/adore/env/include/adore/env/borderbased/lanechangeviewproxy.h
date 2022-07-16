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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/view/alanechangeview.h>
#include <adore/env/borderbased/lanegeometrydataproxy.h>
#include <adore/env/borderbased/lanefollowingviewproxy.h>

// TODO dependency on traffic and conflictset should be removed once it is removed in alane.h
#include <adore/env/traffic/egolanetraffic.h>
#include <adore/env/borderbased/conflictset.h>

namespace adore
{
    namespace env
    {
        namespace BorderBased
        {
            /**
             * @brief Proxy class to access ALane interfaces from preprocessed lane geometry received as data object
             *
             */
            class LaneChangeViewProxy : public adore::view::ALaneChangeView
            {
              private:
                // std::shared_ptr<LaneGeometryDataProxy> lane_;
                // std::shared_ptr<LaneGeometryDataProxy> targetLane_;
                std::shared_ptr<LaneFollowingViewProxy> sourceLane_;
                std::shared_ptr<LaneFollowingViewProxy> targetLane_;
                std::shared_ptr<LaneChangeDataProxy> laneChangeData_;
                // double limitS(double s)
                // {
                //     // std::cout << "s_min " << lane_->s_min << " - s_max " << lane_->s_max << std::flush;
                //     return adore::mad::bound(lane_->s_min, s + lane_->s_min, lane_->s_max);
                // }

              public:
                /**
                 * @brief Construct a new Lane Following View Proxy object
                 *
                 * @param lane preprocessed data object containing linearized lane geometry
                 */
                LaneChangeViewProxy(std::shared_ptr<LaneGeometryDataProxy> sourceLaneGeometryData,
                                    std::shared_ptr<LaneGeometryDataProxy> targetLaneGeometryData,
                                    std::shared_ptr<LaneChangeDataProxy> laneChangeData)
                  : sourceLane_(std::make_shared<LaneFollowingViewProxy>(sourceLaneGeometryData))
                  , targetLane_(std::make_shared<LaneFollowingViewProxy>(targetLaneGeometryData))
                  , laneChangeData_(laneChangeData)
                {
                }

                virtual adore::view::ALane* getSourceLane() override
                {
                    throw std::logic_error("Not implemented"); // seems to never be used
                    return sourceLane_.get();
                }
                /**
                 * getSourceLane - return ALane pointer for the target lane of the lane change
                 */
                virtual adore::view::ALane* getTargetLane() override
                {
                    return targetLane_.get();
                }
                /**
                 * getLCDirection - return the direction of a lane change leading to target lane
                 */
                virtual adore::view::ALaneChangeView::direction getLCDirection() const override
                {
                    return laneChangeData_->direction;
                }
                /**
                 * getProgressOfGateOpen - return progress s of the next opening of a gate (distance to end of solid
                 * line or otherwise impassable lane border)
                 */
                virtual double getProgressOfGateOpen() const override
                {
                    return laneChangeData_->gate_s0;
                }
                /**
                 * getProgressOfGateClosed - return progress s of the closure of the next gate (distance to beginngin of
                 * solid line or otherwise impassable lane border after gate)
                 */
                virtual double getProgressOfGateClosed() const override
                {
                    return laneChangeData_->gate_s1;
                }
                /**
                 * getOffsetOfStartOuterBorder - return lateral offset n of the outer border of the AV's current lane
                 */
                virtual double getOffsetOfStartOuterBorder(double s) override
                {
                    return laneChangeData_->sourceOuterBorderDistance_fct(s);
                }
                /**
                 * getOffsetOfStartInnerBorder - return lateral offset n of the inner border of the AV's current lane
                 */
                virtual double getOffsetOfSeparatingBorder(double s) override
                {
                    return laneChangeData_->separatingBorderDistance_fct(s);
                }
                /**
                 * getOffsetOfDestinationOuterBorder - return lateral offset n of the outer border of target lane
                 */
                virtual double getOffsetOfDestinationOuterBorder(double s) override
                {
                    return laneChangeData_->targetOuterBorderDistance_fct(s);
                }
                virtual double getNavigationCostDifference() override
                {
                    return 0.0;
                }

            };
        }  // namespace BorderBased
    }      // namespace env
}  // namespace adore
