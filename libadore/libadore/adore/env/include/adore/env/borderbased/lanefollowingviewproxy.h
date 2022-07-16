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
#include <adore/view/alane.h>
#include <adore/env/borderbased/lanegeometrydataproxy.h>

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
            class LaneFollowingViewProxy : public adore::view::ALane
            {
              private:
                std::shared_ptr<LaneGeometryDataProxy> lane_;
                double limitS(double s)
                {
                    // std::cout << "s_min " << lane_->s_min << " - s_max " << lane_->s_max << std::flush;
                    return adore::mad::bound(lane_->centerSmoothed_fct.limitLo(), 
                        s+lane_->centerSmoothed_fct.limitLo(),
                         lane_->centerSmoothed_fct.limitHi());
                }

              public:
                /**
               * @brief Construct a new Lane Following View Proxy object
               * 
               * @param lane preprocessed data object containing linearized lane geometry
               */
                LaneFollowingViewProxy(std::shared_ptr<LaneGeometryDataProxy> lane) : lane_(lane)
                {
                }

                bool isValid() const override
                {
                    return lane_->isValid;
                };
                virtual double getSMin()const override
                {
                    return 0.0;
                }
                virtual double getSMax()const override
                {
                    return lane_->centerSmoothed_fct.limitHi()-lane_->centerSmoothed_fct.limitLo();
                }

                double getProgressOfWidthOpen() const override
                {
                    //@TODO
                    return lane_->centerSmoothed_fct.limitLo();
                }
                double getProgressOfWidthClosed() const override
                {
                    //@TODO
                    return lane_->centerSmoothed_fct.limitHi();
                }
                const adore::view::TrafficQueue& getOnLaneTraffic() const override
                {
                    throw std::logic_error("Not implemented");
                    // adore::env::traffic::EgoLaneTraffic t(nullptr);
                    // return t.getQueue();
                };

                const ConflictSet& getConflictSet() const override
                {
                    throw std::logic_error("Not implemented");
                    // adore::env::BorderBased::ConflictSet cs_(nullptr);
                    // return cs_;
                }

                double getSpeedLimit(double s) override
                {
                    throw std::logic_error("Not implemented");
                    return 20.0;
                }

                virtual double getLeftIndicatorHint(double s) override { throw std::logic_error("Not implemented"); return 0.0; }

                virtual double getRightIndicatorHint(double s) override { throw std::logic_error("Not implemented"); return 0.0; }

                bool hasSpeedRecommendation(double s) const override
                {
                    throw std::logic_error("Not implemented");
                    return false;
                }

                double getSpeedRecommendation(double s) const override
                {
                    throw std::logic_error("Not implemented");
                    return 0.0;
                }
                double getNavigationCost(double s) override
                {
                    return lane_->navigationCost_fct(limitS(s));
                }
                virtual void boundNavigationCost(double s0,double s1,double& cmin,double& cmax) override
                {
                    adoreMatrix<double,1,1> y0,y1;
                    lane_->navigationCost_fct.bound(limitS(s0),limitS(s1),y0,y1);
                    cmin = y0;cmax = y1;
                }

                double getHeading(double s) override
                {
                    // return lane_->centerHeading_fct(limitS(s));
                    auto n = lane_->centerNormal_fct(limitS(s));
                    // return std::atan2(n(1),n(0))-M_PI*0.5;
                    return (std::atan2)(-n(0),n(1));
                }

                double getCurvature(double s, int derivative) override
                {
                    if (derivative <= 0)
                    {
                        return lane_->centerSmoothedCurvature_fct(limitS(s));
                    }
                    else if (derivative <= 1)
                    {
                        return lane_->centerSmoothedCurvatureDerivative_fct(limitS(s));
                    }
                    else
                    {
                        return 0.0;
                    }
                }

                double getOffsetOfLeftBorder(double s) override
                {
                    auto x = limitS(s);
                    // std::cout << " bound value: " << x << std::flush;
                    return lane_->leftDistance_fct(x);
                }

                double getOffsetOfRightBorder(double s) override
                {
                    return lane_->rightDistance_fct(limitS(s));
                }

                void toRelativeCoordinates(double xe, double ye, double& s, double& n) override
                {
                    if(!adore::mad::toRelativeWithNormalExtrapolation(xe,ye,&lane_->centerSmoothed_fct,&lane_->centerNormal_fct,s,n))
                    {
                        s = lane_->centerSmoothed_fct.getClosestParameter(xe, ye, 1, 2, n);
                    }
                    s -= lane_->centerSmoothed_fct.limitLo();
                }

                void toEucledianCoordinates(double s, double n, double& xe, double& ye, double& ze) override
                {
                    s = limitS(s);
                    adore::mad::fromRelative(s,n,&lane_->centerSmoothed_fct,&lane_->centerNormal_fct,xe,ye,ze);
                    ze=0.0;
                }
            };
        }  // namespace BorderBased
    }      // namespace env
}  // namespace adore
