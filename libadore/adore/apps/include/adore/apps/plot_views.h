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
 *   Matthias Nichting - initial implementation
 ********************************************************************************/

#pragma once
#include <adore/view/alane.h>
#include <adore/view/alanechangeview.h>
#include <adore/apps/if_plotlab/viewplotter_config.h>
#include <adore/apps/if_plotlab/plot_views.h>
#include <plotlablib/afigurestub.h>
#include <adore/env/borderbased/lanecombinedgeometry.h>
#include <adore/env/borderbased/lanefollowingviewproxy.h>
#include <adore/env/borderbased/lanechangeviewproxy.h>

namespace adore
{
    namespace apps
    {
        /**
         * @brief a plotting application to plot the folling: lanefollowinggeometry, lanechangegeometry
         *
         */
        class PlotViews
        {
          private:
            DLR_TS::PlotLab::AFigureStub* figure_;
            std::string prefix_;
            adore::PLOT::ViewPlotterConfig config_;
            adore::env::AFactory::TLaneGeometryFeed* laneFeed_;

          public:
            PlotViews(DLR_TS::PlotLab::AFigureStub* figure, std::string prefix,
                        const adore::PLOT::ViewPlotterConfig& config)
            {
                figure_ = figure;
                prefix_ = prefix;
                config_ = config;
                laneFeed_ = adore::env::EnvFactoryInstance::get()->getLaneGeometryFeed();
            }

            ~PlotViews()
            {
            }

            void run()
            {
                adore::env::BorderBased::CombinedLaneGeometry combinedGeometry;
                if (laneFeed_->hasNext())
                {
                    laneFeed_->getLatest(combinedGeometry);
                    if (config_.plot_lf_geometry)
                    {
                        std::string prefix = prefix_;
                        prefix.append("lfg");
                        if (combinedGeometry.center->isValid)
                        {
                            auto laneView = adore::env::BorderBased::LaneFollowingViewProxy(combinedGeometry.center);
                            adore::PLOT::plotALane(laneView, prefix, config_, figure_, "lf");
                        }
                        else
                        {
                            figure_->erase_similar(prefix);
                        }
                    }

                    if (config_.plot_lc_geometry)
                    {
                        std::string prefix = prefix_;
                        prefix.append("lcl");
                        if (combinedGeometry.leftChange->isValid)
                        {
                        auto laneChangeLeftView = adore::env::BorderBased::LaneChangeViewProxy(
                            combinedGeometry.left, combinedGeometry.left, combinedGeometry.leftChange);
                            adore::PLOT::plotALaneChangeView(laneChangeLeftView, prefix, config_, figure_);
                        }
                        else
                        {
                            figure_->erase_similar(prefix);
                        }
                    }

                    if (config_.plot_lc_geometry)
                    {
                        std::string prefix = prefix_;
                        prefix.append("lcr");
                        if (combinedGeometry.rightChange->isValid)
                        {
                            auto laneChangeRightView = adore::env::BorderBased::LaneChangeViewProxy(
                                combinedGeometry.right, combinedGeometry.right, combinedGeometry.rightChange);
                            
                            adore::PLOT::plotALaneChangeView(laneChangeRightView, prefix, config_, figure_);
                        }
                        else
                        {
                            figure_->erase_similar(prefix);
                        }
                    }

                    // if (laneChangeViewLeftReader_->hasUpdate() && config_.plot_lc_geometry)
                    // {
                    //     std::string prefix = prefix_;
                    //     prefix.append("lcl");
                    //     auto alanechangeview = new adore::view::ALaneChangeView();
                    //     laneChangeViewLeftReader_->getData(*alanechangeview);
                    //     if (alanechangeview->isValid())
                    //     {
                    //         adore::PLOT::plotALaneChangeView(*alanechangeview->getTargetLane(), prefix, config_,
                    //                                          figure_);
                    //     }
                    //     else
                    //     {
                    //         figure_->erase_similar(prefix);
                    //     }
                    // }
                }
            }
        };
    }  // namespace apps
}  // namespace adore