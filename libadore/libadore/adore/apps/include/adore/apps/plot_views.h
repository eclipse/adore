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
#include <adore/apps/if_plotlab/viewplotter_config.h>
#include <adore/apps/if_plotlab/plot_views.h>
#include <plotlablib/afigurestub.h>
#include <adore/env/threelaneviewdecoupled.h>

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
            env::ThreeLaneViewDecoupled tlvd_;

          public:
            PlotViews(DLR_TS::PlotLab::AFigureStub* figure, std::string prefix,
                      const adore::PLOT::ViewPlotterConfig& config)
              : figure_(figure), prefix_(prefix), config_(config)
            {
                figure_->erase_similar(prefix_);  // erase plots made by another instance of ViewPlotter
            }

            ~PlotViews()
            {
            }

            void run()
            {
                tlvd_.update();
                if (tlvd_.getCurrentLane()->isValid())
                {
                    if (config_.plot_lf_geometry)
                    {
                        std::string prefix = prefix_;
                        prefix.append("lfg");
                        adore::PLOT::plotALane(tlvd_.getCurrentLane(), prefix, config_, figure_, "lf");
                    }
                    
                    if (config_.plot_lc_geometry)
                    {
                        std::string prefix = prefix_;
                        prefix.append("lcl");
                        if (tlvd_.getLeftLaneChange()->getTargetLane()->isValid())
                        {
                            adore::PLOT::plotALaneChangeView(tlvd_.getLeftLaneChange(), prefix, config_, figure_);
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
                        if (tlvd_.getRightLaneChange()->getTargetLane()->isValid())
                        {
                            adore::PLOT::plotALaneChangeView(tlvd_.getRightLaneChange(), prefix, config_, figure_);
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
                else
                {
                    figure_->erase_similar(prefix_);
                }
            }
        };
    }  // namespace apps
}  // namespace adore
