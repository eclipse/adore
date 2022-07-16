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
 * Matthias Nichting
 *
 ********************************************************************************/

#pragma once
#include <plotlablib/figurestubfactory.h>
#include <adore/fun/afactory.h>
#include <adore/mad/adoremath.h>

namespace adore
{
    namespace apps
    {
        /**
         * @brief test bench implementation for minimum risk maneuver planner
         *
         */
        class PlanningDetailsPlotter
        {
          private:
            bool plot_longitudinal_;
            bool plot_lateral_;
            double s_upper_bound_clipped_;
            double s_lower_bound_clipped_;
            double n_upper_bound_clipped_;
            double n_lower_bound_clipped_;
            DLR_TS::PlotLab::FigureStubFactory* fig_factory_;
            DLR_TS::PlotLab::AFigureStub* figure_lon1_;
            DLR_TS::PlotLab::AFigureStub* figure_lon2_;
            DLR_TS::PlotLab::AFigureStub* figure_lat1_;
            DLR_TS::PlotLab::AFigureStub* figure_lat2_;
            adore::mad::AFeed<adore::fun::PlanningResult>* planning_result_feed_;

          public:
            PlanningDetailsPlotter()
              : plot_longitudinal_(true)
              , plot_lateral_(true)
              , s_upper_bound_clipped_(200)
              , s_lower_bound_clipped_(-10)
              , n_upper_bound_clipped_(5)
              , n_lower_bound_clipped_(-5)
            {
                planning_result_feed_ = adore::fun::FunFactoryInstance::get()->getPlanningResultFeed();
                initialize_plot();
            }
            ~PlanningDetailsPlotter()
            {
                delete planning_result_feed_;
                terminate_plot();
            }
            void initialize_plot()
            {
                fig_factory_ = new DLR_TS::PlotLab::FigureStubFactory();
                figure_lon1_ = fig_factory_->createFigureStub(3);
                figure_lon1_->setTitle("Longitudinal Plan - s");
                figure_lon1_->setXLabel("t (s)");
                figure_lon1_->setYLabel("s (m)");
                figure_lon1_->showAxis();
                figure_lon1_->showGrid();
                figure_lon1_->show();
                figure_lon2_ = fig_factory_->createFigureStub(4);
                figure_lon2_->setTitle("Longitudinal Plan - ds(b),dds(g),ddds(r)");
                figure_lon2_->setXLabel("t (s)");
                figure_lon2_->setYLabel("ds (m/s)");
                figure_lon2_->showAxis();
                figure_lon2_->showGrid();
                figure_lon2_->show();
                figure_lat1_ = fig_factory_->createFigureStub(5);
                figure_lat1_->setTitle("Lateral Plan - n");
                figure_lat1_->setXLabel("t (s)");
                figure_lat1_->setYLabel("n (m)");
                figure_lat1_->show();
                figure_lat1_->showAxis();
                figure_lat1_->showGrid();
                figure_lat2_ = fig_factory_->createFigureStub(6);
                figure_lat2_->setTitle("Lateral Plan - dn(b),ddn(r),dddn(g)");
                figure_lat2_->setXLabel("t (s)");
                figure_lat2_->setYLabel("dn (m/s)");
                figure_lat2_->show();
                figure_lat2_->showAxis();
                figure_lat2_->showGrid();
            }
            void terminate_plot()
            {
                delete fig_factory_;
                delete figure_lon1_;
                delete figure_lon2_;
                delete figure_lat1_;
                delete figure_lat2_;
            }
            void plot_longitudinal_planning_information(fun::PlanningResult& planning_result)
            {
                if(!planning_result.nominal_maneuver_valid)return;
                auto& longitudinal_plan = planning_result.nominal_maneuver_longitudinal_plan.getData();
                const int N = longitudinal_plan.nc();
                double XN[N];
                double YN[N];
                const std::string& name= planning_result.name;
                adore::mad::copyRowToArray(longitudinal_plan, XN, 0);  // t
                adore::mad::copyRowToArray(longitudinal_plan, YN, 1);  // s
                figure_lon1_->plot(name+"/s", XN, YN, N, "LineColor=0,0,1");
                adore::mad::copyRowToArray(longitudinal_plan, YN, 2);  // ds
                figure_lon2_->plot(name+"/ds", XN, YN, N, "LineColor=0,0,1");
                adore::mad::copyRowToArray(longitudinal_plan, YN, 3);  // dds
                figure_lon2_->plot(name+"/dds", XN, YN, N, "LineColor=1,0,0");
                adore::mad::copyRowToArray(longitudinal_plan, YN, 4);  // ddds
                figure_lon2_->plot(name+"/ddds", XN, YN, N, "LineColor=0,0.5,0");

                auto& longitudinal_lbx = planning_result.nominal_maneuver_longitudinal_lbx.getData();
                auto& longitudinal_ubx = planning_result.nominal_maneuver_longitudinal_ubx.getData();
                const int n_lon_b = longitudinal_lbx.nc();
                if (n_lon_b > 0)
                {
                    const int nr_lon_b = longitudinal_lbx.nr();
                    double X_lon_b[n_lon_b];
                    double Y_lon_b[n_lon_b];
                    if (nr_lon_b > 1)
                    {
                        adore::mad::copyRowToArray(longitudinal_lbx, X_lon_b, 0);  // t
                        adore::mad::copyRowToArray(longitudinal_lbx, Y_lon_b, 1);  // s
                        for (int i = 0; i < n_lon_b; ++i)
                        {
                            if (Y_lon_b[i] < s_lower_bound_clipped_)
                                Y_lon_b[i] = s_lower_bound_clipped_;
                        }
                        figure_lon1_->plot(name+"/slb", X_lon_b, Y_lon_b, n_lon_b, "LineColor=1,0,0");
                        adore::mad::copyRowToArray(longitudinal_ubx, Y_lon_b, 1);  // s
                        for (int i = 0; i < n_lon_b; ++i)
                        {
                            if (Y_lon_b[i] > s_upper_bound_clipped_)
                                Y_lon_b[i] = s_upper_bound_clipped_;
                        }
                        figure_lon1_->plot(name+"/sub", X_lon_b, Y_lon_b, n_lon_b, "LineColor=1,0,0");
                    }
                    if (nr_lon_b > 2)
                    {
                        adore::mad::copyRowToArray(longitudinal_lbx, Y_lon_b, 2);  // ds
                        figure_lon2_->plot(name+"/dslb", X_lon_b, Y_lon_b, n_lon_b, "LineColor=1,0,0");
                        adore::mad::copyRowToArray(longitudinal_ubx, Y_lon_b, 2);  // ds
                        figure_lon2_->plot(name+"/dsub", X_lon_b, Y_lon_b, n_lon_b, "LineColor=1,0,0");
                    }
                }
            }
            void plot_lateral_planning_information(fun::PlanningResult& planning_result)
            {
                if(!planning_result.nominal_maneuver_valid)return;
                auto& lateral_plan = planning_result.nominal_maneuver_lateral_plan.getData();
                const int N = lateral_plan.nc();
                double XN[N];
                double YN[N];
                const std::string& name= planning_result.name;
                adore::mad::copyRowToArray(lateral_plan, XN, 0);  // t
                adore::mad::copyRowToArray(lateral_plan, YN, 1);  // s
                figure_lat1_->plot(name+"/n", XN, YN, N, "LineColor=0,0,1");

                auto& lateral_lbx = planning_result.nominal_maneuver_lateral_lbx.getData();
                auto& lateral_ubx = planning_result.nominal_maneuver_lateral_ubx.getData();
                const int n_lat_b = lateral_lbx.nc();
                if (n_lat_b > 0)
                {
                    const int nr_lat_b = lateral_lbx.nr();
                    double X_lat_b[n_lat_b];
                    double Y_lat_b[n_lat_b];
                    if (nr_lat_b > 1)
                    {
                        adore::mad::copyRowToArray(lateral_lbx, X_lat_b, 0);  // t
                        adore::mad::copyRowToArray(lateral_lbx, Y_lat_b, 1);  // s
                        for (int i = 0; i < n_lat_b; ++i)
                        {
                            if (Y_lat_b[i] < n_lower_bound_clipped_)
                                Y_lat_b[i] = n_lower_bound_clipped_;
                        }
                        figure_lat1_->plot(name+"/nlb", X_lat_b, Y_lat_b, n_lat_b, "LineColor=1,0,0");
                        adore::mad::copyRowToArray(lateral_ubx, Y_lat_b, 1);  // n
                        for (int i = 0; i < n_lat_b; ++i)
                        {
                            if (Y_lat_b[i] > n_upper_bound_clipped_)
                                Y_lat_b[i] = n_upper_bound_clipped_;
                        }

                        figure_lat1_->plot(name+"/nub", X_lat_b, Y_lat_b, n_lat_b, "LineColor=1,0,0");
                    }
                }
            }
            void run()
            {
                fun::PlanningResult latest_planning_result;
                if (planning_result_feed_->hasNext())
                {
                    planning_result_feed_->getLatest(latest_planning_result);
                    if (latest_planning_result.nominal_maneuver_valid && plot_longitudinal_)
                    {
                        plot_longitudinal_planning_information(latest_planning_result);
                    }
                    if (latest_planning_result.nominal_maneuver_valid && plot_lateral_)
                    {
                        plot_lateral_planning_information(latest_planning_result);
                    }
                }
            }
        };
    }  // namespace apps
}  // namespace adore