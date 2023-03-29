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
 *   Matthias Nichting
 ********************************************************************************/

#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>
#include <adore/apps/plot_views.h>
#include <adore/apps/if_plotlab/viewplotter_config.h>
#include <plotlablib/figurestubfactory.h>

namespace adore
{
    namespace if_ROS
    {
        class PlotViewsNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
        {
          public:
            adore::apps::PlotViews* vp_;
            PlotViewsNode()
            {
            }
            void init(int argc, char** argv, double rate, std::string nodename)
            {
                Baseapp::init(argc, argv, rate, nodename);
                Baseapp::initSim();
                FactoryCollection::init(getRosNodeHandle());
                DLR_TS::PlotLab::FigureStubFactory fig_factory;
                auto figure = fig_factory.createFigureStub(2);
                figure->show();
                
                // assemble config object
                adore::PLOT::ViewPlotterConfig config;
                if (!getParam("plotoptions/viewplotter/lf_geometry_plotoptions", config.lf_geometry_plotoptions))
                {
                    config.lf_geometry_plotoptions = "LineColor=0.0,1.0,0.0;LineWidth=1.2";
                }
                if (!getParam("plotoptions/viewplotter/lc_geometry_plotoptions",
                                       config.lc_geometry_plotoptions))
                {
                    config.lc_geometry_plotoptions = "LineColor=1.0,1.0,0.0;LineWidth=1.2";
                }
                if (!getParam("plotoptions/viewplotter/lf_geometry_narrowing", config.lf_geometry_narrowing))
                {
                    config.lf_geometry_narrowing = 0.2;
                }
                if (!getParam("plotoptions/viewplotter/lc_geometry_narrowing", config.lc_geometry_narrowing))
                {
                    config.lc_geometry_narrowing = 0.2;
                }
                if (!getParam("plotoptions/viewplotter/plot_lc_geometry", config.plot_lc_geometry))
                {
                    config.plot_lc_geometry = true;
                }
                if (!getParam("plotoptions/viewplotter/plot_lf_geometry", config.plot_lf_geometry))
                {
                    config.plot_lf_geometry = true;
                }
                if (!getParam("plotoptions/viewplotter/number_of_samples_per_boundary", config.number_of_samples_per_boundary))
                {
                    config.number_of_samples_per_boundary = 50;
                }
                if (!getParam("plotoptions/viewplotter/lc_sufficient_width_plotoptions", config.lc_sufficient_width_plotoptions))
                {
                    config.lc_sufficient_width_plotoptions = "LineColor=0.0,1.0,0.0;LineWidth=1.2";
                }
                if (!getParam("plotoptions/viewplotter/lc_gate_plotoptions", config.lc_gate_plotoptions))
                {
                    config.lc_gate_plotoptions = "LineColor=0.0,1.0,0.0;LineWidth=1.2";
                }

                std::string prefix = "veh";
                int simulationID = 0;
                getParam("simulationID", simulationID);
                prefix.append(std::to_string(simulationID));
                prefix.append("views");

                vp_ = new adore::apps::PlotViews(figure, prefix, config);

                // timer callbacks
                std::function<void()> run_fcn(std::bind(&adore::apps::PlotViews::run, vp_));
                Baseapp::addTimerCallback(run_fcn);
            }
        };
    }  // namespace if_ROS
}  // namespace adore

int main(int argc, char** argv)
{
    adore::if_ROS::PlotViewsNode vpn;
    vpn.init(argc, argv, 10.0, "plot_views_node");
    vpn.run();
    return 0;
}