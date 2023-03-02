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

#include <plotlablib/figurestubfactory.h>
#include <adore/apps/plot_planning_details.h>
#include <adore_if_ros_scheduling/baseapp.h>
#include <adore_if_ros/factorycollection.h>
#include <ros/ros.h>

namespace adore
{
    namespace if_ROS
    {
        class PlotPlanningDetailsNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp
        {
          public:
            PlotPlanningDetailsNode()
            {
            }
            ~PlotPlanningDetailsNode()
            {
                delete pdp_;
            }
            adore::apps::PlanningDetailsPlotter* pdp_;
            void init(int argc, char** argv, double rate, std::string nodename)
            {
                Baseapp::init(argc, argv, rate, nodename);
                Baseapp::initSim();
                FactoryCollection::init(getRosNodeHandle());
                pdp_ = new adore::apps::PlanningDetailsPlotter();
                // timer callbacks
                std::function<void()> run_fcn(std::bind(&adore::apps::PlanningDetailsPlotter::run, pdp_));
                Baseapp::addTimerCallback(run_fcn);
            }
        };
    }  // namespace if_ROS
}  // namespace adore

adore::if_ROS::PlotPlanningDetailsNode ppdn;
bool terminated;


int main(int argc, char** argv)
{
    terminated = false;
    ppdn.init(argc, argv, 10.0, "adore_plot_planning_details_node");
    ppdn.run();
    terminated = true;
    return 0;
}