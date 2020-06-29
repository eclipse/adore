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
 *   Robert Markowski - initial implementation
 ********************************************************************************/

#pragma once
#include <adore/params/afactory.h>
#include <adore/env/map/navigation_management.h>
#include <adore/env/afactory.h>

#include "if_plotlab/plot.h"
// #include <iostream>

namespace adore
{
    namespace apps
    {
        class Navigation
        {
            private:
                params::APMapProvider* map_params_;
                params::APNavigation* nav_params_;
                adore::env::NavigationManagement nav_management_;
                
                /* read: vehicle position */
                adore::mad::AReader<adore::env::VehicleMotionState9d> * motion_state_reader_;

                /* read: navigation goal*/
                adore::mad::AReader<adore::fun::NavigationGoal> * nav_goal_reader_;

                /* write: border*/
                adore::mad::AWriter<std::pair<adore::env::BorderBased::BorderID,double>>* nav_writer_;

                DLR_TS::PlotLab::FigureStubFactory * figure_factory_;
                DLR_TS::PlotLab::FigureStubZMQ * figure_;

                void plotGlobalNavigation()
                {
                    figure_->show();
                    double maxcost = nav_management_.getMaxCost();
                    auto global_map = nav_management_.getGlobalMap();
                    auto itpair = global_map->getAllBorders();
                    for(auto it = itpair.first; it!= itpair.second; it++)
                    {
                        auto b = it->second;
                        double normedCost = nav_management_.getBorderCost(b->m_id)/maxcost;
                        adore::env::BorderBased::Border * l = nullptr;
                        if(b->m_left != nullptr)
                        {
                            l = global_map->getBorder(*(b->m_left));
                        }
                        PLOT::plotBorderNavigation(b,l,normedCost,figure_);
                    }
                }

            public:
                Navigation(env::AFactory* env_factory,adore::params::AFactory* params_factory, adore::env::BorderBased::BorderSet * set)
                {
                    figure_factory_ = new DLR_TS::PlotLab::FigureStubFactory();
                    figure_ = (DLR_TS::PlotLab::FigureStubZMQ*) figure_factory_->createFigureStub(1);

                    map_params_ = params_factory->getMapProvider();
                    nav_params_ = params_factory->getNavigation();
                    nav_goal_reader_ = env_factory->getNavigationGoalReader(); // add to nav_management_?
                    motion_state_reader_ = env_factory->getVehicleMotionStateReader();
                    nav_management_.addFeed(env_factory->getBorderFeed());
                    nav_management_.init(set); 
                    nav_writer_ = env_factory->getNavigationDataWriter();
                }
                virtual void run()
                {
                  adore::env::VehicleMotionState9d motion_state;
                  env::BorderBased::BorderSubSet data;
                  std::vector<env::BorderBased::BorderID> outdated_data;
                  adore::fun::NavigationGoal goal;

                  motion_state_reader_->getData(motion_state);
                  nav_goal_reader_->getData(goal);
                  
                  // update goal
                  nav_management_.update(adore::env::BorderBased::Coordinate(goal.target_.x_,goal.target_.y_,goal.target_.z_),true);

                  // determine which borders require updates
                  nav_management_.run(motion_state.getX(),motion_state.getY(),map_params_->getVisibiltyRadius(),data,outdated_data,500);
                  
                  //plotting
                  if(nav_management_.goalChanged() && nav_params_->getActivePlottingGlobal())
                  {
                      plotGlobalNavigation();
                  }

                  // send navigation updates
                  for(auto it = data.begin(); it!=data.end(); it++)
                  {
                    auto bId = (*it)->m_id;
                    double cost = nav_management_.getBorderCost(bId);
                    nav_writer_->write(std::make_pair(bId,cost));
                  }
                }

        };
    }
}