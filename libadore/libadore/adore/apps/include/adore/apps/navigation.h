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
#include <adore/if_xodr/xodr2borderbased.h>
#include <adore/if_r2s/r2s2borderbased.h>
#include <adore/mad/csvlog.h>

#include "if_plotlab/plot_shape.h"
#include "if_plotlab/plot_border.h"
// #include <iostream>

namespace adore
{
    namespace apps
    {
        class Navigation
        {
            public:
                struct Config
                {
                    double trans_x_, trans_y_, trans_z_, rot_x_, rot_y_, rot_z_, rot_psi_;
                    Config()
                    {
                        trans_x_ = 0;
                        trans_y_ = 0;
                        trans_z_ = 0;
                        rot_x_   = 0;
                        rot_y_   = 0;
                        rot_psi_ = 0;                    
                    }
                };
                Config config_;
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
                void parseTrackConfigs(std::string trackConfigs, env::BorderBased::BorderSet& targetSet)
                {
                    std::string trackConfig = "";
                    std::stringstream trackstream(trackConfigs);
                    while(std::getline(trackstream,trackConfig,';'))
                    {
                        /* reading of single track configuration, comma separated */
                        std::stringstream trackConfigStream(trackConfig);
                        bool transform = false;
                        std::string xodrFilename = "";
                        std::string r2sReflineFilename = "";
                        std::string r2sLaneFilename = "";
                        std::string token = "";
                        while(std::getline(trackConfigStream,token,','))
                        {
                            if(token.size()<=5)
                            {
                                LOG_W("Unrecognizable token: %s", token.c_str());
                                continue;
                            }
                            if(token.compare("transform")==0)
                            {
                                transform = true;
                            }
                            else if(token.substr(token.size()-5,5).compare(".xodr")==0)
                            {
                                xodrFilename = token;
                            }
                            else
                            {
                                if(token.substr(token.size()-5,5).compare(".r2sr")==0)
                                {
                                    r2sReflineFilename = token;
                                }
                                else if(token.substr(token.size()-5,5).compare(".r2sl")==0)
                                {
                                    r2sLaneFilename = token;
                                }
                                else
                                {
                                    LOG_W("Unrecognizable token: %s", token.c_str());
                                    continue;
                                }
                            }
                            
                        }
                        /* process current file */
                        adore::env::BorderBased::BorderSet partialSet;
                        if(!xodrFilename.empty())
                        {
                            adore::if_xodr::XODR2BorderBasedConverter converter;
                            converter.sampling.numberOfPointsPerBorder = map_params_->getXODRLoaderPointsPerBorder();
                            try
                            {
                                LOG_I("Processing file %s ...", xodrFilename.c_str());
                                converter.convert(xodrFilename.c_str(),&partialSet,transform);
                                LOG_I("Done.");
                            }
                            catch(...)
                            {
                                LOG_E("Could not parse file %s", xodrFilename.c_str());
                            }
                            /* add partial map to global map */
                            auto its = partialSet.getAllBorders();
                            for(;its.first!=its.second;its.first++)
                            {
                                targetSet.insert_border(its.first->second);
                            }
                            /* global map has responsibility for object/pointers */
                            partialSet.setIsOwner(false);
                        }
                        else if(!(r2sReflineFilename.empty() || r2sLaneFilename.empty()))
                        {
                            try
                            {
                                LOG_I("Processing files %s and %s ...", r2sReflineFilename.c_str(), r2sLaneFilename.c_str());
                                if_r2s::R2S2BorderBasedConverter converter;
                                converter.convert(r2sReflineFilename,r2sLaneFilename, partialSet);
                                LOG_I("Done.");
                            }
                            catch(...)
                            {
                                LOG_E("Could not parse R2S files %s and %s", r2sReflineFilename.c_str(), r2sLaneFilename.c_str());
                            }
                            
                            /* add partial map to global map */
                            auto its = partialSet.getAllBorders();
                            for(;its.first!=its.second;its.first++)
                            {
                                targetSet.insert_border(its.first->second);
                            }
                            partialSet.setIsOwner(false);
                        }
                        else
                        {
                            LOG_E("Could not parse configuration: %s", trackConfig.c_str());
                        }
                    }                    
                }

            public:
                Navigation(env::AFactory* env_factory,adore::params::AFactory* params_factory, std::string trackConfigs, Config config)
                {
                    config_ = config;
                    env::BorderBased::BorderSet set;
                    map_params_ = params_factory->getMapProvider();
                    nav_params_ = params_factory->getNavigation();
                    parseTrackConfigs(trackConfigs, set);
                    set.rotate(config_.rot_psi_, config_.rot_x_, config_.rot_y_);
                    set.translate(config_.trans_x_, config_.trans_y_, config_.trans_z_);
                    figure_factory_ = new DLR_TS::PlotLab::FigureStubFactory();
                    figure_ = (DLR_TS::PlotLab::FigureStubZMQ*) figure_factory_->createFigureStub(1);

                    nav_goal_reader_ = env_factory->getNavigationGoalReader(); // add to nav_management_?
                    motion_state_reader_ = env_factory->getVehicleMotionStateReader();
                    nav_management_.addFeed(env_factory->getBorderFeed());
                    nav_management_.init(&set); 
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
                  nav_management_.setLaneChangePenalty(nav_params_->getLaneChangePenalty());
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