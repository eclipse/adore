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
 *   Robert Markowski- initial implementation
 ********************************************************************************/

#pragma once
#include <adore/if_xodr/xodr2borderbased.h>
#include <adore/if_r2s/r2s2borderbased.h>
#include <adore/env/map/map_border_management.h>
#include <adore/env/map/precedence.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/mad/csvlog.h>

#include "if_plotlab/plot_border.h"

namespace adore
{
    namespace apps
    {
        /**
         * @brief base class for middleware dependent implementations of the map provider module
         * 
         * The task of the map provider is to handle map information, specifically read OpenDrive files, and provide the contained information within the adore runtime framework
         * 
         */
        class MapProvider
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
                        rot_z_   = 0;
                        rot_psi_ = 0;                    
                    }
                };
                Config config_;
                
                void setConfig(Config config)
                {
                    config_ = config;
                }
            private:
                params::APMapProvider* params_;
                adore::env::MapBorderManagement map_management_;
                adore::env::PrecedenceSet precedence_set_;
                adore::env::PrecedenceSet precedence_set_local_;
                /* read: vehicle position */
                adore::mad::AReader<adore::env::VehicleMotionState9d>* motion_state_reader_;
                /* read: border change tasks */
                adore::mad::AFeed<adore::env::BorderTypeChangeProfile>* border_type_change_feed_;

                /* write: border*/
                adore::env::AFactory::TBorderWriter* border_output_;
                /* write: precedence rule*/
                adore::env::AFactory::TPrecedenceRuleWriter* precedence_output_;

                /* plot */
                DLR_TS::PlotLab::FigureStubFactory * figure_factory_;
                DLR_TS::PlotLab::FigureStubZMQ * figure_;

                unsigned int subscriber_count_;

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
                            converter.sampling.numberOfPointsPerBorder = params_->getXODRLoaderPointsPerBorder();
                            try
                            {
                                LOG_I("Processing file %s ...", xodrFilename.c_str());
                                converter.convert(xodrFilename.c_str(),&partialSet,transform);
                                LOG_I("Done.");
                            } 
                            catch(const std::exception& e) 
                            {
                                LOG_E("Could not parse file %s", xodrFilename.c_str());
                                std::cout << e.what() << '\n';
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
                MapProvider(        std::string trackConfigs,
                                    adore::env::PrecedenceSet* pset,
                                    Config config)
                {
                    config_ = config;
                    /* process trackConfigs parameter, multiple paths to maps delimited by semicolon, comma separated additional configuration */
                    adore::env::BorderBased::BorderSet globalSet;
                    params_ = adore::params::ParamsFactoryInstance::get()->getMapProvider();
                    parseTrackConfigs(trackConfigs,globalSet);
                    globalSet.rotate(config_.rot_psi_, config_.rot_x_, config_.rot_y_);
                    globalSet.translate(config_.trans_x_, config_.trans_y_, config_.trans_z_);
                    border_output_ = adore::env::EnvFactoryInstance::get()->getBorderWriter();
                    motion_state_reader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
                    border_type_change_feed_ = adore::env::EnvFactoryInstance::get()->getBorderTypeChangeProfileFeed();
                    precedence_output_ = adore::env::EnvFactoryInstance::get()->getPrecedenceRuleWriter();
                    map_management_.init(&globalSet);
                    precedence_set_.init(pset);
                    subscriber_count_ = 0;

                    if(params_->getActivatePlotting())
                    {
                        figure_factory_ = new DLR_TS::PlotLab::FigureStubFactory();
                        figure_ = (DLR_TS::PlotLab::FigureStubZMQ*) figure_factory_->createFigureStub(1);
                        figure_->show();
                        PLOT::plotBorderSet(*(map_management_.getGlobalMap()),figure_);
                    }
                }
                /**
                 * @brief update function
                 * 
                 */
                virtual void run()
                {
                    adore::env::VehicleMotionState9d motion_state;
                    env::BorderBased::BorderSubSet newBorders;
                    std::vector<env::BorderBased::BorderID> outdatedBorders;
                    std::vector<env::BorderBased::BorderID> updatedBorders;
                    
                    motion_state_reader_->getData(motion_state);

                    if (subscriber_count_ != border_output_->getNumberOfSubscribers())
                    {
                        subscriber_count_ = border_output_->getNumberOfSubscribers();
                        map_management_.reset();
                    }

                    while(border_type_change_feed_->hasNext())
                    {
                        LOG_E("Received Message.");
                        adore::env::BorderTypeChangeProfile btcp;
                        border_type_change_feed_->getNext(btcp);
                        map_management_.changeBorderType(btcp);
                    }
                    
                    map_management_.run(motion_state.getX(),motion_state.getY(),params_->getVisibiltyRadius(),newBorders,outdatedBorders,updatedBorders,500);
                    
                    for(auto b = newBorders.begin(); b!=newBorders.end(); b++)
                    {
                        border_output_->write(**b);
                    }
                    for(auto b = updatedBorders.begin(); b!=updatedBorders.end(); b++)
                    {
                        border_output_->write(*map_management_.getBorder(*b));
                    }

                    const double x0 = motion_state.getX()-params_->getVisibiltyRadius();
                    const double y0 = motion_state.getY()-params_->getVisibiltyRadius();
                    const double x1 = motion_state.getX()+params_->getVisibiltyRadius();
                    const double y1 = motion_state.getY()+params_->getVisibiltyRadius();
                    for(auto it = precedence_set_.getRulesInRegion(x0,y0,x1,y1);
                            it.current()!=it.end();
                            it.current()++)
                    {
                        auto rule = it.current()->second;
                        if(!precedence_set_local_.contains(*rule))
                        {
                            precedence_output_->write(*rule);
                            precedence_set_local_.insertRule(*rule);
                        }
                    }
                    precedence_set_local_.refocus(x0,y0,x1,y1);
                }

        };
    }
}
