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
#include <adore/env/map/map_border_management.h>
#include <adore/env/map/precedence.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>

#include "if_plotlab/plot.h"

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
            private:
                params::APMapProvider* params_;
                adore::env::MapBorderManagement map_management_;
                adore::env::PrecedenceSet precedence_set_;
                adore::env::PrecedenceSet precedence_set_local_;
                /* read: vehicle position */
                adore::mad::AReader<adore::env::VehicleMotionState9d>* motion_state_reader_;

                /* write: border*/
                adore::env::AFactory::TBorderWriter* border_output_;
                /* write: precedence rule*/
                adore::env::AFactory::TPrecedenceRuleWriter* precedence_output_;

                /* plot */
                DLR_TS::PlotLab::FigureStubFactory * figure_factory_;
                DLR_TS::PlotLab::FigureStubZMQ * figure_;

            public:
                MapProvider(env::AFactory* env_factory,adore::params::AFactory* params_factory, 
                                    adore::env::BorderBased::BorderSet * bset,
                                    adore::env::PrecedenceSet* pset)
                {
                    params_ = params_factory->getMapProvider();
                    border_output_ = env_factory->getBorderWriter();
                    motion_state_reader_ = env_factory->getVehicleMotionStateReader();
                    precedence_output_ = env_factory->getPrecedenceRuleWriter();
                    map_management_.init(bset);
                    precedence_set_.init(pset);

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
                    
                    motion_state_reader_->getData(motion_state);
                    
                    map_management_.run(motion_state.getX(),motion_state.getY(),params_->getVisibiltyRadius(),newBorders,outdatedBorders,500);
                    
                    for(auto b = newBorders.begin(); b!=newBorders.end(); b++)
                    {
                        border_output_->write(**b);
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