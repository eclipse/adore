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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/view/aconflictpoint.h>
#include <adore/env/afactory.h>
#include <adore/view/alane.h>

namespace adore
{
    namespace env
    {
        class DecoupledConflictPointView: public adore::view::AConflictPointSet
        {
            private:
            std::vector<adore::view::ConflictPoint> conflict_set_;
            adore::env::AFactory::TOCPredictionSetReader* conflict_set_reader_;
            adore::view::ALane* lane_;
            public:
            DecoupledConflictPointView(adore::view::ALane* lane)
            {
                lane_ = lane;
                conflict_set_reader_ = adore::env::EnvFactoryInstance::get()->getConflictSetReader();
            }
            void update()
            {
                if(conflict_set_reader_->hasUpdate())
                {
                    adore::env::OccupancyCylinderPredictionSet set;
                    conflict_set_reader_->getData(set);
                    conflict_set_.clear();
                    for(const auto& prediction: set)
                    {                     
                        for(const auto cylinder: prediction.occupancy_.getLevel(0))
                        {
                            double x,y,s,n;
                            x = cylinder.second.x_;
                            y = cylinder.second.y_;
                            lane_->toRelativeCoordinates(x,y,s,n);
                            conflict_set_.push_back(adore::view::ConflictPoint(s,cylinder.second.t0_,cylinder.second.t1_));
                        } 
                    }
                }
            }
            int size() const override
            {
                return conflict_set_.size();
            }
            adore::view::ConflictPoint getPoint(int i)const override
            {
                return conflict_set_[i];
            }            
        };

    }
}