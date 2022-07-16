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
 *    Daniel Heß - initial implementation and API
 ********************************************************************************/
#pragma once

#include <adore/env/traffic/lanechangegaps.h>
#include <adore/env/afactory.h>
#include <adore/env/threelaneviewdecoupled.h>

namespace adore
{
namespace apps
{
    /**
     * GapProvider computes and publishes gaps in traffic on left and right lane.
     * Gap information includes a rating wrt to ego vehicle.
     */
    class GapProvider
    {
        private:
        adore::env::ThreeLaneViewDecoupled laneView_;
        adore::env::LaneChangeGaps gapsLeft_;
        adore::env::LaneChangeGaps gapsRight_;
        adore::env::AFactory::TVehicleMotionStateReader* egoStateReader_;
        adore::env::AFactory::TGapQueueWriter* gapsLeftWriter_;
        adore::env::AFactory::TGapQueueWriter* gapsRightWriter_;
        
        public:
        GapProvider():
            laneView_(true),
            gapsLeft_(laneView_.getLeftLaneChange()),
            gapsRight_(laneView_.getRightLaneChange())
        {
            egoStateReader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
            gapsLeftWriter_ = adore::env::EnvFactoryInstance::get()->getGapQueueWriterLeftLane();
            gapsRightWriter_ = adore::env::EnvFactoryInstance::get()->getGapQueueWriterRightLane();
        }
        void update()
        {
            adore::env::VehicleMotionState9d x;
            egoStateReader_->getData(x);
            laneView_.update();
            gapsLeft_.computeGaps(laneView_.getLeftLaneChange()->getTargetLane()->getOnLaneTraffic(),x);
            gapsRight_.computeGaps(laneView_.getRightLaneChange()->getTargetLane()->getOnLaneTraffic(),x);
            std::cout<<"left: "<<gapsLeft_.getData().size()<<std::endl;
            std::cout<<"right: "<<gapsRight_.getData().size()<<std::endl;
            gapsLeftWriter_->write(gapsLeft_.getData());
            gapsRightWriter_->write(gapsRight_.getData());
        }
    };
}
}

