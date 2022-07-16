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

#include <adore/env/borderbased/lanematchingstrategy.h>
#include <adore/env/borderbased/borderset.h>
#include <adore/view/trafficobject.h>
#include <adore/view/alane.h>
#include <algorithm>
#include "trafficmap.h"

namespace adore
{
    namespace env
    {
        namespace traffic
        {
            /**
             * EgoLaneTraffic - computes a traffic queue for an ego lane / lane following view
             */
            class EgoLaneTraffic
            {
            private:
                adore::view::TrafficQueue queue_;
                TrafficMap* trafficMap_;
            public:
            /**
             * @brief Construct a new EgoLaneTraffic object
             * 
             * @param trafficMap 
             */
                EgoLaneTraffic(TrafficMap* trafficMap):trafficMap_(trafficMap){}
               /**
                * @brief Get the traffic queue
                * 
                * @return const adore::view::TrafficQueue& traffic queue of the ego lane
                */
                const adore::view::TrafficQueue& getQueue()const
                {
                    return queue_;
                }
                /**
                * @brief Get the traffic map
                * 
                * @return const CSA::VIEW::TrafficQueue& traffic queue of the ego lane
                */
                const TrafficMap* getTrafficMap()const
                {
                    return trafficMap_;
                }
                /**
                 * @brief Map vehicles on borders
                 * 
                 * @param lfv lane following view
                 * @param borders set of borders
                 */
                void mapVehiclesOnBorders(adore::view::ALane* lfv,
                                          const adore::env::BorderBased::BorderSubSet& borders)
                {
                    queue_.clear();
                    if(lfv->isValid())
                    {
                        std::unordered_set<Participant::TTrackingID> idset;
                        for( auto pborder: borders )
                        {
                            auto range = trafficMap_->getBorderToParticipant().equal_range(pborder->m_id);
                            for(auto it = range.first; it != range.second; ++it)
                            {
                                // TODO investigate if the following unused variable is actually a bug
                                // auto pos = it->second.first; // fixed Wunused-but-set-variable
                                Participant::TTrackingID id = it->second.second;
                                if(idset.find(id)!=idset.end())continue;//don't duplicate
                                auto it2 = trafficMap_->getTrackingIDToParticipant().find(id);
                                if(it2==trafficMap_->getTrackingIDToParticipant().end())continue;
                                idset.emplace(id);
                                const Participant& par = it2->second;

                                //@TODO: apply prediction/classification module to determine whether participant is moving
                                // with traffic flow or should be rated as cross traffic. Also determine potential exit time
                                // if participant is changing lanes, enter a prediction of exit time
                                // check neighboring lanes for entering vehicles
                                auto& center = par.center_;
                                double s,n;
                                lfv->toRelativeCoordinates(center(0),center(1),s,n);
                                adore::view::TrafficObject object;
                                object.setEntranceProgress(s);
                                object.setEntranceTime(par.observation_time_);
                                double cpsi = (std::cos)(par.yaw_);
                                double spsi = (std::sin)(par.yaw_);
                                double ctheta = (std::cos)(lfv->getHeading(s));
                                double stheta = (std::sin)(lfv->getHeading(s));
                                //project velocity to road
                                double ds = (ctheta*cpsi+stheta*spsi) * par.vx_ 
                                          + (-ctheta*spsi+stheta*cpsi) * par.vy_;
                                object.setObservationTime(par.observation_time_);
                                object.setEntranceSpeed(ds);
                                object.setCurrentSpeed(ds);
                                object.setCurrentProgress(s);
                                object.setCurrentAcceleration(0.0);
                                object.setExitProgress(1.0e5);
                                object.setExitTime(par.observation_time_+1.0e5);
                                object.setTrackingID(par.trackingID_);
                                object.setV2XStationID(par.v2xStationID_);
                                object.setLength(par.length_);
                                queue_.push_back(object);
                            }
                        }
                        std::sort(queue_.begin(),queue_.end(),[](const adore::view::TrafficObject& a,
                                                                 const adore::view::TrafficObject& b)
                                                                 {return a.getCurrentProgress()
                                                                        <b.getCurrentProgress();});
                    }
                }
            };
        }
    }
}
