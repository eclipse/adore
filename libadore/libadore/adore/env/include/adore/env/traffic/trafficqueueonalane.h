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

#include <adore/view/alane.h>
#include <adore/env/traffic/participant.h>
#include <algorithm>
namespace adore
{
    namespace env
    {
        namespace traffic
        {
            /**
             * LaneTraffic - computes a traffic queue for an ALane object
             */
            class TrafficQueueOnALane
            {
            private:
                adore::view::TrafficQueue queue_;
            public:
                const adore::view::TrafficQueue& getQueue()const
                {
                    return queue_;
                }
                /**
                 * @brief Map traffic unto lane
                 */
                void mapVehicles(adore::view::ALane* lane,adore::env::traffic::TParticipantSet* participantSet)
                {
                    queue_.clear();
                    if(lane->isValid())
                    {
                        for( auto& par: *participantSet )
                        {
                            //@TODO: apply prediction/classification module to determine whether participant is moving
                            // with traffic flow or should be rated as cross traffic. Also determine potential exit time
                            // if participant is changing lanes, enter a prediction of exit time
                            // check neighboring lanes for entering vehicles
                            auto point = par.center_;
                            double s,n;
                            lane->toRelativeCoordinates(point(0),point(1),s,n);
                            bool inside =    lane->getOffsetOfRightBorder(s)<=n
                                          && n<=lane->getOffsetOfLeftBorder(s);
                            //classification, whether an object is matched to lane:
                            //based on center of object
                            // + vehicles in head-on adjacent lane might slightly stick into ego lane (as observed by sensor). 
                            //   using only center creates buffer and prevents false positives
                            // - late reaction, when vehicles in same direction parallel lanes execute a lane change

                            if(!inside)continue;

                            adore::view::TrafficObject object;
                            object.setEntranceProgress(s);
                            object.setEntranceTime(par.observation_time_);

                            double cpsi = (std::cos)(par.yaw_);
                            double spsi = (std::sin)(par.yaw_);
                            double ctheta = (std::cos)(lane->getHeading(s));
                            double stheta = (std::sin)(lane->getHeading(s));
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
