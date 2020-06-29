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
 *   Daniel Heß - initial implementation
 ********************************************************************************/

#pragma once
#include <adore/sim/afactory.h>
#include <adore/params/afactory.h>
#include <unordered_map>

namespace adore
{
    namespace apps
    {
        /**
         * A simple model for sensor detection of traffic participants in the vehicle's vicinity
         */
        class ObjectDetectionModel
        {
            private:
                int simulationID_;
                double sensor_range_;/** < range at which traffic is detected */
                double discard_age_;/** < time after which observations are discarded */
                adoreMatrix<double,3,1> ego_location_;/** < ego location for range filtering */
                adore::sim::AFactory::TParticipantFeed* participant_feed_;/** < retrieve state updates from all vehicles*/
                adore::sim::AFactory::TParticipantSetWriter* participant_set_writer_;/** < publishes list of traffic participant detections*/             
                std::unordered_map<int,adore::env::traffic::Participant> latest_data_;/** < map contains latest updates on traffic participants, tracking id mapping to participant*/
                adore::mad::AReader<double>* timer_;/** < timer is used for discarding old updates */

            public:
                /**
                 * Constructor
                 *  
                 * @param sim_factory adore::sim factory
                 * @param paramfactory adore::params factory
                 * @param simulationID id of vehicle in simulation, required to avoid detecting itself
                 */
                ObjectDetectionModel(sim::AFactory* sim_factory,params::AFactory* paramfactory,int simulationID)
                {
                    simulationID_ = simulationID;
                    sensor_range_ = 1000.0;
                    discard_age_ = 1.0;
                    participant_feed_ = sim_factory->getParticipantFeed();
                    participant_set_writer_ = sim_factory->getParticipantSetWriter();
                    timer_ = sim_factory->getSimulationTimeReader();
                }
                /**
                 * @brief publish updates on the detection of traffic participants
                 * 
                 */
                virtual void run()
                {
                    if(!timer_->hasData())return;
                    double t_now;
                    timer_->getData(t_now);

                    //retrieve state updates
                    while( participant_feed_->hasNext() )
                    {
                        adore::env::traffic::Participant p;
                        participant_feed_->getNext(p);
                        if( p.getTrackingID() == simulationID_ )
                        {
                            //update ego position
                            ego_location_ = p.getCenter();
                        }
                        else
                        {
                            //remember state update
                            if( adore::mad::norm2((adoreMatrix<double,3,1>)(p.getCenter()-ego_location_)) < sensor_range_ )
                            {
                                latest_data_.emplace(p.getTrackingID(), p).first->second = p;
                            }
                        }
                        
                    }
                    //collect latest known state updates
                    adore::env::traffic::TParticipantSet pset;
                    for( auto& pair: latest_data_ )
                    {
                        auto& p = pair.second;
                        if( t_now - p.getObservationTime() < discard_age_ )
                        {
                            pset.push_back(p);
                            // std::cout<<"traffic id="<<p.getTrackingID()<<", x="<<p.getCenter()(0)<<", y="<<p.getCenter()(1)<<"\n";
                        }
                        else
                        {
                            //TODO: do nothing or eventually discard from latest_data_
                        }
                    }
                    //send set of observations
                    participant_set_writer_->write(pset);
                }

        };
    }
}