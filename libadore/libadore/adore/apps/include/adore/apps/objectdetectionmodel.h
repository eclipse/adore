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
#include <adore/env/afactory.h>
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
                adoreMatrix<double,3,1> ego_location_;/** < ego location for range filtering */
                adore::sim::AFactory::TParticipantFeed* participant_feed_;/** < retrieve state updates from all vehicles*/
                adore::sim::AFactory::TParticipantSetWriter* participant_set_writer_;/** < publishes list of traffic participant detections*/             
                std::unordered_map<int,adore::env::traffic::Participant> latest_data_;/** < map contains latest updates on traffic participants, tracking id mapping to participant*/
                adore::mad::AReader<double>* timer_;/** < timer is used for discarding old updates */
                adore::params::APSensorModel* psensor_model_;
                adore::env::AFactory::TVehicleMotionStateReader* localization_state_reader_;
                adore::sim::AFactory::TVehicleMotionStateReader* true_state_reader_;

            public:
                /**
                 * Constructor
                 *  
                 * @param simulationID id of vehicle in simulation, required to avoid detecting itself
                 */
                ObjectDetectionModel(int simulationID)
                {
                    simulationID_ = simulationID;
                    participant_feed_ = adore::sim::SimFactoryInstance::get()->getParticipantFeed();
                    participant_set_writer_ = adore::sim::SimFactoryInstance::get()->getParticipantSetWriter();
                    timer_ = adore::sim::SimFactoryInstance::get()->getSimulationTimeReader();
                    psensor_model_ = adore::params::ParamsFactoryInstance::get()->getSensorModel();
                    localization_state_reader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
                    true_state_reader_ = adore::sim::SimFactoryInstance::get()->getVehicleMotionStateReader();
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

                    //update parameters
                    double sensor_range;/** < range at which traffic is detected */
                    double discard_age;/** < time after which observations are discarded */
                    sensor_range = psensor_model_->get_objectDetectionRange();
                    discard_age = psensor_model_->get_objectDiscardAge();

                    adore::env::VehicleMotionState9d x_localization;
                    localization_state_reader_->getData(x_localization);

                    adore::fun::VehicleMotionState9d x;
                    true_state_reader_->getData(x);

                    adoreMatrix<double, 3, 1> position;
                    // update ego position
                    position(0, 0) = x.getX();
                    position(1, 0) = x.getY();
                    position(2, 0) = 0.0;
                    ego_location_ = position;

                    //retrieve state updates
                    while( participant_feed_->hasNext() )
                    {
                        adore::env::traffic::Participant p;
                        participant_feed_->getNext(p);
                        if( p.getTrackingID() != simulationID_ )
                        {
                            if( adore::mad::norm2((adoreMatrix<double,3,1>)(p.getCenter()-ego_location_)) < sensor_range )
                            {
                                //transform the participant to local coordinates, then to localization coordinates
                                const double ct = std::cos(x.getPSI());
                                const double st = std::sin(x.getPSI());
                                const double ce = std::cos(x_localization.getPSI());
                                const double se = std::sin(x_localization.getPSI());
                                const double X_l = ct*(p.getCenter()(0)-x.getX())+st*(p.getCenter()(1)-x.getY());
                                const double Y_l =-st*(p.getCenter()(0)-x.getX())+ct*(p.getCenter()(1)-x.getY());
                                const double Z_l = p.getCenter()(2)-x.getZ();
                                const double X_e = ct*X_l-st*Y_l + x_localization.getX();
                                const double Y_e = st*X_l+ct*Y_l + x_localization.getY();
                                const double Z_e = Z_l + x_localization.getZ();
                                p.center_(0) = X_e;
                                p.center_(1) = Y_e;
                                p.center_(2) = Z_e;
                                const double cy = std::cos(p.getYaw());
                                const double sy = std::sin(p.getYaw());
                                const double cy_l = ct * cy + st * sy;
                                const double sy_l =-st * cy + ct * sy;
                                const double cy_e = ce * cy_l - se * sy_l;
                                const double sy_e = se * cy_l + ce * sy_l;
                                p.yaw_ = std::atan2(sy_e,cy_e);
                                latest_data_.emplace(p.getTrackingID(), p).first->second = p;
                            }
                        }
                        
                    }

                    //collect latest known state updates
                    adore::env::traffic::TParticipantSet pset;
                    for( auto& pair: latest_data_ )
                    {
                        auto& p = pair.second;
                        
                        if( t_now - p.getObservationTime() < discard_age )
                        {
                            p.existance_certainty_ = 100.0;
                            pset.push_back(p);
                            //std::cout<<"traffic id="<<p.getTrackingID()<<", x="<<p.getCenter()(0)<<", y="<<p.getCenter()(1)<<"\n";
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