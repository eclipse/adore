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
 *    Thomas Lobig - initial implementation and API
 ********************************************************************************/
#pragma once
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
// #include <adore/fun/afactory.h>
#include <adore/env/traffic/occupancycylinderprediction.h>
#include <adore/env/traffic/ocroadbasedprediction.h>
#include <adore/env/traffic/trafficmap.h>
#include <adore/env/borderbased/localroadmap.h>

namespace adore
{
    namespace apps
    {
        using EnvFactory = adore::env::EnvFactoryInstance;
        using ParamsFactory = adore::params::ParamsFactoryInstance;
        // using EnvFactory = adore::env::EnvFactoryInstance;
        class PredictionProvider
        {
            private:
                adore::env::AFactory::TParticipantSetReader* tpsetReader_;
                adore::env::traffic::TParticipantSet participantSet_;
                adore::env::OccupancyCylinderPredictionSet predictionSet_;
                adore::env::AFactory::TOCPredictionSetWriter* prediction_writer_;
                adore::env::OCStraightLinePrediction predictor_straight_;
                adore::env::OCRoadBasedPrediction predictor_roadbased_;
                adore::env::BorderBased::LocalRoadMap roadmap_;
                adore::env::traffic::TrafficMap traffic_map_;
                adore::params::APPrediction* pprediction_;
            public:
            PredictionProvider() : roadmap_(EnvFactory::get(),ParamsFactory::get()), traffic_map_(roadmap_.getBorderSet(),EnvFactory::get()), predictor_roadbased_(&traffic_map_)
            {
                tpsetReader_ = EnvFactory::get()->getTrafficParticipantSetReader();
                prediction_writer_ = EnvFactory::get()->getExpectedRawPredictionSetWriter();
                pprediction_ = ParamsFactory::get()->getPrediction();
            }
            void run()
            {
                roadmap_.update();
                traffic_map_.update();


                predictor_roadbased_.setVMax(pprediction_->get_roadbased_expected_vel_ub());
                predictor_roadbased_.setAMax(pprediction_->get_roadbased_expected_acc_ub());
                predictor_roadbased_.setAMin(pprediction_->get_roadbased_expected_acc_lb());
                predictor_roadbased_.setAngleErrorMax(pprediction_->get_roadbased_heading_deviation_ub());
                predictor_roadbased_.setLatPrecision(pprediction_->get_roadbased_lat_precision());
                predictor_roadbased_.setLatError(pprediction_->get_roadbased_lat_error());
                predictor_roadbased_.setLonError(pprediction_->get_roadbased_lon_error());
                predictor_roadbased_.setTimeHeadway(pprediction_->get_roadbased_time_headway());
                predictor_roadbased_.setTimeLeeway(pprediction_->get_roadbased_time_leeway());
                predictor_straight_.setVMax(pprediction_->get_offroad_expected_vel_ub());
                predictor_straight_.setAMax(pprediction_->get_offroad_expected_acc_ub());
                predictor_straight_.setAMin(pprediction_->get_offroad_expected_acc_lb());
                predictor_straight_.setLatPrecision(pprediction_->get_offroad_lat_precision());
                predictor_straight_.setLatError(pprediction_->get_offroad_lat_error());
                predictor_straight_.setLonError(pprediction_->get_offroad_lon_error());
                predictor_straight_.setTimeHeadway(pprediction_->get_offroad_time_headway());
                predictor_straight_.setTimeLeeway(pprediction_->get_offroad_time_leeway());
                const double dt_rb = pprediction_->get_roadbased_prediction_duration();
                const double dt_or = pprediction_->get_offroad_prediction_duration();

                if(tpsetReader_->hasUpdate())
                {
                    predictionSet_.clear();
                    tpsetReader_->getData(participantSet_);
                    for(const auto& participant:participantSet_)
                    {
                        const double v = std::sqrt(participant.getVx()*participant.getVx()+participant.getVy()*participant.getVy());
                        const double area = participant.getWidth()*participant.getLength();
                        if(v<0.1||area>100.0)
                        {
                            predictor_straight_.setTMaxUTC(participant.getObservationTime() + dt_or);
                            predictor_straight_.predict(participant,predictionSet_);
                        }
                        else
                        {
                            predictor_roadbased_.setTMaxUTC(participant.getObservationTime() + dt_rb);
                            auto intermediatePredictionSetSize = predictionSet_.size();
                            predictor_roadbased_.predict(participant,predictionSet_);
                            if (!(intermediatePredictionSetSize < predictionSet_.size()))
                            {
                                predictor_straight_.setTMaxUTC(participant.getObservationTime() + dt_or);
                                predictor_straight_.predict(participant,predictionSet_);
                            }
                        }
                        
                    }                    
                    prediction_writer_->write(predictionSet_);
                }
            }            
        };

    }
}