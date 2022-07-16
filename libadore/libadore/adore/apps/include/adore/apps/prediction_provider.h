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

                adore::env::BorderBased::LocalRoadMap roadmap_;
                adore::env::traffic::TrafficMap traffic_map_;
                adore::params::APPrediction* pprediction_;

                adore::env::OccupancyCylinderPredictionSet predictionSet_ex_;/**<prediction set for expected behavior*/
                adore::env::AFactory::TOCPredictionSetWriter* prediction_writer_ex_;/**<prediction writer for expected behavior*/
                adore::env::OCStraightLinePrediction predictor_straight_ex_;/**<straight line predictor for expected behavior*/
                adore::env::OCRoadBasedPrediction predictor_roadbased_ex_;/**<roadbased predictor for expected behavior*/

                adore::env::OccupancyCylinderPredictionSet predictionSet_wc_;/**<prediction set for worst case behavior*/
                adore::env::AFactory::TOCPredictionSetWriter* prediction_writer_wc_;/**<prediction writer for worst case behavior*/
                adore::env::OCStraightLinePrediction predictor_straight_wc_;/**<straight line predictor for worst case behavior*/
                adore::env::OCRoadBasedPrediction predictor_roadbased_wc_;/**<roadbased predictor for worst case behavior*/

            public:
            PredictionProvider() : roadmap_(EnvFactory::get(),ParamsFactory::get()), traffic_map_(roadmap_.getBorderSet(),EnvFactory::get()), 
                                    predictor_roadbased_ex_(&traffic_map_), predictor_roadbased_wc_(&traffic_map_)
            {
                tpsetReader_ = EnvFactory::get()->getTrafficParticipantSetReader();
                prediction_writer_ex_ = EnvFactory::get()->getExpectedRawPredictionSetWriter();
                prediction_writer_wc_ = EnvFactory::get()->getWorstCaseRawPredictionSetWriter();
                pprediction_ = ParamsFactory::get()->getPrediction();
            }
            void run()
            {
                roadmap_.update();
                traffic_map_.update();


                predictor_roadbased_ex_.setVMax(pprediction_->get_roadbased_expected_vel_ub());
                predictor_roadbased_ex_.setAMax(pprediction_->get_roadbased_expected_acc_ub());
                predictor_roadbased_ex_.setAMin(pprediction_->get_roadbased_expected_acc_lb());
                predictor_roadbased_ex_.setAngleErrorMax(pprediction_->get_roadbased_heading_deviation_ub());
                predictor_roadbased_ex_.setLatPrecision(pprediction_->get_roadbased_lat_precision());
                predictor_roadbased_ex_.setLatError(pprediction_->get_roadbased_lat_error());
                predictor_roadbased_ex_.setLonError(pprediction_->get_roadbased_lon_error());
                predictor_roadbased_ex_.setTimeHeadway(pprediction_->get_roadbased_time_headway());
                predictor_roadbased_ex_.setTimeLeeway(pprediction_->get_roadbased_time_leeway());
                predictor_roadbased_ex_.setDelay(pprediction_->get_roadbased_expected_acc_ub_delay());
                predictor_roadbased_ex_.setWidthUB(pprediction_->get_prediction_width_ub());
                predictor_roadbased_ex_.setWidthLB(pprediction_->get_prediction_width_lb());
                predictor_straight_ex_.setVMax(pprediction_->get_offroad_expected_vel_ub());
                predictor_straight_ex_.setAMax(pprediction_->get_offroad_expected_acc_ub());
                predictor_straight_ex_.setAMin(pprediction_->get_offroad_expected_acc_lb());
                predictor_straight_ex_.setLatPrecision(pprediction_->get_offroad_lat_precision());
                predictor_straight_ex_.setLatError(pprediction_->get_offroad_lat_error());
                predictor_straight_ex_.setLonError(pprediction_->get_offroad_lon_error());
                predictor_straight_ex_.setTimeHeadway(pprediction_->get_offroad_time_headway());
                predictor_straight_ex_.setTimeLeeway(pprediction_->get_offroad_time_leeway());
                predictor_straight_ex_.setWidthUB(pprediction_->get_prediction_width_ub());
                predictor_straight_ex_.setWidthLB(pprediction_->get_prediction_width_lb());

                predictor_roadbased_wc_.setVMax(pprediction_->get_roadbased_worstcase_vel_ub());
                predictor_roadbased_wc_.setAMax(pprediction_->get_roadbased_worstcase_acc_ub());
                predictor_roadbased_wc_.setAMin(pprediction_->get_roadbased_worstcase_acc_lb());
                predictor_roadbased_wc_.setAngleErrorMax(pprediction_->get_roadbased_heading_deviation_ub());
                predictor_roadbased_wc_.setLatPrecision(pprediction_->get_roadbased_lat_precision());
                predictor_roadbased_wc_.setLatError(pprediction_->get_roadbased_lat_error());
                predictor_roadbased_wc_.setLonError(pprediction_->get_roadbased_lon_error());
                predictor_roadbased_wc_.setTimeHeadway(pprediction_->get_roadbased_time_headway());
                predictor_roadbased_wc_.setTimeLeeway(pprediction_->get_roadbased_time_leeway());
                predictor_roadbased_wc_.setDelay(pprediction_->get_roadbased_worstcase_acc_ub_delay());
                predictor_roadbased_wc_.setWidthUB(pprediction_->get_prediction_width_ub());
                predictor_roadbased_wc_.setWidthLB(pprediction_->get_prediction_width_lb());
                predictor_straight_wc_.setVMax(pprediction_->get_offroad_worstcase_vel_ub());
                predictor_straight_wc_.setAMax(pprediction_->get_offroad_worstcase_acc_ub());
                predictor_straight_wc_.setAMin(pprediction_->get_offroad_worstcase_acc_lb());
                predictor_straight_wc_.setLatPrecision(pprediction_->get_offroad_lat_precision());
                predictor_straight_wc_.setLatError(pprediction_->get_offroad_lat_error());
                predictor_straight_wc_.setLonError(pprediction_->get_offroad_lon_error());
                predictor_straight_wc_.setTimeHeadway(pprediction_->get_offroad_time_headway());
                predictor_straight_wc_.setTimeLeeway(pprediction_->get_offroad_time_leeway());
                predictor_straight_wc_.setWidthUB(pprediction_->get_prediction_width_ub());
                predictor_straight_wc_.setWidthLB(pprediction_->get_prediction_width_lb());

                switch(pprediction_->get_setbased_prediction_strategy())
                {
                    case 0:
                        predictor_roadbased_ex_.setLaneWidthPredictions(false);
                        predictor_roadbased_ex_.setLateralPredictions(false);
                        predictor_roadbased_wc_.setLaneWidthPredictions(false);
                        predictor_roadbased_wc_.setLateralPredictions(false);
                        break;
                    case 1:
                        predictor_roadbased_ex_.setLaneWidthPredictions(true);
                        predictor_roadbased_ex_.setLateralPredictions(false);
                        predictor_roadbased_wc_.setLaneWidthPredictions(true);
                        predictor_roadbased_wc_.setLateralPredictions(false);
                        break;
                    case 2:
                        predictor_roadbased_ex_.setLaneWidthPredictions(true);
                        predictor_roadbased_ex_.setLateralPredictions(true);
                        predictor_roadbased_wc_.setLaneWidthPredictions(true);
                        predictor_roadbased_wc_.setLateralPredictions(true);
                        break;
                }
                
                const double dt_rb = pprediction_->get_roadbased_prediction_duration();
                const double dt_or = pprediction_->get_offroad_prediction_duration();

                if(tpsetReader_->hasUpdate())
                {
                    predictionSet_ex_.clear();
                    predictionSet_wc_.clear();
                    tpsetReader_->getData(participantSet_);
                    for(const auto& participant:participantSet_)
                    {
                        const double v = std::sqrt(participant.getVx()*participant.getVx()+participant.getVy()*participant.getVy());
                        const double area = participant.getWidth()*participant.getLength();
                        if(v<0.1||area>100.0)
                        {
                            predictor_straight_ex_.setTMaxUTC(participant.getObservationTime() + dt_or);
                            predictor_straight_ex_.predict(participant,predictionSet_ex_);
                            predictor_straight_wc_.setTMaxUTC(participant.getObservationTime() + dt_or);
                            predictor_straight_wc_.predict(participant,predictionSet_wc_);
                        }
                        else
                        {
                            predictor_roadbased_ex_.setTMaxUTC(participant.getObservationTime() + dt_rb);
                            predictor_straight_ex_.setTMaxUTC(participant.getObservationTime() + dt_or);
                            predictor_roadbased_wc_.setTMaxUTC(participant.getObservationTime() + dt_rb);
                            predictor_straight_wc_.setTMaxUTC(participant.getObservationTime() + dt_or);
                            if (! predictor_roadbased_ex_.predict(participant,predictionSet_ex_))
                            {
                                predictor_straight_ex_.predict(participant,predictionSet_ex_);
                            }
                            if (!predictor_roadbased_wc_.predict(participant,predictionSet_wc_))
                            {
                                predictor_straight_wc_.predict(participant,predictionSet_wc_);
                            }
                        }
                        
                    }                    
                    prediction_writer_ex_->write(predictionSet_ex_);
                    prediction_writer_wc_->write(predictionSet_wc_);
                }
            }            
        };

    }
}