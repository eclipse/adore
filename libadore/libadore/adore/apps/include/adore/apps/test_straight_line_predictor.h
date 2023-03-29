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
 *   Daniel Heß - simple traffic predictor for testing of interfaces
 ********************************************************************************/
#pragma once
#include <adore/env/afactory.h>
#include <adore/env/traffic/occupancycylinderprediction.h>
namespace adore
{
namespace apps
{
    class TestStraightLinePredictor
    {
        private:
            adore::env::AFactory::TParticipantSetReader* tpsetReader_; /**< reader of traffic participants */
            adore::env::traffic::TParticipantSet participantSet_; /**< set of participants */
            adore::env::OccupancyCylinderPredictionSet predictionSet_;
            adore::env::AFactory::TOCPredictionSetWriter* prediction_writer_;
            adore::env::OCStraightLinePrediction predictor_;/**< apply straight line prediction strategy*/
        public:
            TestStraightLinePredictor()
            {
                tpsetReader_ = adore::env::EnvFactoryInstance::get()->getTrafficParticipantSetReader();
                prediction_writer_ = adore::env::EnvFactoryInstance::get()->getExpectedPredictionSetWriter();
            }
            void run()
            {
                if(tpsetReader_->hasUpdate())
                {
                    predictionSet_.clear();
                    tpsetReader_->getData(participantSet_);
                    for(const auto& participant:participantSet_)
                    {
                        predictor_.setTMaxUTC(participant.getObservationTime() + 5.0);
                        predictor_.predict(participant,predictionSet_);
                    }                    
                    prediction_writer_->write(predictionSet_);
                }
            }
    };
}
}