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
#include <adore/view/atrafficpredictionview.h>
#include <adore/env/afactory.h>

namespace adore
{
    namespace env
    {
        class DecoupledTrafficPredictionView: public adore::view::ATrafficPredictionView
        {
            private:
            AFactory::TOCPredictionSetReader* expectedReader_;
            AFactory::TOCPredictionSetReader* emergencyReader_;
            OccupancyCylinderPredictionSet expected_predictions_;
            OccupancyCylinderPredictionSet emergency_predictions_;
            double pet_min_;/**< post encroachment time, TODO: load parameter*/
            
            public:
            DecoupledTrafficPredictionView(AFactory* factory = EnvFactoryInstance::get())
            {
                expectedReader_ = factory->getExpectedPredictionSetReader();
                emergencyReader_ = factory->getWorstCasePredictionSetReader();
                pet_min_ = 0.9;
            }
            /**
             * get latest prediction sets
             */
            void update()
            {
                  expected_predictions_.clear();
                  expectedReader_->getData(expected_predictions_);
                  emergency_predictions_.clear();
                  emergencyReader_->getData(emergency_predictions_);
            }
            /**
             * overlapsExpectedBehavior returns true if given space overlaps with most likely current or future object positions
             */
            virtual bool overlapsExpectedBehavior(const adore::mad::OccupancyCylinderTree& space)const override
            {
              for(const auto& prediction:expected_predictions_)
              {
                double pet;
                if(prediction.occupancy_.getPostEncroachmentTime(space,pet_min_,pet))
                {
                    return true;
                }
              }
              return false;
            }
            /**
             * overlapsEmergencyBehavior returns true if given space overlaps with current or future object positions belongig to the emergency behavior of these objects
             */
            virtual bool overlapsEmergencyBehavior(const adore::mad::OccupancyCylinderTree& space)const override
            {
              for(const auto& prediction:emergency_predictions_)
              {
                double pet;
                if(prediction.occupancy_.getPostEncroachmentTime(space,pet_min_,pet))
                {
                    return true;
                }
              }
              return false;
            }
        };
    }
}