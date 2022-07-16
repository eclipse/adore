/********************************************************************************
 * Copyright (C) 2017-2021 German Aerospace Center (DLR). 
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
#include <iostream>
#include <random>

namespace adore
{

    namespace apps
    {
        /**
         * @brief a model for localization
         * adds errors to true vehicle state 
         */
        class LocalizationModel
        {
            private:
                std::default_random_engine generator_;
                std::normal_distribution<double> ndistribution_;
                std::uniform_real_distribution<double> udistribution_;
                double eXdrift;/**<current X error*/
                double eYdrift;/**<current Y error*/
                double eXjump;/**<current X error*/
                double eYjump;/**<current Y error*/
                double ePSI;/**<current heading error*/
                bool last_time_valid_;
                double last_time_;
                                
                adore::mad::AReader<adore::fun::VehicleMotionState9d>* vehicle_model_input_; /** < reads "true" state from vehicle model */
                adore::mad::AWriter<adore::fun::VehicleMotionState9d>* localization_estimate_output_; /** < writes "estimate" for vehicle state */
                adore::params::APLocalizationModel* params_;/**<odometry based state estimation model parameters*/

            public:
                LocalizationModel(adore::sim::AFactory* sim_factory = adore::sim::SimFactoryInstance::get(),
                             adore::params::AFactory* paramfactory = adore::params::ParamsFactoryInstance::get(),
                             unsigned int seed = 0): generator_(seed),ndistribution_(0.0,1.0),udistribution_(-1.0,1.0)
                {
                    std::srand(seed);
                    vehicle_model_input_ = sim_factory->getVehicleMotionStateReader();
                    localization_estimate_output_ = sim_factory->getLocalizationEstimatedVehicleStateWriter();
                    params_ = paramfactory->getLocalizationModel();
                    eXjump = 0.0;
                    eYjump = 0.0;
                    eXdrift = 0.0;
                    eYdrift = 0.0;
                    ePSI = 0.0;
                    last_time_valid_ = false;
                }

                double nrand()
                {
                    return ndistribution_(generator_);
                }

                double rand()
                {
                    return udistribution_(generator_);
                }


                /**
                 * @brief simulation step of the odometry estimate model
                 * 
                 */
                virtual void update()
                {
                    if(vehicle_model_input_->hasData())
                    {
                        adore::fun::VehicleMotionState9d x_true;
                        adore::fun::VehicleMotionState9d x_estimate;
                        vehicle_model_input_->getData(x_true);
                        x_estimate = x_true;
                        if(!last_time_valid_)
                        {
                            //initialize with an offset
                            double rjump = nrand() * params_->get_jump_deviation_pos();
                            double phijump = rand() * 3.141592653589793;
                            eXjump = std::cos(phijump)*rjump - std::sin(phijump)*rjump;
                            eYjump = std::sin(phijump)*rjump + std::cos(phijump)*rjump;
                            ePSI = nrand() * params_->get_jump_deviation_heading();
                            last_time_valid_ = true;
                        }
                        else
                        {
                            if(last_time_==x_true.getTime())return;
                        }
                        last_time_ = x_true.getTime();

                        double rdrift = nrand() * params_->get_drift_deviation_pos();
                        double phidrift = rand() * 3.141592653589793;
                        double eXtarget = std::cos(phidrift)*rdrift - std::sin(phidrift)*rdrift;
                        double eYtarget = std::sin(phidrift)*rdrift + std::cos(phidrift)*rdrift;
                        double dX = adore::mad::bound(-params_->get_drift_rate_pos(),0.1*(eXtarget-eXdrift),params_->get_drift_rate_pos());
                        double dY = adore::mad::bound(-params_->get_drift_rate_pos(),0.1*(eYtarget-eYdrift),params_->get_drift_rate_pos());
                        eXdrift+=dX;
                        eYdrift+=dY;
                        if(rand()*0.5+0.5<params_->get_jump_threshold_pos())
                        {
                            double rjump = nrand() * params_->get_jump_deviation_pos();
                            double phijump = rand() * 3.141592653589793;
                            eXjump = std::cos(phijump)*rjump - std::sin(phijump)*rjump;
                            eYjump = std::sin(phijump)*rjump + std::cos(phijump)*rjump;
                        }
                        if(rand()*0.5+0.5<params_->get_jump_threshold_heading())
                        {
                            ePSI = nrand() * params_->get_jump_deviation_heading();
                        }


                        x_estimate.setX(x_true.getX() + eXjump + eXdrift);
                        x_estimate.setY(x_true.getY() + eYjump + eYdrift);
                        x_estimate.setPSI(x_true.getPSI() + ePSI);
                        localization_estimate_output_->write(x_estimate);
                    }
                }

        };
    }
}