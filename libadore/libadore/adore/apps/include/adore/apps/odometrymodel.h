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
#include <adore/mad/oderk4.h>
#include <adore/mad/adoremath.h>
#include <adore/fun/ctrl/kinematic_model.h>
#include <iostream>
#include <random>

namespace adore
{

    namespace apps
    {
        /**
         * @brief a model for odometry sensor
         * integrates velocities as measured with errors 
         */
        class OdometryModel
        {
            private:
                double last_time_;
                double integration_step_;
                bool last_time_valid_;
                adore::fun::VehicleMotionState9d x_estimate_;/**< last vehicle state estimate*/
                bool relative_to_map_;/**< switch between map-relative odometry (default) and absolute odometry (debugging)*/
                std::default_random_engine generator_;
                std::normal_distribution<double> distribution_;
                                
                adore::mad::AReader<adore::fun::VehicleMotionState9d>* vehicle_model_input_; /** < reads "true" state from vehicle model */
                adore::mad::AWriter<adore::fun::VehicleMotionState9d>* odometry_estimate_output_; /** < writes "estimate" for vehicle state */
                adore::mad::OdeRK4<double> solver_;
                adore::mad::AFeed<adore::sim::ResetVehiclePose>* reset_pose_feed_; /** < resets zero position (map origin)*/
                adore::params::APOdometryModel* params_;/**<odometry based state estimation model parameters*/

            public:
                OdometryModel(adore::sim::AFactory* sim_factory = adore::sim::SimFactoryInstance::get(),
                             adore::params::AFactory* paramfactory = adore::params::ParamsFactoryInstance::get(),
                             unsigned int seed = 0): generator_(seed),distribution_(0.0,1.0)
                {
                    std::srand(seed);
                    relative_to_map_ = true;
                    integration_step_ = 0.005;
                    reset_pose_feed_ = sim_factory->getVehiclePoseResetFeed();
                    vehicle_model_input_ = sim_factory->getVehicleMotionStateReader();
                    odometry_estimate_output_ = sim_factory->getOdometryEstimatedVehicleStateWriter();
                    params_ = paramfactory->getOdometryModel();
                }

                double nrand()
                {
                    return distribution_(generator_);
                }


                /**
                 * @brief simulation step of the odometry estimate model
                 * 
                 */
                virtual void update()
                {
                    if( reset_pose_feed_->hasNext() )
                    {
                        adore::sim::ResetVehiclePose pose;
                        reset_pose_feed_->getLatest(pose);
                        if(relative_to_map_)
                        {
                            x_estimate_.setX(0.0);
                            x_estimate_.setY(0.0);
                            x_estimate_.setZ(0.0);
                            x_estimate_.setPSI(0.0);
                        }
                        else
                        {
                            x_estimate_.setX(pose.getX());
                            x_estimate_.setY(pose.getY());
                            x_estimate_.setZ(pose.getY());
                            x_estimate_.setPSI(pose.getPSI());
                        }
                    }

                    if(vehicle_model_input_->hasData())
                    {
                        adore::fun::VehicleMotionState9d x_true;
                        vehicle_model_input_->getData(x_true);
                        double current_time = x_true.getTime();
                        last_time_ = adore::mad::bound(current_time-1.0,last_time_,current_time);


                        if(last_time_valid_ )
                        {
                            if( current_time>last_time_ )
                            {
                                auto T = adore::mad::sequence(last_time_,integration_step_,current_time);
                                adoreMatrix<double,7,1> x;
                                x(0) = x_estimate_.getX();
                                x(1) = x_estimate_.getY();
                                x(2) = x_estimate_.getPSI();
                                x(3) = x_estimate_.getvx();
                                x(4) = x_estimate_.getvy();
                                x(5) = x_estimate_.getOmega();
                                x(6) = x_estimate_.getAx();
                                adore::fun::KinematicModel model;
                                auto X = solver_.solve(&model,T,x);
                                x = dlib::colm(X,X.nc()-1);
                                
                                const double fade_out = std::min(1.0,std::abs(x_true.getvx()));//do not drift while in standstill
                                const double e_vx = fade_out * params_->get_k_e_vx() * nrand();
                                const double e_vy = fade_out * params_->get_k_e_vy() * nrand();
                                const double e_omega = fade_out * params_->get_k_e_omega() * nrand();
                                const double e_ax = fade_out * params_->get_k_e_ax() * nrand();


                                x_estimate_.setX(x(0));
                                x_estimate_.setY(x(1));
                                x_estimate_.setPSI(x(2));
                                x_estimate_.setvx(x_true.getvx()+e_vx);
                                x_estimate_.setvy(x_true.getvy()+e_vy);
                                x_estimate_.setOmega(x_true.getOmega()+e_omega);
                                x_estimate_.setAx(x_true.getAx()+e_ax);
                                x_estimate_.setDelta(x_true.getDelta());
                                x_estimate_.setTime(current_time);
                                last_time_ = current_time;
                            }
                            odometry_estimate_output_->write(x_estimate_);
                        }
                        else
                        {
                            x_estimate_.setvx(x_true.getvx());
                            x_estimate_.setvy(x_true.getvy());
                            x_estimate_.setOmega(x_true.getOmega());
                            x_estimate_.setAx(x_true.getAx());
                            x_estimate_.setDelta(x_true.getDelta());
                            x_estimate_.setTime(current_time);
                            last_time_ = current_time;
                            last_time_valid_ = true;
                        }
                    }
                }

        };
    }
}