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
#include <adore/fun/afactory.h>
#include <adore/params/afactory.h>
#include <adore/fun/ctrl/vlb_openloop.h>
#include <adore/mad/oderk4.h>
#include <adore/mad/adoremath.h>
#include <iostream>

namespace adore
{
    /**
     * @brief abstraction of functional modules to define functionality decoupled from the middleware
     * 
     */
    namespace apps
    {
        /**
         * @brief a vehicle model which can be used in simulations
         * 
         * needs a timer, control_input, gps_output from adore::sim
         * listens to reset_pose_feed and reset_twist_feed to reset pose/twist if requested
         * uses run()-function to update
         * 
         * @see adore::params::APVehicle on how to configure the vehicle model
         * @see VehicleModel::run()
         */
        class VehicleModel//@TODO: public AFixedRateApp - derive from process template which controls execution-timing during non-realtime simulation 
        {
            private:
                params::APVehicle* params_;
                double last_time_;
                double integration_step_;
                double last_time_checkpoint_clearance_;
                bool last_time_valid_;
                bool automatic_control_;
                bool checkpoint_clearance_;
                bool supress_input_;
                double indicator_left_off_from_; /** < upper time bound for setting left indicator on */
                double indicator_right_off_from_;/** < upper time bound for setting right indicator on */
                adoreMatrix<double,10,1> x_;
                adore::fun::MotionCommand u_;
                adore::mad::AReader<double>* timer_;
                adore::mad::AReader<adore::fun::MotionCommand>* control_input_; 
                adore::mad::AWriter<adore::fun::VehicleMotionState9d>* sim_output_; /** < publishes ego state measurement inside vehicle */
                adore::mad::AWriter<adore::fun::VehicleMotionState9d>* gps_output_; /** < publishes ego state measurement inside vehicle */
                adore::mad::AWriter<adore::fun::VehicleMotionState9d>* odom_output_; /** < publishes ego state measurement inside vehicle */
                adore::mad::AWriter<adore::fun::VehicleExtendedState>* extended_state_output_; /** < publishes extended state values inside vehicle */
      			adore::mad::AReader<adore::fun::IndicatorCommand>* indstateReader_;             /**< reads indicator commands*/
                adore::fun::IndicatorCommand indicatorCommand_;
                adore::mad::OdeRK4<double> solver_;
                adore::mad::AFeed<adore::sim::ResetVehiclePose>* reset_pose_feed_;
                adore::mad::AFeed<adore::sim::ResetVehicleTwist>* reset_twist_feed_;  
                adore::sim::AFactory::TParticipantWriter* participant_writer_;/** < publishes vehicle state globally*/   
            protected:          
                adore::env::traffic::Participant::TTrackingID simulationID_;
                adore::env::traffic::Participant::TV2XStationID v2xStationID_;


            public:
                /**
                 * @brief Construct a new Vehicle Model object
                 *  
                 * @param external_ego_measurement_models if true, only simulated true vehicle state is published. if false, odom and localization are also published with same data.
                 * @param sim_factory adore::sim factory
                 * @param paramfactory adore::params factory
                 * @param simulationID id of vehicle in simulation
                 */
                VehicleModel(bool external_ego_measurement_models, adore::env::traffic::Participant::TTrackingID simulationID, adore::env::traffic::Participant::TV2XStationID v2xStationID)
                :indicator_right_off_from_(std::numeric_limits<double>::min()),indicator_left_off_from_(std::numeric_limits<double>::min())
                {
                    simulationID_ = simulationID;
                    v2xStationID_ = v2xStationID;
                    integration_step_ = 0.005;
                    last_time_valid_ = false;
                    x_ = dlib::zeros_matrix<double>(10l,1l);
                    timer_ = adore::sim::SimFactoryInstance::get()->getSimulationTimeReader();
                    control_input_ = adore::sim::SimFactoryInstance::get()->getMotionCommandReader();
                    sim_output_ = adore::sim::SimFactoryInstance::get()->getVehicleMotionStateWriter();
                    if(external_ego_measurement_models)
                    {
                        odom_output_ = nullptr;
                        gps_output_ = nullptr;
                    }
                    else
                    {
                        odom_output_ = adore::sim::SimFactoryInstance::get()->getOdometryEstimatedVehicleStateWriter();
                        gps_output_ = adore::sim::SimFactoryInstance::get()->getLocalizationEstimatedVehicleStateWriter();
                    }
                    extended_state_output_ = adore::sim::SimFactoryInstance::get()->getVehicleExtendedStateWriter();
                    reset_pose_feed_ = adore::sim::SimFactoryInstance::get()->getVehiclePoseResetFeed();
                    reset_twist_feed_ = adore::sim::SimFactoryInstance::get()->getVehicleTwistResetFeed();
                    participant_writer_ = adore::sim::SimFactoryInstance::get()->getParticipantWriter();
                    params_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
                    automatic_control_ = true;
                    checkpoint_clearance_ = false;
                    last_time_checkpoint_clearance_ = 0.0;
                    supress_input_ = false;
                    indstateReader_ = adore::fun::FunFactoryInstance::get()->getIndicatorCommandReader();
                }

                /**
                 * @brief switches between manual and automatic control input
                 */
                void setAutomaticControl(bool value)
                {
                    automatic_control_=value;
                }

                /**
                 * @brief confirmation of current checkout
                 */
                void setCheckpointClearance()
                {
                    checkpoint_clearance_=true;
                }

                /**
                 * @brief supress input to introduce errors, without deactivating automatic control
                 */
                void setSupressInput(bool value)
                {
                    supress_input_ = value;
                }

                bool setIndicatorRightOn(double duration)
                {
                    if(!last_time_valid_)
                    {
                        return false;
                    }
                    indicator_right_off_from_ = last_time_ + duration;
                    return true;
                }

                bool setIndicatorLeftOn(double duration)
                {
                    if(!last_time_valid_)
                    {
                        return false;
                    }
                    indicator_left_off_from_ = last_time_ + duration;
                    return true;
                }

                /**
                 * @brief simulation step of the vehicle model
                 * 
                 */
                virtual void run()
                {
                    if( timer_->hasData() )
                    {

                        if( reset_pose_feed_->hasNext() )
                        {
                            adore::sim::ResetVehiclePose pose;
                            reset_pose_feed_->getLatest(pose);
                            x_(0) = pose.getX();
                            x_(1) = pose.getY();
                            x_(2) = pose.getPSI();

                            x_(3) = 0.0;
                            x_(4) = 0.0;
                            x_(5) = 0.0;
                            x_(6) = 0.0;
                            x_(7) = 0.0;
                            x_(8) = 0.0;
                            x_(9) = 0.0;
                        }
                        if( reset_twist_feed_->hasNext() )
                        {
                            adore::sim::ResetVehicleTwist twist;
                            reset_twist_feed_->getLatest(twist);
                            x_(3) = twist.getVx();
                            x_(4) = twist.getVy();
                            x_(5) = twist.getOmega();
                            x_(6) = 0.0;
                            x_(7) = 0.0;
                            x_(8) = 0.0;
                            x_(9) = 0.0;
                        }

                        double current_time;
                        timer_->getData(current_time);

                        //handle checkpoint clearance event
                        if( checkpoint_clearance_ )
                        {
                            last_time_checkpoint_clearance_ = current_time;
                            checkpoint_clearance_ = false;
                        }
                        bool delayed_checkpoint_clearance = (current_time - last_time_checkpoint_clearance_)<3.0;

                        if(last_time_valid_)
                        {
                            if( automatic_control_ 
                            &&  control_input_->hasData()
                            && !supress_input_ )
                            {
                                control_input_->getData(u_);
                            }
                            else
                            {
                                u_.setAcceleration(0.0);
                                u_.setSteeringAngle(0.0);
                            }
                            
                            auto T = adore::mad::sequence(last_time_,integration_step_,current_time);
                            
                            x_(6) = u_.getAcceleration();
                            x_(7) = u_.getSteeringAngle() / params_->get_steeringRatio();
                            x_(8) = 0.0; //dax=0
                            x_(9) = 0.0; //ddelta=0
                            adore::fun::VLB_OpenLoop model(params_);
                            auto X = solver_.solve(&model,T,x_);
                            x_ = dlib::colm(X,X.nc()-1);
                            
                            //vehicle state measurement
                            adore::fun::VehicleMotionState9d xout;
                            xout.setX(x_(0));
                            xout.setY(x_(1));
                            xout.setZ(0.0);
                            xout.setPSI(x_(2));
                            xout.setvx(x_(3));
                            xout.setvy(x_(4));
                            xout.setOmega(x_(5));
                            xout.setAx(x_(6));
                            xout.setDelta(x_(7));
                            xout.setTime(current_time);
                            sim_output_->write(xout);
                            if(gps_output_!=nullptr)gps_output_->write(xout);
                            if(odom_output_!=nullptr)odom_output_->write(xout);

                            //vehicle extended state
                            if(indstateReader_->hasUpdate())
                            {
                                indstateReader_->getData(indicatorCommand_);
                            }
                            adore::fun::VehicleExtendedState xxout;
                            xxout.setGearState(adore::fun::VehicleExtendedState::Drive);
                            xxout.setAutomaticControlAccelerationOn(automatic_control_);
                            xxout.setAutomaticControlAccelerationActive(automatic_control_);
                            xxout.setAutomaticControlSteeringOn(automatic_control_);
                            xxout.setCheckpointClearance(delayed_checkpoint_clearance);
                            xxout.setIndicatorLeftOn(indicatorCommand_.getIndicatorLeftOn() || indicator_left_off_from_>current_time);
                            xxout.setIndicatorRightOn(indicatorCommand_.getIndicatorRightOn() || indicator_right_off_from_>current_time);
                            extended_state_output_->write(xxout);
                            
                            //publication of vehicle state to other vehicles
                            adore::env::traffic::Participant yout;
                            double length = params_->get_a()+params_->get_b()+params_->get_c()+params_->get_d();
                            double distance_to_center = length*0.5-params_->get_d();
                            yout.center_(0) = xout.getX() + std::cos(xout.getPSI())*distance_to_center;
                            yout.center_(1) = xout.getY() + std::sin(xout.getPSI())*distance_to_center;
                            yout.center_(2) = xout.getZ();
                            yout.classification_ = adore::env::traffic::Participant::EClassification::CAR;
                            yout.classification_certainty_ = 1.0;
                            yout.existance_certainty_ = 1.0;
                            yout.acceleration_x_ = xout.getAx();
                            yout.vx_ = xout.getvx();
                            yout.vy_ = xout.getvy() + xout.getOmega() * distance_to_center;
                            yout.yawrate_ = xout.getOmega();
                            yout.observation_time_ = current_time;
                            yout.height_ = 1.8;
                            yout.length_ = length;
                            yout.width_ = params_->get_bodyWidth();
                            yout.yaw_ = xout.getPSI();
                            yout.trackingID_ = simulationID_;
                            yout.v2xStationID_ = v2xStationID_;
                            participant_writer_->write(yout);

                            last_time_ = current_time;
                        }
                        else
                        {
                            last_time_ = current_time;
                            last_time_valid_ = true;
                        }
                    }
                }

        };
    }
}