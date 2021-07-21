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
 *   Daniel Heß
 *   Jan Lauermann
 ********************************************************************************/
#pragma once

#include <adore/mad/aodesolver.h>
#include <adore/fun/ctrl/linear_tracking_controller.h>
#include <adore/fun/ctrl/terminal_controller.h>
#include <adore/fun/afactory.h>
#include <adore/params/afactory.h>

namespace adore
{
	namespace apps
	{
		class FeedbackController
		{
		private:
			adore::fun::TerminalController terminal_controller_;
			adore::fun::LinearTrackingController linear_tracking_controller_;
			adore::mad::AReader<adore::fun::SetPointRequest>* spr_reader_;
			adore::mad::AReader<adore::fun::TerminalRequest>* trm_reader_;
			adore::mad::AReader<adore::fun::VehicleMotionState9d>* state_reader_;
			adore::mad::AReader<adore::fun::VehicleExtendedState>* x_state_reader_;
			adore::mad::AWriter<adore::fun::MotionCommand>* cmd_writer_;
			adore::fun::SetPointRequest spr_;
			adore::fun::SetPointRequest spr_tmp_;
			adore::fun::TerminalRequest trm_;
			adore::fun::VehicleMotionState9d state_;
			adore::fun::VehicleExtendedState x_state_;
			adore::fun::MotionCommand cmd_;
			adore::params::APVehicle* ap_vehicle_;
			adore::params::APTrajectoryTracking* ap_tracking_;
			adore::params::APEmergencyOperation* ap_emergency_;
			double last_steering_angle_;
			double last_t_;
			double last_ddelta_;

		public:
			FeedbackController(adore::fun::AFactory* fun_factory,
												adore::params::AFactory* params_factory)
						:linear_tracking_controller_(params_factory->getVehicle(),params_factory->getTrajectoryTracking()),
						 terminal_controller_(params_factory->getEmergencyOperation(),params_factory->getVehicle())
			{
				ap_vehicle_ = params_factory->getVehicle();
				ap_tracking_ = params_factory->getTrajectoryTracking();
				ap_emergency_ = params_factory->getEmergencyOperation();
				spr_reader_ = fun_factory->getSetPointRequestReader();
				trm_reader_ = fun_factory->getTerminalRequestReader();
				state_reader_ = fun_factory->getVehicleMotionStateReader();
				x_state_reader_ = fun_factory->getVehicleExtendedStateReader();
				cmd_writer_ = fun_factory->getMotionCommandWriter();
				last_steering_angle_ = 0.0;
				last_t_ = -1.0;
				last_ddelta_ = 0.0;
			}
			virtual ~FeedbackController()
			{
				delete spr_reader_;
				delete trm_reader_;
				delete state_reader_;
				delete x_state_reader_;
				delete cmd_writer_;
			}
			void run()
			{
				if( spr_reader_!=0 && spr_reader_->hasData() && state_reader_!=0 && state_reader_->hasData() )
				{
					state_reader_->getData(state_);
					const double t = state_.getTime();
					if(t!=last_t_)
					{
						last_t_ = t;
					}
					else
					{
						return;
					}
					
					spr_reader_->getData(spr_tmp_);
					if(spr_tmp_.isActive(t))spr_ = spr_tmp_;
					
					if(trm_reader_!=0 && trm_reader_->hasData())trm_reader_->getData(trm_);
					if(x_state_reader_!=0 && x_state_reader_->hasData())x_state_reader_->getData(x_state_);
					if(spr_.isActive(t) && !trm_.isActive(t))
					{
						//trajectory tracking operation
						auto ref_state = spr_.interpolateReference(t,ap_vehicle_);
						switch(x_state_.getGearState())
						{
							case adore::fun::VehicleExtendedState::Drive:
								{
									linear_tracking_controller_.setUseIntegrator(true);
									if(!x_state_.getAutomaticControlAccelerationActive())
									{
										linear_tracking_controller_.resetIntegrator(true);
									}
									else
									{
										linear_tracking_controller_.resetIntegrator(false);
									}
									linear_tracking_controller_.compute_control_input(state_,ref_state,cmd_);
								}
								break;
							case adore::fun::VehicleExtendedState::Reverse:
								{

								}
								break;
							default:
								{
									linear_tracking_controller_.setUseIntegrator(true);
									if(!x_state_.getAutomaticControlAccelerationActive())
									{
										linear_tracking_controller_.resetIntegrator(true);
									}
									else
									{
										linear_tracking_controller_.resetIntegrator(false);
									}
									linear_tracking_controller_.compute_control_input(state_,ref_state,cmd_);//@TODO: remove this line
									// @TODO: implement extendedstatereader, then use next two lines in case of unknown gear state, remove line above
									// cmd_.setAcceleration(0.5*aptracking_->getAxMin());
									// cmd_.setSteeringAngle(0.0);
								}
								break;
						}
					}
					else if (trm_.isActive(t))
					{
						//controlled emergency break
						terminal_controller_.compute_control_input(state_,trm_,cmd_);
						std::cout<<"Emergency break!\n";
					}
					else
					{
						//uncontrolled emergency break
						cmd_.setAcceleration(0.9*ap_emergency_->getamin());
						cmd_.setSteeringAngle(0.0);
						if(spr_.setPoints.size()==0)
						{
							std::cout<<" No SetPointRequest available!\n";
						}
						else
						{
							std::cout<<"SetPointRequest timeout t="<<t<<" tSPR=["<<spr_.setPoints.front().tStart<<";"<<spr_.setPoints.back().tEnd<<"]\n";
						}
					}
				}

				//apply steering ratio
				cmd_.setSteeringAngle(cmd_.getSteeringAngle() * ap_vehicle_->get_steeringRatio());
				
				//steering rate limiter
			
				double measured_delta = state_.getDelta() * ap_vehicle_->get_steeringRatio();
				double ddelta_max_default = ap_tracking_->getDDeltaMax();
				double coeffA = 0.001;
				double coeffB = ap_tracking_->getSteeringRateLimiterGain();
				double v_full_ddelta = 2;
				double v_abs = std::sqrt(state_.getvx()*state_.getvx()+state_.getvy()*state_.getvy());

				//this achieves a gradual acceleration of the steering wheel after hand-over
				if( x_state_.getAutomaticControlAccelerationActive() )
				{
					//gradual acceleration up to max rate, if the user is not steering
					//high(er) speed handling
					if(v_abs > v_full_ddelta)
					{
						last_ddelta_ = std::min(ddelta_max_default,last_ddelta_ + coeffA + coeffB*last_ddelta_);
					}
					//low speed handling
					else
					{
						//ramp from last_ddelta_=0 at v=0 to last_ddelta_=ddelta_max_default at v=v_full_ddelta
						last_ddelta_ = ddelta_max_default * adore::mad::bound(0.0,v_abs,v_full_ddelta) / v_full_ddelta ;
					}	
				}
				else
				{
					//set the steering rate to zero, if the user controls
					last_ddelta_ = 0.0;
				}

				double ddelta_max = last_ddelta_;
				//bound steering rate using last commanded delta
				cmd_.setSteeringAngle(adore::mad::bound(last_steering_angle_-ddelta_max,cmd_.getSteeringAngle(),last_steering_angle_+ddelta_max));

				//fullstop mechanism
				//(prevent vehicle from restarting due to minor gps drift)
				if(v_abs<0.1 && cmd_.getAcceleration()<0.5 && x_state_.getGearState()==adore::fun::VehicleExtendedState::Drive)
				{
					cmd_.setAcceleration(-2.0);
					last_ddelta_ = 0.0;
				}

				//deactivate output, if manual control
				if(!x_state_.getAutomaticControlAccelerationOn())//automation system is not accelerating
				{
					cmd_.setAcceleration(0.0);
				}

				if(!x_state_.getAutomaticControlSteeringOn())//automation system is not steering
				{
					cmd_.setSteeringAngle(measured_delta);
				}
				
				//track the last steering angle for rate limitation
				if(x_state_.getAutomaticControlAccelerationActive())//automation system is active
				{
					last_steering_angle_ = cmd_.getSteeringAngle();
				}
				else //human is steering
				{
					last_steering_angle_ = measured_delta;
				}

				cmd_writer_->write(cmd_);
				
			}

		};
	}
}