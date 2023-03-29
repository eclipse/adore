/********************************************************************************
 * Copyright (C) 2017-2022 German Aerospace Center (DLR). 
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
#include <list>

namespace adore
{
	namespace apps
	{
		class FeedbackController
		{
		private:
			adore::fun::LinearTrackingController linear_tracking_controller_;
			adore::mad::AReader<adore::fun::VehicleExtendedState>* x_state_reader_;
			adore::fun::SetPointRequest spr_;
			adore::fun::VehicleExtendedState x_state_;
			adore::fun::MotionCommand cmd_;
			adore::params::APVehicle* ap_vehicle_;
			adore::params::APTrajectoryTracking* ap_tracking_;
			adore::params::APEmergencyOperation* ap_emergency_;
			double last_steering_angle_;
			double last_t_;
			double last_ddelta_;
			double last_t_manual_control_;
			double dt_;
			std::list<double> dt_list_;
			int dt_list_max_size_;
		protected:
			adore::mad::AReader<adore::fun::SetPointRequest>* spr_reader_;
			adore::mad::AReader<adore::fun::VehicleMotionState9d>* state_reader_;
			adore::fun::VehicleMotionState9d state_;
			adore::mad::AWriter<adore::fun::MotionCommand>* cmd_writer_;


		public:
			FeedbackController()
						:linear_tracking_controller_(adore::params::ParamsFactoryInstance::get()->getVehicle(),
						 							adore::params::ParamsFactoryInstance::get()->getTrajectoryTracking())
			{
				ap_vehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
				ap_tracking_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryTracking();
				ap_emergency_ = adore::params::ParamsFactoryInstance::get()->getEmergencyOperation();
				spr_reader_ = adore::fun::FunFactoryInstance::get()->getOdomSetPointRequestReader();
				state_reader_ = adore::fun::FunFactoryInstance::get()->getVehicleOdometryMotionStateReader();
				x_state_reader_ = adore::fun::FunFactoryInstance::get()->getVehicleExtendedStateReader();
				cmd_writer_ = adore::fun::FunFactoryInstance::get()->getMotionCommandWriter();
				last_steering_angle_ = 0.0;
				last_t_ = -1.0;
				last_ddelta_ = 0.0;
				last_t_manual_control_ = 0.0;
				dt_ = 0.0;
				dt_list_max_size_ = 100;
			}
			virtual ~FeedbackController()
			{
				delete spr_reader_;
				delete state_reader_;
				delete x_state_reader_;
				delete cmd_writer_;
			}
			void run()
			{
				if( state_reader_==nullptr || !state_reader_->hasData() )return;
				state_reader_->getData(state_);
				const double t = state_.getTime();
				if(t!=last_t_)
				{
					double dt_now = t-last_t_;
					dt_list_.push_front(dt_now);
					while(dt_list_.size()>dt_list_max_size_)dt_list_.pop_back();
					dt_ = 0.0; 
					for(auto value:dt_list_)dt_+=value;
					dt_ = dt_ / (double)(dt_list_.size());
					last_t_ = t;
					std::cout<<"dt="<<dt_<<std::endl;
				}
				else
				{
					return;
				}

				if(!x_state_.getAutomaticControlOn())
				{
					last_t_manual_control_ = last_t_;
				}
				const double t_activation_delay = 0.5;/**< delay before activation begins*/
				const double t_active = std::max(0.0,last_t_-last_t_manual_control_-t_activation_delay);
				const bool automatic_control = t_active>0.0;
				std::cout<<t_active<<", ";

				if( spr_reader_!=0 && spr_reader_->hasData() )
				{

					adore::fun::SetPointRequest spr_tmp;
					spr_reader_->getData(spr_tmp);
					if(spr_tmp.isActive(t))spr_ = spr_tmp;
					
					if(x_state_reader_!=0 && x_state_reader_->hasData())x_state_reader_->getData(x_state_);
					if(spr_.isActive(t))
					{
						//trajectory tracking operation
						auto ref_state = spr_.interpolateReference(t,ap_vehicle_);
						switch(x_state_.getGearState())
						{
							case adore::fun::VehicleExtendedState::Drive:
								{
									linear_tracking_controller_.setUseIntegrator(true);
									if(!automatic_control)
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
									//@TODO: implement a behavior for reversing
								}
								break;
							default:
								{
									//@TODO: implement a behavior for unknown gear
									linear_tracking_controller_.setUseIntegrator(true);
									if(!automatic_control)
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
						}
					}
					else
					{
						//uncontrolled emergency break
						cmd_.setAcceleration(ap_emergency_->getamin());
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

				//fullstop mechanism
				//(prevent vehicle from restarting due to minor localization drift)
				if(state_.getvx()<0.1 && cmd_.getAcceleration()<0.5 && x_state_.getGearState()==adore::fun::VehicleExtendedState::Drive)
				{
					cmd_.setAcceleration(-1.0);
					cmd_.setSteeringAngle(0.0);
				}
				
				//steering rate limiter
				const double ddelta_max_default = ap_tracking_->getDDeltaMax() * 100.0 * dt_; // @TODO: ddeltamax should be given as *steering* (not steering-wheel) change per *second* (not per iteration)
				const double v_full_ddelta = 5;/**<speed at which full steering rate is reached*///@TODO create parameter
				const double ddelta_min_rel = 0.001;//@TODO create parameter
				const double ddelta_max_v = ddelta_max_default * adore::mad::bound(ddelta_min_rel,state_.getvx()/v_full_ddelta,1.0);
				const double t_full_ddelta = 10;/**<time after which full steering rate is reached*///@TODO create parameter
				const double ddelta_max_transition = ddelta_max_default * adore::mad::bound(ddelta_min_rel,t_active/t_full_ddelta,1.0);
				const double ddelta_max = std::min(ddelta_max_v,ddelta_max_transition);
				const double delta_max = last_steering_angle_ + ddelta_max;
				const double delta_min = last_steering_angle_ - ddelta_max;
				std::cout<<dt_<<", "<<ddelta_max_default<<", "<<ddelta_max<<", "<<ddelta_max_transition<<", "<<ddelta_max_v<<std::endl;
				cmd_.setSteeringAngle(adore::mad::bound(delta_min,cmd_.getSteeringAngle(),delta_max));

				//deactivate output, if manual control
				if(!automatic_control)//automation system is not accelerating
				{
					cmd_.setAcceleration(0.0);
					cmd_.setSteeringAngle(state_.getDelta() * ap_vehicle_->get_steeringRatio());//set to current steering wheel angle
				}


				//track the last steering angle for rate limitation
				last_steering_angle_ = cmd_.getSteeringAngle();

				//output control command
				cmd_writer_->write(cmd_);
				
			}

		};
	}
}