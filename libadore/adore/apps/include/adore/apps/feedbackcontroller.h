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
			adore::fun::TerminalRequest trm_;
			adore::fun::VehicleMotionState9d state_;
			adore::fun::VehicleExtendedState x_state_;
			adore::fun::MotionCommand cmd_;
			adore::params::APVehicle* ap_vehicle_;
			adore::params::APTrajectoryTracking* ap_tracking_;
			adore::params::APEmergencyOperation* ap_emergency_;
			double last_steering_angle_;

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
					spr_reader_->getData(spr_);
					if(trm_reader_!=0 && trm_reader_->hasData())trm_reader_->getData(trm_);
					state_reader_->getData(state_);
					if(x_state_reader_!=0 && x_state_reader_->hasData())x_state_reader_->getData(x_state_);
					const double t = state_.getTime();
					if(spr_.isActive(t) && !trm_.isActive(t))
					{
						//trajectory tracking operation
						auto ref_state = spr_.interpolateReference(t,ap_vehicle_);
						switch(x_state_.getGearState())
						{
							case adore::fun::VehicleExtendedState::Drive:
								{
									linear_tracking_controller_.setUseIntegrator(true);
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

				//steering rate limiter: 
				double ddelta_max_default = ap_tracking_->getDDeltaMax();
				double v_full_ddelta = 0.5;
				double ddelta_max_slow = ddelta_max_default * v_full_ddelta / adore::mad::bound(0.0,state_.getvx(),v_full_ddelta);//ramp from ddelta_max_slow=0 at v=0 to ddelta_max_slow=ddelta_max_default at v=v_full_ddelta
				double measured_delta = state_.getDelta();
				double delta_max = last_steering_angle_ + ddelta_max_slow;
				double delta_min = last_steering_angle_ - ddelta_max_slow;
				cmd_.setSteeringAngle(adore::mad::bound(delta_min,cmd_.getSteeringAngle(),delta_max));
				
				if(x_state_.getAutomaticControlSteeringOn())//automation system is steering
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