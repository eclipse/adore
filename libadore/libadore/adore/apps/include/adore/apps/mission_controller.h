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
 *   Jan Lauermann - initial implementation
 ********************************************************************************/
#pragma once

#include <adore/fun/afactory.h>
#include <adore/params/afactory.h>
#include <adore/mad/com_patterns.h>
#include <adore/fun/missiondata.h>

namespace adore
{
    namespace apps
    {
        /**
         * @brief Simple state machine for mission controlling
         */
        class MissionController
        {
        private:
            /* read: */
            adore::mad::AReader<adore::fun::VehicleMotionState9d> * motion_state_reader_; /* vehicle motion state */
            adore::mad::AReader<adore::fun::VehicleExtendedState> * x_state_reader_; /* vehicle extended state*/
            adore::mad::AReader<adore::fun::NavigationGoal> * nav_goal_reader_;  /* navigation goal*/
            adore::params::APMissionControl* p_mission_control_; /* parameters for mission control*/         
            
            /* write: */
            adore::mad::AWriter<adore::fun::MissionData> * mission_data_writer_; /* mission state*/
            
            adore::fun::VehicleMotionState9d motion_state_;
            adore::fun::VehicleExtendedState x_state_;
            adore::fun::NavigationGoal nav_goal_;
            adore::fun::MissionData mission_data_;

        public:
            MissionController()
            {
                motion_state_reader_ = adore::fun::FunFactoryInstance::get()->getVehicleMotionStateReader();
                x_state_reader_ = adore::fun::FunFactoryInstance::get()->getVehicleExtendedStateReader();
                nav_goal_reader_ = adore::fun::FunFactoryInstance::get()->getNavigationGoalReader();
                mission_data_writer_ = adore::fun::FunFactoryInstance::get()->getMissionDataWriter();
                p_mission_control_ = adore::params::ParamsFactoryInstance::get()->getMissionControl();
            }
            ~MissionController()
            {
                delete motion_state_reader_;
                delete x_state_reader_;
                delete nav_goal_reader_;
                delete p_mission_control_;
                delete mission_data_writer_;
            }

            void run()
            {
                //update data
                motion_state_reader_->getData(motion_state_);
                x_state_reader_->getData(x_state_);
                nav_goal_reader_->getData(nav_goal_);

                double distance_to_goal = std::sqrt((motion_state_.getX()-nav_goal_.target_.x_)*(motion_state_.getX()-nav_goal_.target_.x_)
                                            + (motion_state_.getY()-nav_goal_.target_.y_)*(motion_state_.getY()-nav_goal_.target_.y_));

                //state machine
                switch(mission_data_.getMissionState())
                {
                case adore::fun::MissionData::AUTOMATION_NOT_ACTIVE:
                    {
                        if(x_state_.getAutomaticControlAccelerationOn() && distance_to_goal > p_mission_control_->getDistanceToGoalForStart())
                        {
                            mission_data_.setMissionState(adore::fun::MissionData::DRIVING_TO_GOAL);
                        }
                    }
                    break;
                case adore::fun::MissionData::DRIVING_TO_GOAL:
                    {
                        if(distance_to_goal < p_mission_control_->getDistanceToGoalForStop())
                        {
                            mission_data_.setMissionState(adore::fun::MissionData::GOAL_REACHED);
                        }
                    }
                    break;
                case adore::fun::MissionData::GOAL_REACHED:
                    {
                        if(distance_to_goal > p_mission_control_->getDistanceToGoalForStart())
                        {
                            mission_data_.setMissionState(adore::fun::MissionData::DRIVING_TO_GOAL);
                        }
                    }         
                    break;
                }

                //reset mission state, if automation is not actived
                if(!x_state_.getAutomaticControlAccelerationOn())
                {
                    mission_data_.setMissionState(adore::fun::MissionData::AUTOMATION_NOT_ACTIVE);
                }

                //write mission data
                mission_data_writer_->write(mission_data_);
            }
        };
    }
}