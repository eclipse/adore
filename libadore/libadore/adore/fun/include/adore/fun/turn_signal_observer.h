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
 *   Matthias Nichting- initial API and implementation
 ********************************************************************************/
#pragma once
#include <adore/fun/afactory.h>

namespace adore
{
    namespace fun
    {
        class TurnSignalObserver
        {
          private:
            bool right_indicator_;
            bool left_indicator_;
            double last_change_left_indicator_;
            double last_change_right_indicator_;
            bool turned_on_manually_left_;
            bool turned_on_manually_right_;
            bool right_state_provided_;
            bool left_state_provided_;
            double turned_on_automatically_left_;
            double turned_on_automatically_right_;
            double max_powerup_delay_;
            double t0_;
            adore::fun::VehicleExtendedState extended_state_;
            adore::fun::AFactory::TVehicleExtendedStateReader* extended_state_reader_;
            AFactory::TIndicatorCommandReader* indicator_command_reader_;

          public:
            TurnSignalObserver()
              : right_indicator_(false)
              , left_indicator_(false)
              , last_change_left_indicator_(std::numeric_limits<double>::lowest())
              , last_change_right_indicator_(std::numeric_limits<double>::lowest())
              , right_state_provided_(true)
              , left_state_provided_(true)
              , turned_on_manually_left_(false)
              , turned_on_manually_right_(false)
              , turned_on_automatically_left_(std::numeric_limits<double>::lowest())
              , turned_on_automatically_right_(std::numeric_limits<double>::lowest())
              , max_powerup_delay_(0.3)
              , t0_(std::numeric_limits<double>::lowest())
            {
                extended_state_reader_ = adore::fun::FunFactoryInstance::get()->getVehicleExtendedStateReader();
                indicator_command_reader_ = FunFactoryInstance::get()->getIndicatorCommandReader();
            }
            ~TurnSignalObserver()
            {
                delete extended_state_reader_;
                delete indicator_command_reader_;
            }
            void update(double t)
            {
                if(!extended_state_reader_->hasUpdate())
                {
                    return;
                }
                extended_state_reader_->getData(extended_state_);
                if(indicator_command_reader_->hasUpdate())
                {
                    fun::IndicatorCommand ic;
                    indicator_command_reader_->getData(ic);
                    turned_on_automatically_left_ = ic.getIndicatorLeftOn()?t:t-10*max_powerup_delay_;
                    turned_on_automatically_right_ = ic.getIndicatorRightOn()?t:t-10*max_powerup_delay_;
                }
                if (extended_state_.getIndicatorLeftOn() != left_indicator_)
                {
                    left_indicator_ = extended_state_.getIndicatorLeftOn();
                    last_change_left_indicator_ = t;
                    left_state_provided_ = false;
                    turned_on_manually_left_ = t > turned_on_automatically_left_ + max_powerup_delay_;                    
                }
                if (extended_state_.getIndicatorRightOn() != right_indicator_)
                {
                    right_indicator_ = extended_state_.getIndicatorRightOn();
                    last_change_right_indicator_ = t;
                    right_state_provided_ = false;
                    turned_on_manually_right_ = t > turned_on_automatically_right_ + max_powerup_delay_;    
                }
                t0_ = t;
            }
            bool rightIndicatorTurnedOnWithinLastSecond(double current_time, double max_delay = 1.0)
            {
                if (bothIndicatorsOn(current_time))
                {
                    return false;
                }
                if (right_indicator_ && (last_change_right_indicator_ + max_delay) > current_time)
                {
                    right_state_provided_ = true;
                    return true;
                }
                return false;
            }
            bool rightIndicatorTurnedOnManuallyWithinLastSecond(double current_time, double max_delay = 1.0)
            {
                return rightIndicatorTurnedOnWithinLastSecond(current_time, max_delay) && turned_on_manually_right_;
            }
            bool leftIndicatorTurnedOnManuallyWithinLastSecond(double current_time, double max_delay = 1.0)
            {
                return leftIndicatorTurnedOnWithinLastSecond(current_time, max_delay) && turned_on_manually_left_;
            }
            bool leftIndicatorTurnedOnWithinLastSecond(double current_time, double max_delay = 1.0)
            {
                if (bothIndicatorsOn(current_time))
                {
                    return false;
                }
                if(left_indicator_ && (last_change_left_indicator_ + max_delay) > current_time)
                {
                    left_state_provided_ = true;
                    return true;
                }
                return false;
            }
            bool bothIndicatorsOn(double t = -1.0) const
            {
                if (t<0.0)t=t0_;
                if (right_indicator_ && left_indicator_ )
                {
                    if (std::abs(last_change_right_indicator_-last_change_left_indicator_) < max_powerup_delay_)
                    {
                        return true;
                    }
                    if (t - last_change_right_indicator_ > 0.5 && t - last_change_left_indicator_ > 0.5)
                    {
                        return true;
                    }
                }
                return false;
            }
            bool newRightIndicatorOnEvent()
            {
                if (bothIndicatorsOn())
                {
                    return false;
                }
                if(right_indicator_ && !right_state_provided_)
                {
                    right_state_provided_ = true;
                    return true;
                }
                return false;
            }
            bool newLeftIndicatorOnEvent()
            {
                if (bothIndicatorsOn())
                {
                    return false;
                }
                if(left_indicator_ && !left_state_provided_)
                {
                    left_state_provided_ = true;
                    return true;
                }
                return false;
            }
            bool newManualLeftIndicatorOnEvent()
            {
                return newLeftIndicatorOnEvent() && turned_on_manually_left_;
            }
            bool newManualRightIndicatorOnEvent()
            {
                return newRightIndicatorOnEvent() && turned_on_manually_right_;
            }
        };
    }  // namespace fun
}  // namespace adore