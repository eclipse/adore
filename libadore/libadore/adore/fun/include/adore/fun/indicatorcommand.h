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

namespace adore
{
    namespace fun
    {
        /**
         * A control input, which allows to manipulate indicators (blinkers) of the vehicle.
         */
        struct IndicatorCommand
        {
            private:
            bool indicatorLeftOn_;
            bool indicatorRightOn_;

            public:
            IndicatorCommand():indicatorLeftOn_(false),indicatorRightOn_(false)
            {

            }
            bool getIndicatorLeftOn() const
            {
            	return this->indicatorLeftOn_;
            }
            void setIndicatorLeftOn(bool indicatorLeftOn) 
            {
            	this->indicatorLeftOn_ = indicatorLeftOn;
            }


            bool getIndicatorRightOn() const
            {
            	return this->indicatorRightOn_;
            }
            void setIndicatorRightOn(bool indicatorRightOn) 
            {
            	this->indicatorRightOn_ = indicatorRightOn;
            }
        
        };
    }
}