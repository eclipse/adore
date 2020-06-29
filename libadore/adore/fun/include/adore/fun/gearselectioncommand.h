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
 *  Jan Lauermann - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/fun/vehicleextendedstate.h>

namespace adore
{
    namespace fun
    {
        /**
         * A control input, which allows to manipulate gear state of the vehicle.
         */
        struct GearSelectionCommand
        {
            private: 
            VehicleExtendedState::GearState gear_;

            public:
            VehicleExtendedState::GearState getGear() const
            {
            	return this->gear_;
            }
            void setGear(VehicleExtendedState::GearState gear) 
            {
            	this->gear_ = gear;
            }

        };
    }
}