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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/
#pragma once
#include <adore/fun/afactory.h>

namespace adore
{
namespace fun
{
    class ActivationStateObserver
    {
        private:
            AFactory::TVehicleExtendedStateReader* vehicle_extended_state_reader_;/**< get information about automatic control state*/
        public:
            ActivationStateObserver()
            {
                vehicle_extended_state_reader_ = FunFactoryInstance::get()->getVehicleExtendedStateReader();
            }
            ~ActivationStateObserver()
            {
                delete vehicle_extended_state_reader_;
            }
            bool isAutomaticControlEnabled()
            {
                if(!vehicle_extended_state_reader_->hasData())
                {
                    return false;
                }
                VehicleExtendedState xx;
                vehicle_extended_state_reader_->getData(xx);    
                return xx.getAutomaticControlOn() //< Freigabe im Fahrzeuginterface für Längs und Quer erhalten
                    && xx.getAutomaticControlAccelerationActive();// Bestätigung der Freigabe durch Benutzer/Gaspedal erfolgt
            }

    };
}
}