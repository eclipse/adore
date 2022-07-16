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
 *   Matthias Nichting
 ********************************************************************************/

#pragma once
#include <cstdint>

namespace adore
{
  namespace sim
  {
    struct Action
    {
        private:
        uint32_t action_;
        public:
        uint32_t getAction() const
        {
            return action_;
        }
        void setAction (uint32_t action)
        {
            action_=action;
        }
    };
  }
}