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
 *    Thomas Lobig - initial implementation and API
 ********************************************************************************/

#pragma once

#include <vector>

namespace adore
{
    namespace env
    {
        // todo: use this as an example for a templatized class later
        class RoadAnnotation
        {
            public:
            // todo: update internals according to car position
            virtual void update() = 0;
            
        };
    }
}