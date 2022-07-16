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
 *   Daniel Heß - definition of view
 ********************************************************************************/


#pragma once
#include "limitline.h"
namespace adore
{
namespace view
{
    /**
     * ALimitLineEnRoute is a view, which provides the next, visible limit line on the vehicle's route
     */
    class ALimitLineEnRoute
    {
        public:
        virtual bool hasLimitLine(double s0)=0;
        virtual LimitLine getLimitLine(double t0,double s0)=0;
    };
}//end view
}//end adore