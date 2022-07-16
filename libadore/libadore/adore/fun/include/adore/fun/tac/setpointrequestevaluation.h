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
#include <adore/fun/setpointrequest.h>

namespace adore
{
    namespace fun
    {
        /**
         * ASPRConstraint constraint evaluation for SetPointRequest 
         */
        class ASPRConstraint
        {
            public:
            virtual bool isValid(const adore::fun::SetPointRequest& spr)const =0;
        };
        /**
         * ASPRCost cost function for SetPointReqeust
         */
        class ASPRCost
        {
            public:
            /**
             * @return positive double value for given SetPointRequest
             */
            virtual double getCost(const adore::fun::SetPointRequest& spr)const =0;
                        /**
             * @return name of cost function or objective
             */
            virtual std::string getName()const =0;
        };

        
    }
}