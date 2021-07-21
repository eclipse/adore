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
 *    Daniel Heß - initial implementation and API
 ********************************************************************************/


#pragma once
#include "precedence.h"
#include <adore/view/alane.h>
#include <vector>

namespace adore
{
    namespace env
    {
        /**
         * @brief PrecedenceFilter filters precedence rules in the path of the vehicle, as determined by ALane
         */
        class PrecedenceFilter
        {
            public:
            typedef std::vector<const PrecedenceRule*> TFilterResult;
            /**
             * getHighPrecedenceOnLane filters all PrecedenceRules in pset and adds them to rulesset, if the high-priority connection is on the lane
             */
            void getHighPrioritiesOnLane(const PrecedenceSet* pset,adore::view::ALane* lane, TFilterResult& result)
            {
                if(pset!=nullptr && lane!=nullptr  && lane->isValid())
                {
                    for(auto it = pset->getAllRulesIt();it.current()!=it.end();it.current()++)
                    {
                        PrecedenceRule* rule = it.current()->second;
                        double s0,n0,s1,n1;
                        lane->toRelativeCoordinates(rule->high_.from_(0),rule->high_.from_(1),s0,n0);
                        lane->toRelativeCoordinates(rule->high_.to_(0),rule->high_.to_(1),s1,n1);
                        bool inlane0 = (lane->getOffsetOfRightBorder(s0)<=n0 && n0<=lane->getOffsetOfLeftBorder(s0));
                        bool inlane1 = (lane->getOffsetOfRightBorder(s1)<=n1 && n1<=lane->getOffsetOfLeftBorder(s1));
                        if(inlane0 && inlane1)
                        {
                            result.push_back(rule);
                        }                    
                    }
                }
            }
        };
    }
}