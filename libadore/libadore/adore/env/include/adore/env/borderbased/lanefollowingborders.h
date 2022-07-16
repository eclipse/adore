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

#include <adore/env/borderbased/borderaccumulator.h>
#include <adore/env/borderbased/bordercostmap.h>
#include <adore/env/borderbased/border.h>
#include <adore/env/borderbased/borderset.h>
#include <list>

namespace adore
{
namespace env
{
namespace BorderBased
{
/**
 * @brief Selects Borders from BorderSet required for LaneFollowingView construction
 */
class LaneFollowingBorders
{
    private:
    std::list<const Border*> laneFollowingBorders_;/**<Borders (right-hand) proceeding along the current lane from upstream to downstream*/


    public:
    /**
     * @return initial border used as ventage point for construction of view
     */
    const Border* getInitialBorder()
    {
        return nullptr;
    }
    /**
     * @return current result of the class: Borders belonging to current lane, ordered upstream to downstream
     */
    const std::list<const Border*>* getLaneFollowingBorders()
    {
        return &laneFollowingBorders_;
    }
};
}
}
}
