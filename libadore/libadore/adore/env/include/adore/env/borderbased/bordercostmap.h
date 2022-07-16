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

#include <unordered_map>
#include <adore/env/borderbased/border.h>
#include <adore/env/ego/navigationcost.h>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * This container maps from a BorderID to the remaining cost to reach the goal position
			 */
			class BorderCostMap : public std::unordered_map<BorderID, adore::env::NavigationCost, adore::env::BorderBased::BorderIDHasher>
			{
			public:
				typedef std::pair<BorderID, adore::env::NavigationCost> TCostPair;
				/** get a random pair of borderID and borderCost
				 *  @param[out] out filled pair of borderID and (double) borderCost
				 *  @return true if pair is filled, false if not
				 */
				bool getRandomBorderIDCostPair(TCostPair &result)
				{
					if(this->size()==0)
					{
						return false;
					}
					int r = std::floor(((double)(std::rand())/((double)RAND_MAX)) * ((double)this->size()));				
					for(auto it = this->begin();it!=this->end();it++)
					{
						if(r==0)
						{
							result = *it;
							return true;
						}
						r--;
					}
					auto it = this->begin();
					result = *it;
					return true;
				}
			};
		}
	}
}