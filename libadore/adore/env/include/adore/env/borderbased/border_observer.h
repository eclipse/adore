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
 *   Robert Markowski - initial API and implementation
 ********************************************************************************/

#pragma once

#include "adore/env/borderbased/borderset.h"
#include "adore/mad/com_patterns.h"
#include <list>
#include <unordered_map>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			class BorderObserver :public adore::mad::ALocalObserver<Border>,public BorderSet
			{
			public:
				BorderObserver()
				{
				}
				/**
				 * update - retrieve new borders and updates on border information
				 */
				void update() override
				{
					while (feed_->hasNext())
					{
						Border* b = new Border();
						BorderID oldID;
						feed_->getNext(*b);
						auto result = m_byID.find(oldID);
						if (result!=m_byID.end())//object exists => delete and create new
						{
							erase_border(oldID);
						}

						insert_border(b);
					}
				}

				void discard_radius_based(double x, double y, double z, double radius) override
				{
          removeBorders(
            getBordersOutsideRegion(  x-radius,
			                          x+radius,
			  	                      y-radius,
				                      y+radius)
          );
				}

        void addFeed(adore::mad::AFeed<Border> * feed)
        {
          feed_ = feed;
        }

			};
		}
	}
}