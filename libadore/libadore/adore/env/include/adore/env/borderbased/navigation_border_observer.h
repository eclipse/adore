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

#include "adore/mad/com_patterns.h"
#include "adore/env/borderbased/borderset.h"
#include "adore/env/borderbased/border_observer.h"

namespace adore
{
  namespace env
  {
    namespace BorderBased
    {
			class NavigationBorderObserver : public BorderObserver
			{
			  private:
				bool m_changedBordersDuringLastUpdate;
				bool m_finishedUpdating;
			  
        public:
				NavigationBorderObserver()
				{
					m_changedBordersDuringLastUpdate = false;
					m_finishedUpdating = false;
				}
				void init(BorderSet * set)
				{
					set->deepBorderCopy(*dynamic_cast<BorderSet*>(this));
					//auto it = set.getAllBorders();
					//for(;it.first!=it.second;it.first++)
					//{
					//	auto border = it.first->second;
					//	insert_border(new Border(*border));
					//}
				}
				virtual void update() override
				{
					bool relevantUpdateFound = false;
					while (feed_->hasNext())
					{
						/*
							getting old ID and new border
							check whether there is a relevant change in the borderset (new border added or relevant type change)
							delete border with old ID
						 */
						// reading feed
						Border* b = new Border();
						BorderID oldID;
						feed_->getNext(*b);
						LOG_T("Nav received: %s\t%i", b->m_id.toString().c_str(), static_cast<int>(b->m_type));

						// process feed information
						// if the current border has a relevant type and doesn't exist in border set, a relevant update was found
						if(b->typeIsChangeable() && !hasBorder(b))
						{							
							relevantUpdateFound = true;
						}
						// if oldID exists in border set, remove border with that id
						if(hasBorder(oldID))
						{
							erase_border(oldID);
						}
						insert_border(b,true);
					}
					m_finishedUpdating = !relevantUpdateFound && m_changedBordersDuringLastUpdate;
					m_changedBordersDuringLastUpdate = relevantUpdateFound;
				}
				bool navigationReplanningNecessary()
				{
					return m_finishedUpdating;
				}
			};
    }
  }
}