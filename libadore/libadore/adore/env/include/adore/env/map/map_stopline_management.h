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
 ********************************************************************************/


#pragma once

#include "adore/env/borderbased/lanepositionedobjectset.h"
#include "adore/env/borderbased/stopline.h"
#include <deque>

namespace adore
{
	namespace env
	{
		/**
		 * @brief manage new and outdated stop lines based on newly visible and no longer visible borders and the last state of the object
		 * 
		 */
		class MapStopLineManagement
		{
			adore::env::BorderBased::LanePositionedObjectSet m_globalSet;
			adore::env::BorderBased::LanePositionedObjectSet m_localSet;
			std::deque<adore::env::BorderBased::BorderID> m_borderQueue;

		public:
			/**
			 * @brief Construct a new Map Stop Line Management object
			 * 
			 */
			MapStopLineManagement()
			{
				m_localSet.setIsOwner(false);
			}

			/**
			 * @brief initialization routine
			 * 
			 * @param globalSet 
			 */
			void init(adore::env::BorderBased::LanePositionedObjectSet &globalSet)
			{
				m_globalSet = globalSet;
				globalSet.setIsOwner(false);
			}
			
			/**
			 * @brief clear local sets and pending updates
			 * 
			 */
			void reset()
			{
				m_localSet.clear();
				m_borderQueue.clear();
			}
			/**
			 * @brief receive newly visible or newly outdated stop lines based on new and outdated borders
			 * 
			 * @param newBorders 
			 * @param outdatedBorders 
			 * @param newLines 
			 * @param outdatedLines 
			 * @param MAX_SEND_NUMBER 
			 */
			void run
			(
				std::vector<adore::env::BorderBased::Border*> newBorders,
				std::vector<adore::env::BorderBased::BorderID> outdatedBorders,
				std::vector<adore::env::BorderBased::StopLine> &newLines,
				std::vector<adore::env::BorderBased::StopLine> &outdatedLines,
				int MAX_SEND_NUMBER = 5
			)
			{
				newLines.clear();
				outdatedLines.clear();

				/* collect outdated stop lines */
				for(auto borderID = outdatedBorders.begin(); borderID!=outdatedBorders.end(); borderID++)
				{
					if(m_localSet.hasObjects(*borderID))
					{
						for(auto it = m_localSet.getObjects(*borderID); it.first!=it.second; it.first++)
						{
							outdatedLines.push_back(*(static_cast<adore::env::BorderBased::StopLine*>(it.first->second)));
						}
					}
				}
				/* remove outdated stop lines from local set */
				m_localSet.eraseObjectsBorderBased(outdatedBorders);
				

				/* collect new stop lines */
				/* add borders to queue that has to be worked off */
				for(auto border = newBorders.begin(); border!=newBorders.end(); border++)
				{
					m_borderQueue.push_back((*border)->m_id);
				}

				/* collect new stop lines from borders in border queue */
				int newCount = 0;
				int bCount = 0;
				for(auto borderId = m_borderQueue.begin(); borderId != m_borderQueue.end(); borderId++)
				{
					bCount++;
					if(!m_localSet.hasObjects(*borderId))
					{
						for(auto it = m_globalSet.getObjects(*borderId); it.first!=it.second; it.first++)
						{
							newLines.push_back(*(static_cast<adore::env::BorderBased::StopLine*>(it.first->second)));
							newCount++;
							m_localSet.insert_object(static_cast<adore::env::BorderBased::StopLine*>(it.first->second));
						}
						if(newCount>=MAX_SEND_NUMBER)
						{
							break;
						}
					}
				}

				/* remove all processed borders from border queue */
				for(;bCount>0;bCount--)
				{
					m_borderQueue.pop_front();
				}
			}
		};
	}
}