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

#include "map_auxiliary.h"
#include <adore/env/tcd/tcdset.h>

namespace adore
{
	namespace env
	{
		/**
		 * @brief manage visible traffic control devices based on vehicle position and last state of the object
		 * 
		 */
		class MapTCDManagement
		{
			adore::env::TCDSet m_globalSet;
			adore::env::TCDSet m_localSet;
		public:
			/**
			 * @brief Construct a new Map T C D Management object
			 * 
			 */
			MapTCDManagement()
			{
				m_localSet.setIsOwner(false);
			}
			/**
			 * @brief initialization routine
			 * 
			 * @param globalSet 
			 */
			void init(adore::env::TCDSet &globalSet)
			{
				m_globalSet = globalSet;
				globalSet.setIsOwner(false);
			}
			/**
			 * @brief clear local set
			 * 
			 */
			void reset()
			{
				m_localSet.clear();
			}
			/**
			 * @brief get new/outdated traffic control devices / traffic lights based on last object state and current vehicle position
			 * 
			 * @param x 
			 * @param y 
			 * @param r 
			 * @param newTCD 
			 * @param outdatedTCD 
			 * @param newTL 
			 * @param outdatedTL 
			 * @param MAX_NEW_NUMBER 
			 */
			void run(
				double x, double y, double r,
				std::vector<adore::env::TrafficControlDevice*> &newTCD,
				std::vector<adore::env::TrafficControlDevice*> &outdatedTCD,
				std::vector<adore::env::TTCDTrafficLightTuple> &newTL,
				std::vector<adore::env::TTCDTrafficLightTuple> &outdatedTL,
				int MAX_NEW_NUMBER = 5
			)
			{
				newTCD.clear();
				outdatedTCD.clear();
				newTL.clear();
				outdatedTL.clear();

				auto corners = MAP_AUX::getCornerPoints(x,y,r);

				/* collect old TCDs/TLs */
				std::vector<int> delTcdId;
				for(auto it = m_localSet.getTCDsOutsideRegion(corners.at(0),corners.at(1),corners.at(2),corners.at(3));it.first!=it.second;it.first++)
				{
					delTcdId.push_back(it.first->second);
					adore::env::TrafficControlDevice* tcd = m_localSet.getTCD(it.first->second);
					if(tcd->getType()==adore::env::TrafficControlDevice::TRAFFIC_LIGHT)
					{
						outdatedTL.push_back(m_localSet.getTCDTrafficLight(tcd->getID()));
					}
					else
					{
						outdatedTCD.push_back(tcd);
					}
				}
				for(auto it = delTcdId.begin(); it!=delTcdId.end();it++)
				{
					m_localSet.eraseTCD(*it);
				}

				/* collect new TCDs/TLs */
				for(auto it = m_globalSet.getTCDsInRegion(corners.at(0),corners.at(1),corners.at(2),corners.at(3)); it.first!=it.second; it.first++)
				{
					auto tcdId = it.first->second;
					if(!m_localSet.hasTCD(tcdId))
					{
						auto tcd = m_globalSet.getTCD(tcdId);
						m_localSet.insertTCD(tcd);
						if(tcd->getType()==adore::env::TrafficControlDevice::TRAFFIC_LIGHT)
						{
							auto tcdt = m_globalSet.getTCDTrafficLight(tcdId);
				
							m_localSet.setMovementId(std::get<0>(tcdt)->getID(), std::get<1>(tcdt));
							m_localSet.setJunctionId(std::get<1>(tcdt),std::get<2>(tcdt));
							newTL.push_back(tcdt);

						}
						else
						{
							newTCD.push_back(tcd);
						}
					}
				}
			}
		};
	}
}