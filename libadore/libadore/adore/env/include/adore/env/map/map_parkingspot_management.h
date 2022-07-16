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

#include "adore/env/borderbased/parkingspotset.h"
#include "map_auxiliary.h"

namespace adore
{
	namespace env
	{
		/**
		 * @brief automatically manage parking spot based on current vehicle position and last state of object
		 * 
		 */
		class MapParkingSpotManagement
		{
			adore::env::BorderBased::ParkingSpotSet m_globalSet;
			adore::env::BorderBased::ParkingSpotSet m_localSet;

		public:
			/**
			 * @brief initialization routine with global parking spot set
			 * 
			 * @param globalSet 
			 */
			void init(adore::env::BorderBased::ParkingSpotSet &globalSet)
			{
				m_globalSet = globalSet;
			}

			/**
			 * @brief clear the local set
			 * 
			 */
			void reset()
			{
				m_localSet.clear();
			}

			/**
			 * @brief get new and outdated parking spots based on vehicle position
			 * 
			 * @param x 
			 * @param y 
			 * @param r 
			 * @param newSpots 
			 * @param outdatedSpots 
			 * @param MAX_SEND_NUMBER 
			 */
			void run(double x, double y, double r, std::vector<adore::env::BorderBased::ParkingSpot> &newSpots, std::vector<adore::env::BorderBased::ParkingSpot> &outdatedSpots, int MAX_SEND_NUMBER = 5)
			{
				auto corners = MAP_AUX::getCornerPoints(x,y,r);

				newSpots.clear();
				outdatedSpots.clear();

				/* collect ParkingSpots that will be removed */
				outdatedSpots = m_localSet.itPair2ParkingSpotVector(m_localSet.getParkingSpotsOutsideRegion(corners.at(0),corners.at(1),corners.at(2),corners.at(3)));
				for(auto it = outdatedSpots.begin(); it!=outdatedSpots.end(); it++)
				{
					m_localSet.eraseParkingSpot(*it);
				}

				/* collect new ParkingSpots */
				int newCount = 0;
				for(auto visible = m_globalSet.getParkingSpotsInRegion(corners.at(0),corners.at(1),corners.at(2),corners.at(3)); visible.first!=visible.second; visible.first++)
				{
					adore::env::BorderBased::ParkingSpot ps(adore::env::BorderBased::Coordinate(visible.first->first),visible.first->second);
					if(!m_localSet.hasParkingSpot(ps))
					{
						m_localSet.insertParkingSpot(ps);
						newSpots.push_back(ps);
						newCount++;
						if(newCount>=MAX_SEND_NUMBER)
						{
							break;
						}
					}
				}

				
			}
		};
	}
}