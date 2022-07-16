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
 *    Stephan Lapoehn - initial implementation and API
 ********************************************************************************/

#pragma once

#include <map>
#include <tuple>
#include <adore/env/tcd/trafficcontroldevice.h>

namespace adore
{
	namespace env
	{
		typedef std::tuple<TrafficControlDevice*, int		, int>				TTCDTrafficLightTuple;
		//		std::tuple<trafficControlDevice, movementId, intersectionId>> 
		/**
		 * store the complete information needed for mapping with MAPEM and SPATEM, made to be filled through xodr
		 */
		class TCDTrafficLightSet : public std::map<int,TTCDTrafficLightTuple>
		{
			/**
			 * ControllerId		1	-> * signalId
			 *	iterate over controllers and update controllerId of tuple: setControllerId(int tcdId, int controllerId)
			 * junctionId		1	-> * controllerId
			 *	iterate over junctions and update junctionId of tuple: setIntersectionId(int controllerId, int intersectionId)
			 * 
			 * 
			 * updateControllerOfTcd(int tcdId, int controllerId)
			 * updateIntersectionOfTcd(int tcdId, int intersectionId)
			 * addTcd(TrafficControlDevice* tcd)
			 * TrafficControlDevice* getTcd(int tcdId)
			 * int getControllerId(int tcdId)
			 * int getIntersectionId(int tcdId)
			 */
		public:
			void addTcd(TrafficControlDevice* tcd)
			{
				this->insert(std::make_pair(tcd->getID(), TTCDTrafficLightTuple(tcd, -1, -1)));
			}

			void setMovementId(int tcdId, int movementId)
			{
				if(this->count(tcdId)>0)
				{
					std::get<1>(this->at(tcdId)) = movementId;
				}
			}

			void setJunctionId(int movementId, int junctionId)
			{
				for(auto it = this->begin(); it!=this->end(); it++)
				{
					if(std::get<1>(it->second)==movementId)
					{
						std::get<2>(it->second)=junctionId;
					}
				}
			}

			TrafficControlDevice* getTcdObj(int tcdId)
			{
				return std::get<0>(this->at(tcdId));
			}

			int getMovementId(int tcdId)
			{
				return std::get<1>(this->at(tcdId));
			}

			int getIntersectionId(int tcdId)
			{
				return std::get<2>(this->at(tcdId));
			}
		};
	}
}