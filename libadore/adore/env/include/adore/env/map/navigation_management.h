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

#include "adore/env/map/map_border_management.h"
#include "adore/env/borderbased/navigation_border_observer.h"
#include "adore/env/borderbased/coordinate.h"
#include "adore/env/borderbased/bordergraph.h"

#include "adore/mad/com_patterns.h"
#include "adore/mad/csvlog.h"

namespace adore
{
	namespace env
	{
		class NavigationManagement
		{
			MapBorderManagement m_map;
			double m_maxCost;
			bool m_goalChanged;
			double m_laneChangePenalty;
			
			BorderBased::Coordinate m_destination;
			BorderBased::BorderGraph::BorderCostMap m_borderCosts;
			BorderBased::NavigationBorderObserver m_navBorderObserver;
      
			BorderBased::Border * getBorderFromCurrentTarget()
			{
				/* border is found at exact target position */
				adore::env::BorderBased::Border* matchedBorder = nullptr;
				double destinationX[] = { m_destination.m_X };
				double destinationY[] = { m_destination.m_Y};
				BorderBased::BorderSubSet borders;
				m_map.getGlobalMap()->matchLanesInRegion(destinationX,destinationY,1,borders);
				bool borderFound = false;
				
				if(borders.size()>0)
				{
					for(auto border = borders.begin();border!=borders.end();border++)
					{
						if((*border)->m_type == adore::env::BorderBased::BorderType::DRIVING)
						{
							matchedBorder = *border;
							borderFound = true;
						}
					}
				}
				
				/* no border is found at exact target position */
				if(!borderFound)
				{
					LOG_W("Dadore_CORE_Navigation.cpp: No borders were found for given point. Trying to use closest border...");
					for(double distance = 1.0;;distance = distance +10.0)
					{
						auto bordersInArea = m_map.getGlobalMap()->getBordersInRegion(m_destination.m_X-distance,m_destination.m_X+distance,m_destination.m_Y-distance,m_destination.m_Y+distance);
						auto minDistance = distance;
						for(;bordersInArea.first!=bordersInArea.second;bordersInArea.first++)
						{
							double distanceToBorder = distance;
							auto curBorder = bordersInArea.first->second;
							if(curBorder->m_type!=adore::env::BorderBased::BorderType::DRIVING)
							{
								continue;
							}
							curBorder->m_path->getClosestParameter(m_destination.m_X,m_destination.m_Y,1,2,distanceToBorder);
							if(distanceToBorder<minDistance)
							{
								matchedBorder = curBorder;
								minDistance = distanceToBorder;
								borderFound = true;
							}


						}
						if(borderFound) break;
						if(distance>1000)
						{
							LOG_E("No border to navigate to was found.");
							break;
						}
					}
				}
				if(matchedBorder==nullptr)
				{
					LOG_E("Dadore_CORE_Navigation.cpp: No border to navigate to was found.");
				}
				else
				{
					LOG_I("Found border to navigate to.");
				}
				return matchedBorder;
			}

			void setMaxCost()
			{
				/* costs are negative */
				m_maxCost = 0;
				for(auto it = m_borderCosts.begin(); it!=m_borderCosts.end(); it++)
				{
					if(it->second < m_maxCost)
					{
						m_maxCost = it->second;
					}
				}
				m_maxCost = std::abs(m_maxCost);
			}
			

		public:
			NavigationManagement()
			{
				m_destination = BorderBased::Coordinate();
				m_borderCosts = BorderBased::BorderGraph::BorderCostMap();
				m_maxCost = -1;
				m_goalChanged = true;
				m_laneChangePenalty = 100.0;
			}

			void setLaneChangePenalty(double value)
			{
				m_laneChangePenalty = value;
			}

			void addFeed(adore::mad::AFeed<adore::env::BorderBased::Border> * feed)
			{
				m_navBorderObserver.addFeed(feed);
			}

			double getMaxCost()
			{
				return m_maxCost;
			}

			bool hasMap()
			{
				return m_map.getGlobalMap()->size() > 0;
			}

			void init(BorderBased::BorderSet * set)
			{
				m_map.init(set);
				m_navBorderObserver.init(m_map.getGlobalMap());
			}

			/*
			 * update:
			 * - check for target change
			 * - check for map changes
			 */
			void update(BorderBased::Coordinate newDestination, bool destinationValid)
			{
				/* check for map changes via lane border feed */
				m_navBorderObserver.update();

				/* if destination changed and is valid, use new destination */
				if(destinationValid && newDestination != m_destination)
				{
					setDestination(newDestination);
					m_goalChanged = true;
				}
				/* if destination didn't change but map did, replan with old destination */
				else if(m_navBorderObserver.navigationReplanningNecessary())
				{
					setDestination(m_destination);
					m_goalChanged = true;
				}
				else
				{
					m_goalChanged = false;
				}
			}

			bool goalChanged()
			{
				return m_goalChanged;
			}

			/*
			 * reset:
			 * - reset map and navBorderObserver to base set
			 */
			void reset()
			{
				m_map.reset();
				m_navBorderObserver.clear();
				m_navBorderObserver.init(m_map.getGlobalMap());
			}

			/*
			 * clear:
			 * - clear local border set to re-send navigation information
			 */
			void clear()
			{
				m_map.clearLocalMap();
			}

			/* change target, reset border costs, reset localmap to send all new updates */
			void setDestination(BorderBased::Coordinate c)
			{
				/* set new target */
				m_destination = c;
				
				/* clear current border costs */
				m_borderCosts.clear();
				
				/* prepare graph usage */
				BorderBased::BorderGraphCostWithLaneChanges * cost_function = new BorderBased::BorderGraphCostWithLaneChanges();
				cost_function->setLaneChangePenalty(m_laneChangePenalty);
				auto targetBorder = getBorderFromCurrentTarget();
				if(targetBorder!=nullptr)
				{
					BorderBased::Node startNode(targetBorder);
				
					/* build graph and generate new costs, use navBorderObserver to generate graph */
					BorderBased::BorderGraph graph;
					graph.init(dynamic_cast<BorderBased::BorderSet*>(&m_navBorderObserver),cost_function);
					graph.djikstra(&startNode, &m_borderCosts);
					setMaxCost();
				}
				
				/* clear local border set to send new update */
				m_map.clearLocalMap();
			}

			void run(double x, double y, double r, std::vector<adore::env::BorderBased::Border*> &newBorders, std::vector<adore::env::BorderBased::BorderID> &outdatedBorders, int MAX_SEND_NUMBER = 40)
			{
				m_map.run(x,y,r,newBorders,outdatedBorders,MAX_SEND_NUMBER);
			}

			double getBorderCost(BorderBased::BorderID id)
			{
				auto entry = m_borderCosts.find(id);
				
				/* border costs are stored as negative values, transform to positive here */
				if(entry != m_borderCosts.end())
				{
					return std::abs(entry->second);
				}
				return std::numeric_limits<double>::max();
			}

			adore::env::BorderBased::Border * getBorder(adore::env::BorderBased::BorderID id)
			{
				return m_map.getGlobalMap()->getBorder(id);
			}

			adore::env::BorderBased::BorderSet * getGlobalMap()
			{
				return m_map.getGlobalMap();
			}
		};
	}
}