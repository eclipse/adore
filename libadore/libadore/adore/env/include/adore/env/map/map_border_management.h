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

#include "adore/env/borderbased/borderset.h"
#include "map_auxiliary.h"

#include <vector>
#include <queue>

namespace adore
{
	namespace env
	{
		struct BorderTypeChangeProfile
		{
			BorderBased::Coordinate start;
			BorderBased::Coordinate end;
			std::vector<BorderBased::BorderType::TYPE> borderTypeProfile;
		};
		/**
		 * @brief Automatically manage local map and necessary updates based on vehicle position and last state of object
		 * 
		 */
		class MapBorderManagement
		{
			// base set that should not be altered
			adore::env::BorderBased::BorderSet m_baseSet;

			// global set from which borders can be removed
			adore::env::BorderBased::BorderSet m_globalSet;

			// local set that represents local map, borders can be removed and added
			adore::env::BorderBased::BorderSet m_localSet;

			// update queue - all borders that are changed
			std::queue<adore::env::BorderBased::BorderID> m_updateQueue;


			/**
			 * @brief initialization of global set
			 * 
			 */
			void initGlobalSet()
			{
				m_baseSet.deepBorderCopy(m_globalSet);
			}
			
			/**
			 * @brief initialization of internal sets
			 * 
			 * @param baseSet 
			 */
			void initBaseSet(adore::env::BorderBased::BorderSet * baseSet)
			{
				baseSet->deepBorderCopy(m_baseSet);
				initGlobalSet();
			}

			// returns vector newBorders with borders that newly appear on the local map and vector outdatedBorders that references all borders that are out of sight
			void do_run(double x, double y, double r, std::vector<adore::env::BorderBased::Border*> &newBorders, std::vector<adore::env::BorderBased::BorderID> &outdatedBorders, std::vector<adore::env::BorderBased::BorderID> &updatedBorders, int MAX_SEND_NUMBER = 40)
			{
				static int MIN_DELETE_NUMBER = 5;
				newBorders.clear();
				outdatedBorders.clear();
				updatedBorders.clear();
				auto corners = MAP_AUX::getCornerPoints(x,y,r);

				// collect borders that will be removed
				auto invisible = m_localSet.getBorderSetOutsideRegion(corners.at(0),corners.at(1),corners.at(2),corners.at(3));
				for(auto it = invisible.begin(); it!=invisible.end(); it++)
				{
					outdatedBorders.push_back((*it)->m_id);
					m_localSet.erase_border((*it)->m_id);
				}

				// collect new borders
				int newCount = 0;
				for(auto visible = m_globalSet.getBordersInRegion(corners.at(0),corners.at(1),corners.at(2),corners.at(3));visible.first!=visible.second;visible.first++)
				{
					adore::env::BorderBased::Border* border = visible.first->second;
					// border type sensitive hasBorder()
					if(!m_localSet.hasBorder(border) && border->m_path!=0)
					{
						newBorders.push_back(border);
						m_localSet.insert_border(new adore::env::BorderBased::Border(*border),true);
						newCount++;
						// if there are no borders to delete, use full buffer, otherwise leave  at least MIN_DELETE_NUMBER for deleting borders
						if(newCount >= MAX_SEND_NUMBER - MIN_DELETE_NUMBER)
						{
							if(!m_updateQueue.empty())
							{
								break;
							}
							else if(newCount >= MAX_SEND_NUMBER)
							{
								break;
							}
						}
					}
				}

				// collect updated borders
				for(int i = 0; i < MAX_SEND_NUMBER - newCount ;i++)
				{
					if(m_updateQueue.empty())
					{
						break;
					}
					m_localSet.insert_border(new adore::env::BorderBased::Border(*m_globalSet.getBorder(m_updateQueue.front())));
					updatedBorders.push_back(m_updateQueue.front());
					m_updateQueue.pop();
				}
			}
			
		public:
			/**
			 * @brief Construct a new Map Border Management object
			 * 
			 */
			MapBorderManagement()
			{
			}

			/**
			 * @brief Get successors of a given border from global set
			 * 
			 * @param b 
			 * @return env::BorderBased::itCoordinate2Border 
			 */
			env::BorderBased::itCoordinate2Border getSuccessors(env::BorderBased::Border* b)
			{
				return m_globalSet.getSuccessors(b);
			}

			/**
			 * @brief Get borders at given point
			 * 
			 * @param x 
			 * @param y 
			 * @return env::BorderBased::BorderSubSet 
			 */
			env::BorderBased::BorderSubSet getBordersAtPoint(double x, double y)
			{
				return m_globalSet.getBordersAtPoint(x,y);
			}

			/**
			 * @brief initialization routine with base map
			 * 
			 * @param baseSet 
			 */
			void init(adore::env::BorderBased::BorderSet * baseSet)
			{
				initBaseSet(baseSet);
			}

			/**
			 * @brief Direct access to border in global map for auxiliary uses like plotting
			 * 
			 * @param bId 
			 * @return adore::env::BorderBased::Border* 
			 */
			adore::env::BorderBased::Border* getBorder(adore::env::BorderBased::BorderID &bId)
			{
				return m_globalSet.getBorder(bId);
			}

			/**
			 * @brief clear local map
			 * 
			 */
			void clearLocalMap()
			{
				m_localSet.clear();
			}

			/**
			 * @brief undo all changes to global map and clears local map
			 * 
			 */
			void reset()
			{
				std::queue<adore::env::BorderBased::BorderID>().swap(m_updateQueue);
				auto its = m_baseSet.getAllBorders();
				for(;its.first!=its.second;its.first++)
				{
					auto b = its.first->second;
					if(!m_globalSet.hasBorder(b))
					{
						m_updateQueue.push(b->m_id);
					}
				}
				m_globalSet.clear();
				m_localSet.clear();
				initGlobalSet();
			}

			adore::env::BorderBased::BorderSet * getGlobalMap()
			{
				return &m_globalSet;
			}

			/**
			 * @brief removes borders in region from global and local map
			 * 
			 * @param x0 
			 * @param x1 
			 * @param y0 
			 * @param y1 
			 */
			void deleteBordersInRegion(double x0, double x1, double y0, double y1)
			{
				auto itpair = m_baseSet.getBordersInRegion(x0,x1,y0,y1);
				for(;itpair.first!=itpair.second;itpair.first++)
				{
					auto border = itpair.first->second;
					if(!border->typeIsChangeable()) continue;

					auto b = itpair.first->second->m_id;
					if(m_localSet.hasBorder(b))
					{
						m_localSet.erase_border(b);
						auto newB = new adore::env::BorderBased::Border(*border);
						newB->deleteType();
						m_localSet.insert_border(newB);
					}
					if(m_globalSet.hasBorder(b))
					{
						m_globalSet.erase_border(b);
						auto newB = new adore::env::BorderBased::Border(*border);
						newB->deleteType();
						m_globalSet.insert_border(newB);
						// erase ALL deleted borders via feed
						m_updateQueue.push(b);
					}
				}
			}

			/**
			 * @brief add formerly deleted borders in a given region into global map from base map
			 * 
			 * @param x0 
			 * @param x1 
			 * @param y0 
			 * @param y1 
			 */
			void addBordersInRegion(double x0, double x1, double y0, double y1)
			{
				auto itpair = m_baseSet.getBordersInRegion(x0,x1,y0,y1);
				for(;itpair.first!=itpair.second;itpair.first++)
				{
					auto b = itpair.first->second;
					auto newB = new adore::env::BorderBased::Border(*b);
					m_globalSet.insert_border(newB,true);
					m_updateQueue.push((*b).m_id);
				}
			}

			// full run method, return new borders, old borders and deleted borders
			void run(double x, double y, double r, std::vector<adore::env::BorderBased::Border*> &newBorders, std::vector<adore::env::BorderBased::BorderID> &outdatedBorders, std::vector<adore::env::BorderBased::BorderID> &updatedBorders, int MAX_SEND_NUMBER = 40)
			{
				do_run(x, y, r, newBorders, outdatedBorders, updatedBorders, MAX_SEND_NUMBER);
			}

			/**
			 * @brief reduced run method, receive new visible borders and now outdated borders
			 * 
			 * @param x 
			 * @param y 
			 * @param r 
			 * @param newBorders 
			 * @param outdatedBorders 
			 * @param MAX_SEND_NUMBER 
			 */
			void run(double x, double y, double r, std::vector<adore::env::BorderBased::Border*> &newBorders, std::vector<adore::env::BorderBased::BorderID> &outdatedBorders, int MAX_SEND_NUMBER = 40)
			{
				std::vector<adore::env::BorderBased::BorderID> dummyDelete;
				do_run(x, y, r, newBorders, outdatedBorders, dummyDelete, MAX_SEND_NUMBER);
			}

			/**
			 * @brief change border type of border at exactly the given position
			 * 
			 * @param t 
			 * @param x 
			 * @param y 
			 */
			void changeBorderType(adore::env::BorderBased::BorderType::TYPE t, double x, double y)
			{
				auto vec = m_globalSet.getBordersAtPoint(x,y);
				for(auto b = vec.begin(); b!= vec.end(); b++)
				{
					if((*b)->typeIsChangeable())
					{
						(*b)->m_type = t;
						m_updateQueue.push((*b)->m_id);
					}
				}
				
			}

			/**
			 * @brief change border type of border identified by id
			 * 
			 * @param id 
			 * @param t 
			 */
			void changeBorderType(adore::env::BorderBased::BorderID id, adore::env::BorderBased::BorderType::TYPE t)
			{
				if(m_globalSet.getBorder(id)->typeIsChangeable())
				{
					m_globalSet.getBorder(id)->m_type = t;
					m_updateQueue.push(id);
					LOG_T("Update queue: %i", m_updateQueue.size());
				}
			}

			/**
			 * @brief change border type of borders in region to given type
			 * 
			 * @param t 
			 * @param x0 
			 * @param x1 
			 * @param y0 
			 * @param y1 
			 */
			void changeBorderType(adore::env::BorderBased::BorderType::TYPE t, double x0, double x1, double y0, double y1)
			{
				auto itpair = m_globalSet.getBordersInRegion(x0,x1,y0,y1);
				for(;itpair.first!=itpair.second;itpair.first++)
				{
					if(itpair.first->second->typeIsChangeable())
					{
						itpair.first->second->m_type = t;
						m_updateQueue.push(itpair.first->second->m_id);
					}
				}
			}

			/**
			 * @brief change border type based on BorderTypeChangeProfile struct
			 */
			void changeBorderType(BorderTypeChangeProfile btcp)
			{
				LOG_T("Enter function.");
				// profiles are ordered from leftmost to rightmost
				BorderBased::BorderSubSet startingProfile, endingProfile;

				// find starting profile, ordered from left to right
				auto startingBorders = m_globalSet.getBordersAtPoint(btcp.start.m_X, btcp.start.m_Y);
				for(auto b = startingBorders.begin(); b!=startingBorders.end(); b++)
				{
					if((*b)->typeIsChangeable())
					{
						startingProfile = m_globalSet.getIndexableNeighbors((*b));
						break;
					}
				}
				// find ending profile, ordered from left to right
				auto endingBorders = m_globalSet.getBordersAtPoint(btcp.end.m_X, btcp.end.m_Y);
				for(auto b = endingBorders.begin(); b!=endingBorders.end(); b++)
				{
					if((*b)->typeIsChangeable())
					{
						endingProfile = m_globalSet.getIndexableNeighbors((*b));
						break;
					}
				}
				LOG_T("Border Sizes: %i, %i", startingProfile.size(), endingProfile.size());
				// check for inconsistencies
				if(startingProfile.size()==0 or endingProfile.size()==0 or startingProfile.size()!=endingProfile.size() or btcp.borderTypeProfile.size()<startingProfile.size())
				{
					LOG_E("BorderTypeChangeProfile is inconsistent.");
					return;
				}
				// find path per border in profile
				for(int i=0; i<startingProfile.size(); i++)
				{
					auto startingBorder = startingProfile.at(i);
					auto endingBorder = endingProfile.at(i);
					BorderBased::BorderSubSet targetBorder;
					targetBorder.push_back(endingBorder);
					std::deque<BorderBased::BorderID> path;
					m_globalSet.findPathBetweenBorders(startingBorder->m_id,path,targetBorder);
					if(path.size()==0)
					{
						LOG_W("BorderTypeChangeProfile path between borders was not found.");
					}
					LOG_T("Path size: %i", path.size());
					// change border type of path
					for(auto id=path.begin(); id!=path.end(); id++)
					{
						// border type profile is also ordered from left to right
						this->changeBorderType(*id,btcp.borderTypeProfile.at(i));
						LOG_T("%s\ttype:", (*id).toString().c_str(), static_cast<int>(btcp.borderTypeProfile.at(i)));
					}
				}
			}
		};
	}
}