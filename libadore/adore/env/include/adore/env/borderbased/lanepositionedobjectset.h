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
#include <adore/env/borderbased/border.h>
#include <adore/env/borderbased/borderset.h>
#include <adore/env/borderbased/alanepositionedobject.h>
#include <vector>
#include <unordered_map>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * @brief This class represents a set of objects that are positioned by LanePosition
			 * 
			 */
			class LanePositionedObjectSet
			{
			

			public:
				typedef ALanePositionedObject object;
				typedef std::unordered_multimap<BorderID,object*, BorderIDHasher> BorderID2Object;
				typedef BorderID2Object::iterator ObjectIterator;
				typedef std::pair<ObjectIterator,ObjectIterator> ObjectIteratorPair;
			private:
				BorderID2Object borderId2Object; /**< mapping the border ids with the objects */
				bool m_isOwner; /**< indicates whether object is owned */

			public:
			/**
			 * @brief Construct a new LanePositionedObjectSet object
			 * 
			 */
				LanePositionedObjectSet()
				{
					m_isOwner = true;
				}
				/**
				 * @brief Destroy the LanePositionedObjectSet object
				 * 
				 */
				virtual ~LanePositionedObjectSet()
				{
					clear();
				}
				/**
				 * @brief Set the owner flag
				 * 
				 * @param isOwner 
				 */
				void setIsOwner(bool isOwner)
				{
					m_isOwner = isOwner;
				}
				/**
				 * @brief Clear the LanePositionedObjectSet
				 * 
				 */
				void clear()
				{
					if(m_isOwner)
					{
						for(auto it = borderId2Object.begin();it!=borderId2Object.end();it++)
						{
							delete it->second;
						}
					}
					borderId2Object.clear();
				}
				/**
				 * @brief Discard distant objects
				 * 
				 * @param x x-coordinate of the reference point
				 * @param y y-coordinate of the reference point
				 * @param distance maximal distance to for the objects to keep (default: 100)
				 */
				void discard_distant_objects(double x, double y, double distance = 100.0)
				{
					for (auto it = getAllObjects().first; it != getAllObjects().second; )
					{	
						auto checkCoord = Coordinate(x,y,0.0);

						if(it->first.m_first.distance(checkCoord)>=distance
							&& it->first.m_last.distance(checkCoord) >= distance)
						//if (!it->first.m_first.isInArea(x-distance,x+distance,y-distance, y+distance))
						{
							delete it->second;
							it=borderId2Object.erase(it);
							continue;
						}
						it++;
					}
				}
				/**
				 * @brief Discard distant objects
				 * 
				 * The removed Objects are written into a vector
				 * @param x x-coordinate of the reference point
				 * @param y y-coordinate of the reference point
				 * @param removedObjects list of removed objects
				 * @param distance maximal distance to for the objects to keep (default: 100)
				 */
				template <typename T>
				void discard_distant_objects(double x, double y, std::vector<T> &removedObjects, double distance = 100.0)
				{
					for (auto it = getAllObjects().first; it != getAllObjects().second; )
					{	
						auto checkCoord = Coordinate(x,y,0.0);

						if(it->first.m_first.distance(checkCoord)>=distance
							&& it->first.m_last.distance(checkCoord) >= distance)
						//if (!it->first.m_first.isInArea(x-distance,x+distance,y-distance, y+distance))
						{
							removedObjects.push_back(*(dynamic_cast<T*>((*it).second)));

							delete it->second;
							it=borderId2Object.erase(it);
							continue;
						}
						it++;
					}
				}
				/**
				 * @brief Insert a new object
				 * 
				 * @param obj object to insert
				 * @param remove_duplicates indicates whether duplicates should be removed (default: true)
				 * @param precision precision for checking for duplicates (default: 0.5)
				 */
				void insert_object(object* obj,bool remove_duplicates=false,double precision=0.5)
				{
					if(remove_duplicates)
						erase_object(obj->getLanePosition(),precision);
					
					borderId2Object.insert(std::make_pair(obj->getLanePosition().m_rightID,obj));
				}
				/**
				 * @brief Erase objects on a certain LanePosition
				 * 
				 * @param position LanePosition where objects should be erased
				 * @param precision the Position is compared with this precision (default: 0.5)
				 */
				void erase_object(const LanePosition& position,double precision=0.5)
				{
					for(auto it = borderId2Object.equal_range(position.m_rightID);it.first!=it.second;)
					{
						if((std::abs)(it.first->second->getLanePosition().m_progress-position.m_progress)<precision)
						{
							if(m_isOwner)delete it.first->second;
							it.first = borderId2Object.erase(it.first);
							continue;
						}
						it.first++;
					}
				}
				/**
				 * @brief Erase Objects that are positioned on certain borders
				 * 
				 * @param borderSet vector of borders on which objects are removed
				 */
				void eraseObjectsBorderBased(const std::vector<Border*> borderSet)
				{
					for(auto it = borderSet.begin();it!=borderSet.end();it++)//borderSet
					{
						for(auto it2 = borderId2Object.equal_range((*it)->m_id);it2.first!=it2.second;)//borderid2object
						{
							if(m_isOwner)delete it2.first->second;
							it2.first = borderId2Object.erase(it2.first);
						}
					}
				}
				/**
				 * @brief Erase Objects that are positioned on certain borders given by their BorderIDs
				 * 
				 * @param borderIDSet vector of borders given by BorderIDs on which objects are removed
				 */
				void eraseObjectsBorderBased(const std::vector<BorderID> borderIDSet)
				{
					for(auto it = borderIDSet.begin();it!=borderIDSet.end();it++)//borderIDVector
					{
						for(auto it2 = borderId2Object.equal_range(*it);it2.first!=it2.second;)//borderid2object
						{
							if(m_isOwner)delete it2.first->second;
							it2.first = borderId2Object.erase(it2.first);
						}
					}
				}
				/**
				 * @brief Check whether at least one object is contained that is positioned on a certain Border
				 * 
				 * @param borderID specifies the border
				 * @return true if at least one object in the set is positioned with the given border
				 * @return false if no object is positioned with the given border
				 */
				bool hasObjects(const BorderID& borderID)
				{
					if(borderId2Object.size() == 0)
						return false;

					return borderId2Object.count(borderID)>0;
				}
				/**
				 * @brief Get the begin()- and end()-iterator for the whole set
				 * 
				 * @return ObjectIteratorPair begin()- and end()-iterator
				 */
				ObjectIteratorPair getAllObjects()
				{
					return ObjectIteratorPair(borderId2Object.begin(),borderId2Object.end());
				}
				/**
				 * @brief Get the begin()- and end()-iterator for objects that are positioned on a certain border
				 * 
				 * @param borderID specifies the border
				 * @return ObjectIteratorPair begin()- and end()-iterator
				 */
				ObjectIteratorPair getObjects(const BorderID& borderID)
				{
					return borderId2Object.equal_range(borderID);
				}
				/**
				 * @brief Get the objects that are positioned on a certain LanePosition
				 * 
				 * @param position LanePosition
				 * @param precision precision that is used for the comparision of the LanePositions
				 * @return std::vector<object*> vector of objects that are positioned on the specified LanePosition
				 */
				std::vector<object*> getObjects(const LanePosition& position,double precision=0.5)
				{
					std::vector<object*> result;
					for( auto itpair=getObjects(position.m_rightID);itpair.first!=itpair.second;itpair.first++ )
					{
						if((std::abs)(itpair.first->second->getLanePosition().m_progress-position.m_progress)<precision)
						{
							result.push_back(itpair.first->second);
						}
					}
					return result;
				}
				/**
				 * @brief Check whether the set holds at least one object on a certain LanePosition
				 * 
				 * @param position LanePosition
				 * @param precision precision for the comparison of the LanePositions
				 * @return true if there is at least one object at the specified LanePosition 
				 * @return false if no object is at the specified LanePosition
				 */
				bool hasObjects(const LanePosition& position,double precision=0.5)
				{
					for( auto itpair=getObjects(position.m_rightID);itpair.first!=itpair.second;itpair.first++ )
					{
						if((std::abs)(itpair.first->second->getLanePosition().m_progress-position.m_progress)<precision)
						{
							return true;
						}
					}
					return false;
				}
				/**
				 * @brief Erase objects that are positioned that are outside a specified circle
				 * 
				 * @param borderSet set of Borders. Objects that are positioned on these borders are not deleted.
				 * @param center Coordinates of the center point of the circle
				 * @param radius radius of the circle
				 */
				void erase_objectsWithUnknownBordersOutsideRadius(BorderSet* borderSet,Coordinate center,double radius)
				{
					for( auto it = borderId2Object.begin();it!=borderId2Object.end(); )
					{
						auto bid = it->first;
						if(!borderSet->hasBorder(bid))
						{
							if(bid.m_first.distance(center)>radius
							 && bid.m_last.distance(center)>radius)
							{
								if(m_isOwner)delete it->second;
								it = borderId2Object.erase(it);
								continue;
							}
						}
						it++;
					}
				}
			};

		}
	}
}