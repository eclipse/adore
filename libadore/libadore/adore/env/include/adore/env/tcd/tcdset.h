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

#include <adore/env/tcd/trafficcontroldevice.h>
#include <adore/env/tcd/tcdtrafficlightset.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <unordered_map>
#include <list>

namespace adore
{
	namespace env
	{
		/**
		 * TCD Set that maps from coordinate to a list of TCDs with rTree from BOOST
		 */
		class TCDSet
		{
		private:
			/* R-TREE STUFF */
			/* implement on higher level! rTree-implementation probably belongs as template into adore_MAD */
			typedef boost::geometry::model::box<env::BorderBased::Coordinate::boost_point> boost_box;
			
			template<typename T1, typename T2>
			struct itpair
			{
				T1 first;
				T2 second;
				itpair(T1 first,T2 second):first(first),second(second){}
				T1& current(){return first;}
				T2& end(){return second;}
			};

			template<typename value_type,typename Tfirst>
			struct my_equal 
			{ 
				typedef bool result_type; 
				result_type operator() (value_type const& v1, value_type const& v2) const 
				{ 
					return boost::geometry::equals<Tfirst,Tfirst>(v1.first, v2.first) && v1.second == v2.second;
				} 
			}; 

			/* boost_point, tcdID */
			typedef std::pair<env::BorderBased::Coordinate::boost_point,int> idxCoordinate2tcdID;

			typedef boost::geometry::index::rtree<	idxCoordinate2tcdID,
													boost::geometry::index::quadratic<16>,
													boost::geometry::index::indexable<idxCoordinate2tcdID>,
													my_equal<idxCoordinate2tcdID,env::BorderBased::Coordinate::boost_point>
												 > Coordinate2tcdIDs_RT;

			typedef std::pair<std::unordered_map<int, env::TrafficControlDevice*>::iterator,std::unordered_map<int, env::TrafficControlDevice*>::iterator> TCDIteratorPair;
			typedef itpair<Coordinate2tcdIDs_RT::const_query_iterator,Coordinate2tcdIDs_RT::const_query_iterator> ItCoordinate2tcdID;
			
		protected:			
			/* get TCD_ID from coordinate */
			Coordinate2tcdIDs_RT m_coordinate2tcdIDs;

			/* get TCD from ID */
			std::unordered_map<int, env::TrafficControlDevice*> m_id2TCD;
			adore::env::TCDTrafficLightSet m_tcdTlSet;

			/* constants */
			double m_guard; //min/max value

			bool m_isOwner;

		public:
			/**
			 * Constructor
			 */
			TCDSet()
			{
				m_guard = 1e99;
				m_isOwner = true;
			}
			
			/**
			 * Destructor
			 * clear xodrTCDSet
			 */
			virtual ~TCDSet()
			{
				clear();
			}

			/**
			 * setIsOwner
			 * if xodrTCDSet is responsible for memory management, set true
			 */
			void setIsOwner(bool b)
			{
				m_isOwner = b;
			}


			/**
			 * getIsOwner
			 * returns true if xodrTCDSet is responsible for memory management
			 */
			bool getIsOwner()
			{
				return m_isOwner;
			}

			/**
			 * insert new TCD
			 * as for now, if TCD already exists, replace it
			 * returns false if TCD was replaced
			 */
			bool insertTCD(env::TrafficControlDevice* tcd)
			{
				bool returnValue = true;
				
				/* tcd already exists */
				if(hasTCD(tcd->getID()))
				{
					/* @TODO What to do when tcd already exists? As for now: replace the old one */
					eraseTCD(tcd->getID());
					returnValue = false;
				}
				m_id2TCD.insert(std::make_pair(tcd->getID(),tcd));
				if(tcd->getType()==adore::env::TrafficControlDevice::TRAFFIC_LIGHT)
				{
					m_tcdTlSet.addTcd(tcd);
				}
				m_coordinate2tcdIDs.insert(std::make_pair(tcd->getCoordinate().getBoostPoint(),tcd->getID()));
				return returnValue;
				
			}

			void setMovementId(int tcdId, int movementId)
			{
				m_tcdTlSet.setMovementId(tcdId, movementId);
			}

			void setJunctionId(int movementId, int junctionId)
			{
				m_tcdTlSet.setJunctionId(movementId, junctionId);
			}

			/**
			 * erase
			 * DO NOT USE WHILE ITERATING WITH ITERATORS OVER RTree!
			 * erases specifed TCD from xodrTCDSet, if xodrTCDSet is owner also deletes TCD
			 * returns false if TCD does not exist
			 */
			bool eraseTCD(int tcdID)
			{
				/* TCD does not exist */
				if(!hasTCD(tcdID))
				{
					return false;
				}

				env::TrafficControlDevice* tcdToRemove = m_id2TCD[tcdID];
				m_coordinate2tcdIDs.remove(std::make_pair(tcdToRemove->getCoordinate().getBoostPoint(), tcdToRemove->getID()));
				m_id2TCD.erase(tcdID);
				m_tcdTlSet.erase(tcdID);
				if(getIsOwner()) delete tcdToRemove;
				return true;
			}
			
			/**
			 * clear
			 * clear xodrTCDSet, also deletes TCD if owner
			 */
			void clear()
			{
				if(getIsOwner())
				{
					auto iterators = getAllTCDs();
					for(auto it = iterators.first; it != iterators.second; it++)
					{
						delete it->second;
					}
				}
				m_id2TCD.clear();
				m_coordinate2tcdIDs.clear();
				m_tcdTlSet.clear();
				// m_region2TCDs.clear();
			}

			/**
			 * getAllTCDs
			 * get begin() and end() iterators for all TCDs in xodrTCDSet
			 */
			TCDIteratorPair getAllTCDs()
			{
				return TCDIteratorPair(m_id2TCD.begin(), m_id2TCD.end());
			}

			/**
			 * getTCDsInRegion
			 * get begin() and end() iterators for all TCDs in defined region
			 */
			ItCoordinate2tcdID getTCDsInRegion(double x0, double x1, double y0, double y1)
			{
				auto it = m_coordinate2tcdIDs.qbegin(boost::geometry::index::intersects(
					boost_box(
						env::BorderBased::Coordinate::boost_point(x0,y0,-m_guard),
						env::BorderBased::Coordinate::boost_point(x1,y1,+m_guard)
					)));
				return ItCoordinate2tcdID(it,m_coordinate2tcdIDs.qend());
			}

			/**
			 * getTCDsOutsideRegion
			 * get begin() and end() iterators for all TCDs outsite of defined region
			 */
			ItCoordinate2tcdID getTCDsOutsideRegion(double x0,double x1, double y0, double y1)
			{
				auto it = m_coordinate2tcdIDs.qbegin(boost::geometry::index::disjoint(
					boost_box(
						env::BorderBased::Coordinate::boost_point(x0,y0,-m_guard),
						env::BorderBased::Coordinate::boost_point(x1,y1,+m_guard)
					)));
				return ItCoordinate2tcdID(it,m_coordinate2tcdIDs.qend());
			}
			/**
			 * removing TCDs in given region without
			 */
			void removeTCDsOutsideRegion(double x0,double x1, double y0, double y1)
			{
				std::vector<int> tcdsToRemove;
				for(auto itpair = getTCDsOutsideRegion(x0,x1,y0,y1); itpair.first!=itpair.second; itpair.first++)
				{
					/* pushing ID of TCD */
					tcdsToRemove.push_back(itpair.first->second);
				}
				for(auto it = tcdsToRemove.begin(); it!=tcdsToRemove.end();it++)
				{
					eraseTCD(*it);
				}
			}

			
			/**
			 * hasTCD
			 * returns true if TCD is in xodrTCDSet
			 */
			bool hasTCD(int tcdID)
			{
				return m_id2TCD.find(tcdID)!=m_id2TCD.end();
			}

			env::TrafficControlDevice* getTCD(int tcdID)
			{
				if(hasTCD(tcdID))
				{
					return m_id2TCD[tcdID];
				}
				std::cout << "TCDSet::getTCD(int tcdID): TrafficControlDevice with ID " << tcdID << " does not exist." << std::endl;
				return nullptr;
			}
			env::TTCDTrafficLightTuple getTCDTrafficLight(int tcdID)
			{
				if(hasTCD(tcdID))
				{
					return m_tcdTlSet.at(tcdID);
				}
				std::cout << "TCDSet::getTCD(int tcdID): TrafficControlDevice with ID " << tcdID << " does not exist." << std::endl;
				return TTCDTrafficLightTuple();
			}
		};
	}
}