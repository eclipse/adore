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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/


#pragma once

#include <adore/env/borderbased/coordinate.h>
#include <adore/env/borderbased/parkingspot.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <unordered_map>
#include <list>
#include <vector>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{

			/**
			 * parking spot set that maps from adore::env::BorderBased::Coordinate to (double) heading with rTree from BOOST
			 */
			class ParkingSpotSet
			{
			private:
				/* R-TREE STUFF */
				/* box to search in for Parking Spots */

				template<typename T1, typename T2>
				struct itpair
				{
					T1 first;
					T2 second;
					itpair(T1 first,T2 second):first(first),second(second){}
					T1& current(){return first;}
					T2& end(){return second;}
				};

				/* equal for parking spots -> boost points are equal, heading doesn't matter */
				template<typename value_type,typename Tfirst>
				struct my_equal 
				{ 
					typedef bool result_type; 
					result_type operator() (value_type const& v1, value_type const& v2) const 
					{ 
						return boost::geometry::equals<Tfirst,Tfirst>(v1.first, v2.first);
					} 
				}; 

				/* boost_point + heading = parking spot */
				typedef std::pair<adore::env::BorderBased::Coordinate::boost_point,double> idxCoordinate2heading;

				typedef boost::geometry::index::rtree<	idxCoordinate2heading,
														boost::geometry::index::quadratic<16>,
														boost::geometry::index::indexable<idxCoordinate2heading>,
														my_equal<idxCoordinate2heading,env::BorderBased::Coordinate::boost_point>
													 > Coordinate2heading_RT;

				typedef itpair<Coordinate2heading_RT::const_query_iterator,Coordinate2heading_RT::const_query_iterator> ItPairCoordinate2heading;
			
			protected:			
				/* get heading from coordinate, heading + coordinate = parking spot definition */
				Coordinate2heading_RT m_coordinate2heading;

				/* constants */
				double m_guard; //min/max value
			
				/* protected functions */
				bool hasParkingSpot(adore::env::BorderBased::Coordinate::boost_point bp)
				{
					auto it = m_coordinate2heading.qbegin(boost::geometry::index::nearest(bp,1));
					if(it!=m_coordinate2heading.qend() && Coordinate(bp)==Coordinate(it->first)) // boost::geometry::equals(bp,it->first)) -> this doesn't seem to be reliable enough, a lot of false negatives
					{
						return true;
					}
					return false;
				}
				/**
				 * erase(boostPoint bp)
				 * DO NOT USE WHILE ITERATING WITH ITERATORS OVER RTree!
				 * no check for existance
				 */
				bool eraseParkingSpot(adore::env::BorderBased::Coordinate::boost_point bp)
				{
					if(hasParkingSpot(bp))
					{
						auto it = m_coordinate2heading.qbegin(boost::geometry::index::nearest(bp,1));
						if(it!= m_coordinate2heading.qend() && boost::geometry::equals(bp,it->first))
						{
							m_coordinate2heading.remove(*it);
							return true;
						}
					}
					return false;
				}

			public:
				/**
				 * Constructor
				 */
				ParkingSpotSet()
				{
					m_guard = 1e99;
				}
			
				/**
				 * Destructor
				 * clear ParkingSpotSet
				 */
				virtual ~ParkingSpotSet()
				{
					clear();
				}

				/* 
				 * insert Parking Spot
				 * as for now, if Parking Spot already exists, replace it
				 * returns false if Parking Spot was replaced
				 */
				bool insertParkingSpot(ParkingSpot parkingSpot)
				{
					bool returnValue = true;
				
					/* Parking Spot already exists */
					if(hasParkingSpot(parkingSpot))
					{
						/* @TODO What to do when Parking Spot already exists? As for now: replace the old one */
						eraseParkingSpot(parkingSpot);
						returnValue = false;
					}
					m_coordinate2heading.insert(std::make_pair(parkingSpot.getCoordinate().getBoostPoint(),parkingSpot.getHeading()));
					return returnValue;
				
				}

				/**
				 * erase
				 * DO NOT USE WHILE ITERATING WITH ITERATORS OVER RTree!
				 * erases specifed parkingSpot from ParkingSpotSet
				 * returns false if ParkingSpot does not exist
				 */
				bool eraseParkingSpot(ParkingSpot parkingSpot)
				{
					return eraseParkingSpot(parkingSpot.getCoordinate().getBoostPoint());
				}

				bool eraseParkingSpot(adore::env::BorderBased::Coordinate coordinate)
				{
					return eraseParkingSpot(coordinate.getBoostPoint());
				}
			
			
				/**
				 * clear
				 * clears ParkingSpotSet
				 */
				void clear()
				{
					m_coordinate2heading.clear();
				}

				/**
				 * getAllParkingSpots
				 * get begin() and end() iterators for all TCDs in ParkingSpotSet
				 */
				ItPairCoordinate2heading getAllParkingSpots()
				{
					auto first = m_coordinate2heading.qbegin(boost::geometry::index::intersects(m_coordinate2heading.bounds()));
					auto last = m_coordinate2heading.qend();
					return ItPairCoordinate2heading(first, last);
				}

				/**
				 * getParkingSpotsInRegion
				 * get begin() and end() iterators for all ParkingSpots in defined region
				 */
				ItPairCoordinate2heading getParkingSpotsInRegion(double x0, double x1, double y0, double y1)
				{
					auto it = m_coordinate2heading.qbegin(boost::geometry::index::intersects(
						boost::geometry::model::box<env::BorderBased::Coordinate::boost_point>(
							env::BorderBased::Coordinate::boost_point(x0,y0,-m_guard),
							env::BorderBased::Coordinate::boost_point(x1,y1,+m_guard)
						)));
					return ItPairCoordinate2heading(it,m_coordinate2heading.qend());
				}

				/**
				 * getTCDsOutsideRegion
				 * get begin() and end() iterators for all TCDs outsite of defined region
				 */
				ItPairCoordinate2heading getParkingSpotsOutsideRegion(double x0,double x1, double y0, double y1)
				{
					auto it = m_coordinate2heading.qbegin(boost::geometry::index::disjoint(
						boost::geometry::model::box<env::BorderBased::Coordinate::boost_point>(
							env::BorderBased::Coordinate::boost_point(x0,y0,-m_guard),
							env::BorderBased::Coordinate::boost_point(x1,y1,+m_guard)
						)));
					return ItPairCoordinate2heading(it,m_coordinate2heading.qend());
				}
				/**
				 * removing TCDs in given region without
				 */
				void removeParkingSpotsOutsideRegion(double x0,double x1, double y0, double y1)
				{
					std::vector<adore::env::BorderBased::Coordinate::boost_point> parkingSpotsToRemove;
					for(auto itpair = getParkingSpotsOutsideRegion(x0,x1,y0,y1); itpair.first!=itpair.second; itpair.first++)
					{
						parkingSpotsToRemove.push_back(itpair.first->first);
					}
					for(auto it = parkingSpotsToRemove.begin(); it!=parkingSpotsToRemove.end();it++)
					{
						eraseParkingSpot(*it);
					}
				}

			
				/**
				 * hasParkingSpot
				 * returns true if ParkingSpot is in ParkingSpotSet
				 */
				bool hasParkingSpot(ParkingSpot parkingSpot)
				{
					return hasParkingSpot(parkingSpot.getCoordinate().getBoostPoint());
				}

				/**
				 * itPair2ParkingSpotVector
				 * re-organizes itPairCoordinate2heading to a ParkingSpot object vector
				 */
				std::vector<ParkingSpot> itPair2ParkingSpotVector(ItPairCoordinate2heading itPair)
				{
					std::vector<ParkingSpot> returnValue;
					for(auto it = itPair.current(); it!=itPair.end(); it++)
					{
						returnValue.push_back(ParkingSpot(BorderBased::Coordinate(it->first.get<0>(),it->first.get<1>(),it->first.get<2>()),it->second));
					}
					return returnValue;
				}
			};
		}
	}
}