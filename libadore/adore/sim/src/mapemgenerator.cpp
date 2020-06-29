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
 *   Stephan Lapoehn - initial API and implementation
 ********************************************************************************/
#include <adore/sim/tcd/mapemgenerator.h>

using namespace adore::sim;
using namespace adore::env;

std::unordered_map<int, BorderBased::Coordinate> MAPEMGenerator::GenerateCentroidCoordForIntersections(MAPEMGenerator::Intersections2Tl & tcdOfIntersection)
{
	std::unordered_map<int, BorderBased::Coordinate> referenceCoordinate;
	//for every intersection
	for(auto iter = tcdOfIntersection.begin(); iter != tcdOfIntersection.end(); ++iter)
	{
		BorderBased::Coordinate centroidSumCoord;

		//for every TL generate centroid coordinate
		for(std::multimap<int,TrafficLight*>::iterator iterTL = iter->second.begin(); iterTL != iter->second.end();++iterTL)
		{
			centroidSumCoord.m_X += iterTL->second->getCoordinate().m_X;
			centroidSumCoord.m_Y += iterTL->second->getCoordinate().m_Y;
			centroidSumCoord.m_Z += iterTL->second->getCoordinate().m_Z;
		}
		centroidSumCoord.m_X = (double) centroidSumCoord.m_X/iter->second.size();
		centroidSumCoord.m_Y = (double) centroidSumCoord.m_Y/iter->second.size();
		centroidSumCoord.m_Z = (double) centroidSumCoord.m_Z/iter->second.size();

		// MAPEM works with WGS84 lat/lon so convert coordinates here
		adore::mad::CoordinateConversion::UTMXYToLatLonDegree(centroidSumCoord.m_X
			,centroidSumCoord.m_Y,32,false,centroidSumCoord.m_X,centroidSumCoord.m_Y);

		referenceCoordinate.insert( std::make_pair(iter->first,centroidSumCoord));
	}

	return referenceCoordinate;
}

std::vector<BorderBased::Coordinate> MAPEMGenerator::GetWGS84CenterLineOfBorder(BorderBased::Border * border,
	BorderBased::BorderSet * observer, double offset)
{
		auto centerline = observer->getCenterline(border->m_id);
		std::vector<BorderBased::Coordinate> laneCoordinateVector;

		// extract coordinates from centerline
		int n = centerline.getData().nc();
		double X[150] = {0};
		double Y[150] = {0};

		auto posOfStoplineonCenterLine = centerline.f(offset);

		//X[0] = posOfStoplineonCenterLine(0,0);
		//Y[0] = posOfStoplineonCenterLine(1,0);

		n = centerline.findIndex(offset,0.0000001);

		centerline.writePointsToArray(0,n,0,X);
		centerline.writePointsToArray(0,n,1,Y);

		//store values referring to the reference point
		// we only want 5 points of each lane
		
		int steps = 1;
		
		if(n > 5)
			steps = std::floor((float) n/5.f);
		
		int validValueCounter = 0;

		while(validValueCounter < n+1 && X[validValueCounter] != 0)
		{
			BorderBased::Coordinate nodeCoordinate;

			double lat = 0;
			double lon = 0;

			adore::mad::CoordinateConversion::UTMXYToLatLonDegree(X[n-validValueCounter],Y[n-validValueCounter],32,false,lat,lon);

			nodeCoordinate.m_X = lat;
			nodeCoordinate.m_Y = lon;

			laneCoordinateVector.push_back(nodeCoordinate);
			validValueCounter+=steps;
		}

		return laneCoordinateVector;
}

TrafficLight*  MAPEMGenerator::MapTL2Border(BorderBased::Border * border, 
	std::multimap<int,TrafficLight*> & TlOfIntersection,float wOrientation, float wDistance)
{
	
	dlib::vector<double,2> borderDirection;
	dlib::vector<double,2> tlDirection;

	if (border == NULL)
		return NULL;

	// take the orientation from the center of the border
	double s = border->m_path->limitHi() != 0 ? border->m_path->limitHi()  / 2 : 0;
	adoreMatrix<double,3,1> p0 = border->m_path->f(s);

	// differentiate
	borderDirection.x() = border->m_path->dfidx(s,0);
	borderDirection.y() = border->m_path->dfidx(s,1);

	TrafficLight * mostLikelyRelatedTL = nullptr;
	double mostLikelyRating = -1;

	// find related TCD
	for(auto iterTCD = TlOfIntersection.begin(); iterTCD != TlOfIntersection.end(); ++iterTCD)
	{
		auto tcdCoord = iterTCD->second->getCoordinate();

		double distance = tcdCoord.distance(p0);

		// tcd is > 30 m from tcd to stopline, matching not possible 
		if(distance > 30)
			continue;

		// orientation of TCD
		double tl_gx = std::cos(iterTCD->second->getOrientation());		
		double tl_gy = std::sin(iterTCD->second->getOrientation());

		tlDirection.x() = tl_gx;
		tlDirection.y() = tl_gy;

		double angle = adore::mad::ArrayMatrixTools::aglBetwVecInArcLen<2>(borderDirection,tlDirection);

		distance = std::abs(distance) / 10;
		double rating = wOrientation * (1/angle) + wDistance * (1/distance);

		// guaranties that for every stopline a tcd will be taken into account (but possibly the wrong one, if the rating is low)
		if(rating > mostLikelyRating)
		{
			mostLikelyRating = rating;
			mostLikelyRelatedTL = iterTCD->second;
		}

	}
	if(mostLikelyRating > 5)
		return mostLikelyRelatedTL;
	else 
		return nullptr;
}

std::vector<MAPEMIntersection> MAPEMGenerator::GenerateLaneDescriptions(std::unordered_map<int, BorderBased::Coordinate> & refCoordinates,
	std::unordered_map<int ,std::vector<BorderBased::Coordinate>> & borderCoordinates,IntersectionBordesr2Tl & trafficLightWithBorder,
	BorderBased::BorderSet * observer)
{
	std::vector<MAPEMIntersection> topoInformation;

	// for every intersection
	for(auto loiIter = trafficLightWithBorder.begin(); loiIter != trafficLightWithBorder.end(); ++loiIter)
	{	
		MAPEMIntersection mapem_intersection;
		
		//set intersection
		mapem_intersection.intersection_id_ = (*loiIter).first;

		// for every border (border is unique key)
		for(auto iter = loiIter->second.begin(); iter != loiIter->second.end(); ++iter)
		{
			double lat;
			double lon;

			auto refcoord = refCoordinates.find((*loiIter).first)->second;

			// SPATEM works with wgs48 lat/lon so convert coordinates here
			adore::mad::CoordinateConversion::UTMXYToLatLonDegree(refcoord.m_X
				,refcoord.m_Y,32,false,lat,lon);

			mapem_intersection.ref_latitude_ = lat;
			mapem_intersection.ref_longitude_ = lon;

			if(observer->hasLeftNeighbor((*iter).first))
				mapem_intersection.lane_width_ = (*iter).first->m_id.m_first.distance((*iter).first->m_left->m_first);
			else
				mapem_intersection.lane_width_ = 1;

		
			// set borders of TCD
			MAPEMLane mapem_lane;
		
			mapem_lane.lane_id_ = BorderBased::BorderIDHasher()((*iter).first->m_id);
			mapem_lane.v_nodes_ = borderCoordinates.find(mapem_lane.lane_id_)->second;
			mapem_lane.nodes_count_ = mapem_lane.v_nodes_.size();
		
			//find successors of border
			BorderBased::itCoordinate2Border successor = observer->getSuccessors((*iter).first);
			for(auto its = successor;its.current()!=its.end();its.current()++)
			{
				MAPEMLane::ConnectingLane cL;
				cL.lane_id = BorderBased::BorderIDHasher()((*its.current()).second->m_id);
				cL.signal_group_id =  (*iter).second->movement_id_;
				cL.allowed_maneuvers_on_connection = MAPEMGenerator::DiscoverPossibleManeuvers((*iter).first,observer,cL.maneuver_count);
				mapem_lane.v_connector_lanes_id_.push_back(cL);
			}
			mapem_intersection.lanes_.push_back(mapem_lane); 
		}
		
		mapem_intersection.lanes_count_ = mapem_intersection.lanes_.size();
		topoInformation.push_back(mapem_intersection);
	}

	return topoInformation;
}

 MAPEMLane::ConnectingLane::AllowedManeuvers MAPEMGenerator::DiscoverPossibleManeuvers(BorderBased::Border* border, 
	 BorderBased::BorderSet * observer, int & numberOfManeuvers)
{
	numberOfManeuvers = 0;

	MAPEMLane::ConnectingLane::AllowedManeuvers aM;
	
	BorderBased::itCoordinate2Border successor = observer->getSuccessors(border);

	numberOfManeuvers = std::distance(successor.current(),successor.end());

	if(numberOfManeuvers == 3)
	{
		aM.set(0);
		aM.set(1);
		aM.set(2);
	}
	else if(numberOfManeuvers == 1)
	{
		aM.set(0);
	}

	return aM;
}

std::vector<MAPEMIntersection> MAPEMGenerator::GenerateTopoInfoFromTLS(BorderBased::BorderSet * observer, 
	MAPEMGenerator::Intersections2Tl & tcdOfIntersection, std::vector<BorderBased::StopLine*> & stoplines)
{
	std::vector<MAPEMIntersection> topoInformation;

	IntersectionBordesr2Tl trafficLightWithBorder;

	std::unordered_map<int, BorderBased::Coordinate> referenceCoordinate;

	std::unordered_map<int ,std::vector<BorderBased::Coordinate>> borderCoordinates;

	/*
	* -------------------------------------------------------------
	*	generate geo - reference of intersections by using TCDs
	* _____________________________________________________________
	*/
	if(tcdOfIntersection.begin() == tcdOfIntersection.end())
		return std::vector<MAPEMIntersection>();

	referenceCoordinate = MAPEMGenerator::GenerateCentroidCoordForIntersections(tcdOfIntersection);

	/*
	* -------------------------------------------------------------
	*	find pairs of TCDs and borders StopLines 
	* _____________________________________________________________
	*/

	// for every intersection
	for(std::unordered_map<int, std::multimap<int,TrafficLight*> >::iterator iterIntersec = tcdOfIntersection.begin(); iterIntersec != tcdOfIntersection.end(); ++iterIntersec)
	{
		BorderBased::Coordinate referenceCoord = referenceCoordinate[(*iterIntersec).first];

		// for every stopline on intersection
		for(int i = 0; i < stoplines.size();i++)
		{
			auto border = observer->getBorder(stoplines[i]->getLanePosition().m_rightID);
			
			if(border && border->m_type != BorderBased::BorderType::DRIVING)
				continue;

			// match borders to traffic lights if possible
			TrafficLight * mostLikelyRelatedTL = MAPEMGenerator::MapTL2Border(border,(*iterIntersec).second,0.5f,0.5f);

			if(mostLikelyRelatedTL)
			{
				trafficLightWithBorder[(*iterIntersec).first][border] = mostLikelyRelatedTL;
				borderCoordinates[BorderBased::BorderIDHasher()(border->m_id)] = MAPEMGenerator::GetWGS84CenterLineOfBorder(border,observer,stoplines[i]->getLanePosition().m_progress);
			}
		}

	}

	if(trafficLightWithBorder.empty())
		return topoInformation;

	/*
	* -------------------------------------------------------------
	*	Create Topo for every intersection in viewing range
	* -------------------------------------------------------------
	*/
	
	return MAPEMGenerator::GenerateLaneDescriptions(referenceCoordinate,borderCoordinates,trafficLightWithBorder, observer);

}

void MAPEMGenerator::CreateMAPEMs(BorderBased::BorderSet * observer, std::vector<TrafficLight*> & trafficLights, 
	std::vector<BorderBased::StopLine*> & stoplines,std::vector<MAPEMIntersection> & result)
{
	MAPEMGenerator::Intersections2Tl tcdOfIntersection;

	// sort traffic lights
	for(auto it = trafficLights.begin(); it != trafficLights.end();it++)
	{
		TrafficLight * tl = (*it);
		auto item = std::make_pair(tl->movement_id_,tl);
		tcdOfIntersection[tl->intersection_id_].insert(item);

	}

	result = MAPEMGenerator::GenerateTopoInfoFromTLS(observer,tcdOfIntersection,stoplines);

}