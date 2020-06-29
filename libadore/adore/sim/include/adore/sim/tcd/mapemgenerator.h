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
#pragma once


#include <string>
#include <math.h>
#include <unordered_map>
#include <vector>

#include <adore/env/tcd/trafficlight.h>
#include <adore/env/borderbased/stopline.h>

#include <adore/env/borderbased/borderset.h>
#include <adore/env/borderbased/border.h>
#include <adore/env/borderbased/alanepositionedobject.h>

#include <adore/env/tcd/MAPEMIntersection.h>
#include <adore/mad/coordinateconversion.h>
#include <adore/mad/arraymatrixtools.h>

namespace adore
{
	namespace sim
	{
		/**
		 *  MAPEMGenerator is a static class that generates from borders, stoplines and traffic lights MAPEM messages for usage in simulations
		 */ 
		class MAPEMGenerator
		{

		public:

			typedef std::unordered_map<int, std::unordered_map<adore::env::BorderBased::Border*, adore::env::TrafficLight*>> IntersectionBordesr2Tl;
			typedef std::unordered_map<int, std::multimap<int,adore::env::TrafficLight*>> Intersections2Tl;

			/**
			 * 	Creates MAPEMIntersections from borders, stoplines and traffic lights for usage in simulations
			 * 
			 * @param observer pointer to a BorderSet containing all relevant borders for this intersection
			 * @param trafficLights reference to a vector of trafficLights of the intersections
			 * @param stoplines reference to a vector of stoplines of the intersections
			 * @param result reference to the result vector that will hold the MAPEMIntersection data afer func. executation 
			 */
			static void CreateMAPEMs(adore::env::BorderBased::BorderSet * observer, std::vector<adore::env::TrafficLight*> & trafficLights, 
				std::vector<adore::env::BorderBased::StopLine*> & stoplines, std::vector<adore::env::MAPEMIntersection> & result);

		private:

			static std::vector<adore::env::MAPEMIntersection> GenerateTopoInfoFromTLS(adore::env::BorderBased::BorderSet * observer, 
				Intersections2Tl & tcdOfIntersection, std::vector<adore::env::BorderBased::StopLine*> & stoplines);

			static adore::env::MAPEMLane::ConnectingLane::AllowedManeuvers DiscoverPossibleManeuvers(adore::env::BorderBased::Border* border, adore::env::BorderBased::BorderSet * observer, int & numberOfManeuvers);

			static adore::env::TrafficLight*  MapTL2Border(adore::env::BorderBased::Border * border, std::multimap<int,adore::env::TrafficLight*> & TlOfIntersection, float wOrientation = 1.0f, float wDistance = 1.0f);

			static std::unordered_map<int, adore::env::BorderBased::Coordinate> GenerateCentroidCoordForIntersections (Intersections2Tl & tcdOfIntersection);

			static std::vector<adore::env::BorderBased::Coordinate> GetWGS84CenterLineOfBorder(adore::env::BorderBased::Border * border,adore::env::BorderBased::BorderSet * observer,double offset = 0);

			static std::vector<adore::env::MAPEMIntersection> GenerateLaneDescriptions(std::unordered_map<int, adore::env::BorderBased::Coordinate> & refCoordinates,	
				std::unordered_map<int ,std::vector<adore::env::BorderBased::Coordinate>> & borderCoordinates,IntersectionBordesr2Tl & trafficLightWithBorder, 
				adore::env::BorderBased::BorderSet * observer);

		};
	}
}