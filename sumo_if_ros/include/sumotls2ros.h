/****************************************************************************/
// Eclipse ADORe, Automated Driving Open Research; see https://eclipse.org/adore
// Copyright (C) 2017-2020 German Aerospace Center (DLR).
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    sumotls2ros.h
/// @author  Stephan Lapoehn
/// @contact Stephan.Lapoehn@DLR.de
/// @date 2020/07/02


#pragma once


#include <adore/env/tcd/MAPEMIntersection.h>
#include <adore/env/borderbased/coordinate.h>
#include <adore/mad/coordinateconversion.h>
#include <unordered_map>
#include <utils/traci/TraCIAPI.h>
#include <v2xsim/SimMAPEM.h>
#include <v2xsim/SimSPATEM.h>
#include <math.h>
namespace adore
{
  namespace sumo_if_ros
  {
      /**
       * generates MAPEM and SPATEM messages from sumo simulation
       **/
      class SumoTLs2Ros
      {
        public:

           struct hash_pair { 
                template <class T1, class T2> 
                size_t operator()(const std::pair<T1, T2>& p) const
                { 
                    auto lhs = std::hash<T1>{}(p.first); 
                    auto rhs = std::hash<T2>{}(p.second); 

                    lhs ^= rhs + 0x9e3779b9 + (lhs << 6) + (lhs >> 2);
                    return lhs;
                } 
            }; 

        private:

            // maps the sumo-lane-id strings to the v2x lane ids for SPATEM generation
            std::unordered_map<std::string, uint8_t> sumo_lane_id_mapping_;
            
            // maps the sumo-intersection-id strings to the v2x intersection ids for MAPEM / SPATEM generation
            std::unordered_map<std::string, int> sumo_intersection_id_mapping_; 

            // MAPEM data will only be generated once, due to its static nature
            std::unordered_map<int, adore::env::MAPEMIntersection> intersections_;

            // MAPEM data will only be generated once, due to its static nature
            std::unordered_map<int, v2xsim::SimMAPEM> _MAPEM;

            // one signal group for every ingress - egress lane pair 
            std::unordered_map<std::pair<std::string,std::string>, uint8_t, SumoTLs2Ros::hash_pair> sumo_signal_group_id_mapping_;

            // generates n nodes for one sumo lane as vector of adore coordinates
            std::vector<adore::env::BorderBased::Coordinate> getNodesFromSUMOLane(libsumo::TraCIPositionVector & lane, int nodeCount = 5);
            
            // gathers information of the lane links on a intersection and generates coordinate vectors for these links
            void getGeoreferencedLinks(TraCIAPI & client, std::string sumoIntersectionID, std::unordered_map<std::string, std::vector<std::string>>& linklist, std::unordered_map<std::string, std::vector<adore::env::BorderBased::Coordinate>> & id_2_shape);

            void autofill_bit_string(std::size_t size, std::vector<uint8_t> & items);

            // entry point for MAPEM generation
            std::unordered_map<int, adore::env::MAPEMIntersection> generateMAPEMFromSUMO(TraCIAPI & client);

            // convert adore internal MAPEM representation to ROS msg
            std::unordered_map<int, v2xsim::SimMAPEM> convertToROSMsg(std::unordered_map<int, adore::env::MAPEMIntersection> mapem_data, double time = 0,double power = 23);

            // calculates the distance btw. 2 wgs84 points in m
            double wgs84_distance(double lat1, double lon1, double lat2,double lon2);

            double getSystemTime();

            int32_t getMOY(double time);

            int getLeapYearSeconds(double time);

            double getSecondOfYearFromUTC(double time);

        public:

            // get MAPEM information based on sumo traffic-light interface TraCI 
            std::unordered_map<int, v2xsim::SimMAPEM> getMAPEMFromSUMO(TraCIAPI & client,double time,double power=23);

            // get SPATEM information based on sumo traffic-light interface TraCI 
            std::vector<v2xsim::SimSPATEM> getSPATEMFromSUMO(TraCIAPI & client, double time, double power=23);

            int getIntersectionIDForSUMOString(std::string sumo_intersection_id);

            std::string getSUMOStringFromIntersectionID(int intersection_id);

            SumoTLs2Ros():utm_zone_{ 32 },is_south_hemi_{false},_use_system_time{false},_generate_spat_timing{true}{};

            int utm_zone_;

            bool is_south_hemi_;

            bool _use_system_time;

            bool _generate_spat_timing;
      };
  }
}