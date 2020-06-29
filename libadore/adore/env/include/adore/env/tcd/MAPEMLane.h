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

#include <bitset>
#include <vector>
#include <adore/env/borderbased/coordinate.h>

namespace adore
{
	namespace env
	{
		class MAPEMLane 
		{
		public:

			/**
			 * ConnectingLane defines whether the igressing MAPEMLane is connected 
			 * to an egressing MAPEMLane, the related controller id and allowed maneuvers on this connection
			 */ 
			struct ConnectingLane
			{
				/**
				* Use these enum values as pos parameter in allowed_maneuvers
				*/
				enum Maneuvers
				{
					ManeuverRight,
					ManeuverStraight,
					ManeuverLeft,

					// Needs to be last
					NumManeuvers
				};

				typedef std::bitset<NumManeuvers> AllowedManeuvers;
				
				AllowedManeuvers allowed_maneuvers_on_connection;

				int maneuver_count;
				
				unsigned int lane_id;
			
				unsigned int signal_group_id;

				bool operator==(const ConnectingLane & other) const
				{
					return (other.lane_id == lane_id)
						&& (other.signal_group_id == signal_group_id)
						&& (other.allowed_maneuvers_on_connection == allowed_maneuvers_on_connection);
				}
			};

			enum LaneTypesAttributes
			{
				VEHICLE,	// motor vehicle lanes
				CROSSWALK,  // pedestrian crosswalks
				BIKELANE,	// bike lanes
				SIDEWALK,	// pedestrian sidewalk paths
				MEDIAN,		// medians and channelization
				STRIPING,	// roadway markings
				TRACKEDVEHICLE,	// trains and trolleys
				PARKING,	// parking and stopping lanes
			};

			/**
			 * is lane ingressing or egressing lane of intersection
			 */
			enum DirectionalUse
			{
				INGRESS,
				EGRESS,
				BOTH
			};


			LaneTypesAttributes lanetype_attribute_;

			unsigned int lane_id_;
			
			int nodes_count_;
			
			std::vector<adore::env::BorderBased::Coordinate> v_nodes_;
			
			int connected_lanes_count_;
			
			std::vector<ConnectingLane> v_connector_lanes_id_;
			
			DirectionalUse directional_use_;

			bool operator==(const MAPEMLane & other) const
			{
				return (other.lane_id_ == lane_id_)
					&& (other.v_connector_lanes_id_ == v_connector_lanes_id_)
					&& (other.v_nodes_ == v_nodes_);
			}

			/**
			* Make a quick but incomplete comparison
			*
			* Compares only first lane and node of this and other.
			*/
			virtual bool isEqualShallow(const MAPEMLane & other) const
			{
				if (other.lane_id_ != lane_id_)
					return false;

				// Compare lanes
				const std::size_t lanesSize = v_connector_lanes_id_.size();
				const std::size_t otherLanesSize = other.v_connector_lanes_id_.size();
				if (lanesSize != otherLanesSize)
					return false;
				// First lanes unequal?
				if (lanesSize && !(*v_connector_lanes_id_.cbegin() == *other.v_connector_lanes_id_.cbegin()))
					return false;

				// Compare nodes
				const std::size_t nodesSize = v_nodes_.size();
				const std::size_t otherNodesSize = other.v_nodes_.size();
				if (nodesSize != otherNodesSize)
					return false;
				// First nodes unequal?
				if (nodesSize && !(*v_nodes_.cbegin() == *other.v_nodes_.cbegin()))
					return false;

				return true;
			}
		};

	} 
}  