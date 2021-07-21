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
#include <adore/env/tcd/MAPEMLane.h>

namespace adore
{
	namespace env
	{
		/**
		 * Intersection represented by a simplified MAPEM message
		 */
		class MAPEMIntersection
		{
			public:

				unsigned int intersection_id_;

				int lanes_count_;

				float lane_width_;				

				/**
				 * latitude of the reference coordinate of the intersection 
				 * often centroid of the traffic light positions on an intersection
				 */ 
				double ref_latitude_;				

				/**
				 * longitude of the reference coordinate of the intersection 
				 * often centroid of the traffic light positions on an intersection
				 */ 
				double ref_longitude_;

				/**
				 * lanes of the intersection and their relation to each other
				 */
				std::vector<MAPEMLane> lanes_;

			/**
			 * Compare equal
			 *
			 * We do not compare the float elements, because that
			 * requires to consider accuracy.
			 */
			bool operator==(const MAPEMIntersection &other) const
			{
				return (other.intersection_id_ == intersection_id_)
					&& (other.lanes_ == lanes_);
			}

			/**
			 * Make a quick but incomplete comparison
			 * Calls shallow comparison for the lanes.
			 */
			virtual bool isEqualShallow(const MAPEMIntersection & other) const
			{
				if((other.intersection_id_ != intersection_id_)
				   || (other.lanes_.size() != lanes_.size()))
					return false;

				auto otherLaneIt = other.lanes_.cbegin();

				auto laneIt = lanes_.cbegin();

				for(; laneIt != lanes_.cend() && otherLaneIt != other.lanes_.cend(); ++otherLaneIt, ++laneIt)
				{
					if(!laneIt->isEqualShallow(*otherLaneIt))
						return false;
				}

				return true;
			}
	
		};
	}
}