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

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * @brief This class definition is to represent parking spots
			 * 
			 */
			class ParkingSpot
			{
				BorderBased::Coordinate m_coordinate; /**< coordinates of the parking spot */
				double m_heading; /**< heading of the parking spot */
		
			public:
			/**
			 * @brief Construct a new ParkingSpot object
			 * 
			 */
				ParkingSpot()
				{
					m_coordinate = BorderBased::Coordinate();
					m_heading = 0;
				}
				/**
				 * @brief Construct a new ParkingSpot object
				 * 
				 * @param coordinate coordinates of the new parking spot
				 * @param heading heading of the new parking spot
				 */
				ParkingSpot(BorderBased::Coordinate coordinate, double heading)
				{
					m_coordinate = coordinate;
					m_heading = heading;
				}
				/**
				 * @brief Get the heading of the parking spot
				 * 
				 * @return double heading
				 */
				double getHeading() const
				{
					return m_heading;
				}
				/**
				 * @brief Get the Coordinate object
				 * 
				 * @return BorderBased::Coordinate coordinates of the parking spot
				 */
				BorderBased::Coordinate getCoordinate() const
				{
					return m_coordinate;
				}
				/**
				 * @brief Set the heading of the parking spot
				 * 
				 * @param heading new heading of the parking spot
				 */
				void setHeading(double heading)
				{
					m_heading = heading;
				}
				/**
				 * @brief Set the Coordinate
				 * 
				 * @param c new coordinates of the parking spot
				 */
				void setCoordinate(adore::env::BorderBased::Coordinate c)
				{
					m_coordinate = c;
				}
			};
		}
	}
}