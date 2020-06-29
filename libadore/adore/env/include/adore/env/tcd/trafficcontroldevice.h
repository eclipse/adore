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

#include <adore/env/borderbased/coordinate.h>

namespace adore
{
	namespace env
	{
		class TrafficControlDevice
		{
		public:

			enum TCDType
			{
				TRAFFIC_LIGHT = 0,
				TRAFFIC_LIGHT_PEDESTRIAN = 1,
				RIGHT_OF_WAY = 2, 
				STOP_SIGN = 3,
				SPEED_LIMIT_10 = 4,
				SPEED_LIMIT_20 = 5,
				SPEED_LIMIT_30 = 6,
				SPEED_LIMIT_40 = 7,
				SPEED_LIMIT_50 = 8,
				SPEED_LIMIT_60 = 9,
				SPEED_LIMIT_70 = 10,
				SPEED_LIMIT_80 = 11,
				SPEED_LIMIT_90 = 12,
				SPEED_LIMIT_100 = 13,
				SPEED_LIMIT_120 = 14,
				SPEED_LIMIT_130 = 15,
				SPEED_LIMIT_NON = 16,
				CITY_LIMIT_SIGN = 17,
				MOTOR_WAY_SIGN = 18,
				UNKNOWN = 99
			};

			TrafficControlDevice() {}
			TrafficControlDevice(const TrafficControlDevice& other) 
			{
				this->coordinate_ = other.coordinate_;
				this->id_ = other.id_;
				this->orientation_ = other.orientation_;
				this->type_ = other.type_;
			}

			void translate(double dx,double dy,double dz)
			{
				this->coordinate_.translate(dx,dy,dz);
			}

			int getID() const { return id_; }

			void setID(int id) { id_ = id; }

			void setType(TCDType type)
			{
				type_ = type;
			}

			TCDType getType() const
			{
				return type_;
			}

			void setOrientation(double orientation)
			{
				orientation_ = orientation;
			}

			double getOrientation() const
			{
				return orientation_;
			}

			void setCoordinate(BorderBased::Coordinate coordinate)
			{
				coordinate_ = coordinate;
			}

			BorderBased::Coordinate getCoordinate() const
			{
				return coordinate_;
			}


		private:

			TCDType type_;

			int id_;

			double orientation_;

			BorderBased::Coordinate coordinate_;
		};
	}
}