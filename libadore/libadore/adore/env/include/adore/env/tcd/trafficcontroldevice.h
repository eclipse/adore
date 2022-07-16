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

			enum TCDClass
			{
				C_TRAFFIC_LIGHT = 0,
				C_PRECEDENCE = 1,
				C_SPEED_LIMIT = 2,
				C_UNKNOWN = 99
			};

			/**
			 *  @brief generalization of tcd types
			 * 	@return enum type value for tcd class: allows to treat similar types of tcds similarly.
			 */
			TCDClass getTCDClass()
			{
				switch(type_)
				{
					case TRAFFIC_LIGHT:
					case TRAFFIC_LIGHT_PEDESTRIAN:
						return C_TRAFFIC_LIGHT;
					case RIGHT_OF_WAY:
					case STOP_SIGN:
						return C_PRECEDENCE;
					case SPEED_LIMIT_10:
				    case SPEED_LIMIT_20:
				    case SPEED_LIMIT_30:
				    case SPEED_LIMIT_40:
				    case SPEED_LIMIT_50:
				    case SPEED_LIMIT_60:
				    case SPEED_LIMIT_70:
				    case SPEED_LIMIT_80:
				    case SPEED_LIMIT_90:
				    case SPEED_LIMIT_100:
				    case SPEED_LIMIT_120:
				    case SPEED_LIMIT_130:
						return C_SPEED_LIMIT;
					default:
						return C_UNKNOWN;
				}
			}

			/**
			 * @brief Some tcd's have one or more numeric value affixed to them, e.g. speed limits have one.
			 * @return first numeric value
			 */
			double getNumericValue1()
			{
				switch(type_)
				{
					case SPEED_LIMIT_10:
						return 10.0/3.6;
				    case SPEED_LIMIT_20:
						return 20.0/3.6;
				    case SPEED_LIMIT_30:
						return 30.0/3.6;
				    case SPEED_LIMIT_40:
						return 40.0/3.6;
				    case SPEED_LIMIT_50:
						return 50.0/3.6;
				    case SPEED_LIMIT_60:
						return 60.0/3.6;
				    case SPEED_LIMIT_70:
						return 70.0/3.6;
				    case SPEED_LIMIT_80:
						return 80.0/3.6;
				    case SPEED_LIMIT_90:
						return 90.0/3.6;
				    case SPEED_LIMIT_100:
						return 100.0/3.6;
				    case SPEED_LIMIT_120:
						return 120.0/3.6;
				    case SPEED_LIMIT_130:
						return 130.0/3.6;
					default:
						return 0.0;
				}
			}

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
			void rotate(double angle, double x0=0, double y0=0)
			{
				this->coordinate_.rotate(angle, x0, y0);
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