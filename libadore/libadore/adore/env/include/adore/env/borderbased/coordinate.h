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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#pragma once
#include <limits>
#include <adore/mad/adoremath.h>
#include <string>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <adore/mad/csvlog.h>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * @brief This struct represents 3-dimensional coordines
			 * 
			 */
			struct Coordinate
			{
				double m_X, m_Y, m_Z;
				typedef boost::geometry::model::point<double,3,boost::geometry::cs::cartesian> boost_point;
				/**
				 * @brief Construct a new Coordinate object. All coordinates are set to zero.
				 * 
				 */
				Coordinate()
				{
					m_X = 0;m_Y = 0;m_Z = 0;
				}
				/**
				 * @brief Construct a new Coordinate object. The coordinates are taken from a boost_point
				 * 
				 * @param bp boost_point whose coordinates are taken for the newly cunstructed object
				 */
				Coordinate(boost_point bp)
				{
					m_X = bp.get<0>();
					m_Y = bp.get<1>();
					m_Z = bp.get<2>();
				}
				/**
				 * @brief Construct a new Coordinate object. The coordinates are taken from a adoreMatrix.
				 * 
				 * @param c adoreMatrix with the coordinates for the newly constructed object
				 */
				Coordinate(adoreMatrix<double,3,1> c)
				{
					m_X = c(0);
					m_Y = c(1);
					m_Z = c(2);
				}
				/**
				 * @brief Construct a new Coordinate object.
				 * 
				 * @param x x-coordinate
				 * @param y y-coordinate
				 * @param z z-coordinate
				 */
				Coordinate(double x,double y,double z)
				{
					m_X=x;
					m_Y=y;
					m_Z=z;
				}
				/**
				 * @brief Translate a coordinate object
				 * 
				 * @param dx move x-coordinate by this value
				 * @param dy move y-coordinate by this value
				 * @param dz move z-coordinate by this value
				 */
				void translate(double dx,double dy,double dz)
				{
					m_X += dx;
					m_Y += dy;
					m_Z += dz;
				}
				/**
				 * @brief rotate around x0,y0
				 * 
				 * @param angle 
				 * @param x0 
				 * @param y0 
				 */
				void rotate(double angle, double x0=0.0, double y0=0.0)
				{
					double x = m_X;
					double y = m_Y;
					m_X =  (x-x0)*std::cos(angle) - (y-y0)*std::sin(angle) + x0;
					m_Y =  (x-x0)*std::sin(angle) + (y-y0)*std::cos(angle) + y0;
				}
				/**
				 * @brief Discretize the coordinate object
				 * 
				 * Everything with distance smaller than precision will be mapped to the same id.
				 * @param lx function writes discretized x-coordinate to this parameter
				 * @param ly function writes discretized y-coordinate to this parameter
				 * @param iz function writes discretized z-coordinate to this parameter
				 */
				void discretize(long& lx, long& ly, int& iz)const
				{
					static const double precisionX = 1.0 / 0.2;// 0.2m precision
					static const double precisionY = 1.0 / 0.2;// 0.2m precision
					static const double precisionZ = 1.0;// 1m precision
					//static const double precisionX = 1.0;
					//static const double precisionY = 1.0;
					//static const double precisionZ = 1.0;
					lx = (long)(m_X*precisionX);
					ly = (long)(m_Y*precisionY);
					iz = (int)(m_Z*precisionZ);
				}
				/**
				 * @brief Calculate the distance between two Coordinates
				 * 
				 * @param other Coordinate to which the distance should be calculated 
				 * @return double distance between the two Coordinates
				 */
				double distance(const Coordinate& other) const
				{
					double dx = this->m_X-other.m_X;
					double dy = this->m_Y-other.m_Y;
					double dz = this->m_Z-other.m_Z;
					return std::sqrt(dx*dx+dy*dy+dz*dz);
				}
				/**
				 * @brief Check whether the Coordinate is near to another Coordinate
				 * 
				 * @param other second Coordinate that is used for the check
				 * @return true if the Coordinates are near to each other in all dimensions
				 * @return false in all other cases
				 */
				bool isNear(const Coordinate& other) const
				{
					static const double precisionX = 1.0;
					static const double precisionY = 1.0;
					static const double precisionZ = 3.0;
					return std::abs(this->m_X-other.m_X)<=precisionX
						&& std::abs(this->m_Y-other.m_Y)<=precisionY
						&& std::abs(this->m_Z-other.m_Z)<=precisionZ;
				}
				/**
				 * @brief Check equality of two Coordinate objects
				 * 
				 * @param other second Coordinate that should be checked for equality
				 * @return true if the two Coordinates are equal in all dimensions
				 * @return false in all other cases
				 */
				bool operator==(const Coordinate& other)const
				{
					long lx1, ly1, lx2, ly2;
					int iz1, iz2;
					this->discretize(lx1, ly1, iz1);
					other.discretize(lx2, ly2, iz2);
					return lx1 == lx2 && ly1 == ly2 && iz1 == iz2;
				}
				/**
				 * @brief Check inequality of two Coordinate objects
				 * 
				 * @param other 
				 * @return true 
				 * @return false 
				 */
				bool operator!=(const Coordinate& other)const
				{
					long lx1, ly1, lx2, ly2;
					int iz1, iz2;
					this->discretize(lx1, ly1, iz1);
					other.discretize(lx2, ly2, iz2);
					return !(lx1 == lx2 && ly1 == ly2 && iz1 == iz2);
				}
				/**
				 * @brief Write information of the Coordinate to a string
				 * 
				 * @return std::string information of the Coordinate
				 */
				std::string toString() const
				{
					std::stringstream s;
					s<<"coord("<< std::setprecision(10) <<m_X<<","<< std::setprecision(10) <<m_Y<<","<<std::setprecision(3)<<m_Z<<")";
					return s.str();
				}
				/**
				 * @brief Get a boost_point that has the same coordinates as the Coordinate object
				 * 
				 * @return boost_point resulting boost_point with the same values for the x-, y- and z-coordinate
				 */
				boost_point getBoostPoint()
				{
					return boost_point(m_X,m_Y,m_Z);
				}
				/**
				 * @brief Check whether a Coordinate is in a certain area.
				 * 
				 * @param x_min lower x boundary of the area 
				 * @param x_max upper x boundary of the area
				 * @param y_min lower y boundary of the area
				 * @param y_max upper y boundary of the area
				 * @return true if both the x-coordinate and the y-coordinate are in the specified area
				 * @return false in all other cases
				 */
				bool isInArea(double x_min, double x_max, double y_min, double y_max) const
				{
					return m_X >= x_min && m_X <= x_max && m_Y >= y_min && m_Y <= y_max;
				}
				/**
				 * @brief Add a given Coordinates to the current Coordinate
				 * 
				 * @param other Coordiante to add
				 * @return Coordinate& resulting Coordinate
				 */
				Coordinate& operator+=(const Coordinate& other)
				{
					this->m_X = this->m_X + other.m_X;
					this->m_Y  = this->m_Y + other.m_Y;
					this->m_Z = this->m_Z + other.m_Z;

					return *this;
				}
				/**
				 * @brief Add two Coordinates
				 * 
				 * @param summer second Coordinate to add
				 * @return Coordinate new Coordinate object that results from the addition
				 */
				Coordinate operator+(const Coordinate& summer)
				{
					Coordinate newCoord;
					newCoord.m_X = this->m_X + summer.m_X;
					newCoord.m_Y  = this->m_Y + summer.m_Y;
					newCoord.m_Z = this->m_Z + summer.m_Z;
					return newCoord;
				}
				/**
				 * @brief Subtract two Coordinates
				 * 
				 * @param summer Coordinate that is the subtrahend
				 * @return Coordinate new Coordinate object that results from the subtraction
				 */
				Coordinate operator-(const Coordinate& subber)
				{
					Coordinate newCoord;
					newCoord.m_X = this->m_X - subber.m_X;
					newCoord.m_Y  = this->m_Y - subber.m_Y;
					newCoord.m_Z = this->m_Z - subber.m_Z;
					return newCoord;
				}
				/**
				 * @brief Compare two Coordinates
				 * 
				 * @param other second Coordinate to compare
				 * @return true if the first coordinate is closer to the origin than the second Coordinate
				 * @return false in all other cases
				 */
				bool operator<(const Coordinate& other) const
				{					
					return this->m_X*this->m_X+this->m_Y*this->m_Y+this->m_Z*this->m_Z
						< other.m_X*other.m_X+other.m_Y*other.m_Y+other.m_Z*other.m_Z;
				}
			};

			/**
			 * @brief a functor, which hashes a Coordinate object -> std::unordered_set<Coordinate,CoordinateHasher> amap(0);
			 * 
			 */
			struct CoordinateHasher
			{
				std::size_t operator()(const Coordinate& c) const
				{
					long lx1, ly1;
					int iz1;
					c.discretize(lx1, ly1, iz1);
					//uses bit4 to bit11 of z, bit0 to bit11 of y and bit0 to bit11 of x
					return ((iz1 >> 4) << 24) | ((ly1 << (32 - 12)) >> 8) | ((lx1 << (32 - 12)) >> (8 + 12));//12bit per X,Y with 0.2 precision --> ~820m distance --> hashing becomes really bad, if things are spaced at or above 820m
				}
			};
		}
	}
}