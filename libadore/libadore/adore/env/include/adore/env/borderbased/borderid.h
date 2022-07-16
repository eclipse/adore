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
 *   Stephan Lapoehn - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/env/borderbased/coordinate.h>
#include <string>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * @brief This struct identifies a Border by the coordinates of the starting and the end point.
			 * 
			 */
			struct BorderID
			{
				Coordinate m_first, m_last; /**< starting and end point */
				BorderID() {}
				BorderID(Coordinate first, Coordinate last) :m_first(first), m_last(last) {}
				/**
				 * @brief Translate a border
				 * 
				 * @param dx move x-coordinate by this value
				 * @param dy move y-coordinate by this value
				 * @param dz move z-coordinate by this value
				 */
				void translate(double dx,double dy,double dz)
				{
					m_first.translate(dx,dy,dz);
					m_last.translate(dx,dy,dz);
				}
				/**
				 * @brief returns sum of distance between this.m_first and other.m_first and distance between this.m_last + other.m_last
				 * @param other the BorderID to compare with
				 */
				double distance(const BorderID& other)const
				{
					return this->m_first.distance(other.m_first)
					     + this->m_last.distance(other.m_last);
				}
				/**
				 * @brief returns distance between coordinates in id
				 */
				double distance()const
				{
					return this->m_first.distance(this->m_last);
				}
				/**
				 * @return the distance between first and last coordinate
				 */
				double getLength()
				{
					const double dx = m_first.m_X-m_last.m_X;
					const double dy = m_first.m_Y-m_last.m_Y;
					const double dz = m_first.m_Z-m_last.m_Z;
					return std::sqrt(dx*dx+dy*dy+dz*dz);
				}

				/**
				 * @brief 
				 * 
				 */
				void rotate(double angle, double x0=0.0, double y0=0.0)
				{
					m_first.rotate(angle, x0, y0);
					m_last.rotate(angle, x0, y0);
				}
				/**
				 * @brief Check equality of two BorderIDs
				 * 
				 * @param other second BorderID that should be checked for equality
				 * @return true if BorderIDs are equal
				 * @return false if BorderIDs are different
				 */
				bool operator==(const BorderID& other)const
				{
					return this->m_first == other.m_first && this->m_last == other.m_last;
				}
				/**
				 * @brief returns the inverse of this
				 */
				BorderID inverse()const
				{
					return BorderID(m_last,m_first);
				}
				/**
				 * @brief Compare two BorderIDs
				 * 
				 * @param other second BorderID to compare
				 * @return true if starting point of this is closer to the origin than starting point of other
				 * @return false in all other cases
				 */
				bool operator<(const BorderID& other)const
				{
					double tx=this->m_first.m_X;
					double ty=this->m_first.m_Y;
					double tz=this->m_first.m_Z;
					double ox=other.m_first.m_X;
					double oy=other.m_first.m_Y;
					double oz=other.m_first.m_Z;
					return std::sqrt(tx*tx+ty*ty+tz*tz)<std::sqrt(ox*ox+oy*oy+oz*oz);
				}
				
				//bool operator<(const BorderID* rhs) 
				//{
				//	unsigned int hash = BorderBased::BorderIDHasher()(*this);
				//	unsigned int hashr = BorderBased::BorderIDHasher()(*rhs);
				//	return hash < hashr;
				//}

				/**
				 * @brief Write information of the BorderID to a string
				 * 
				 * @return std::string information of the BorderID
				 */
				std::string toString() const
				{
					std::stringstream s;
					s<<"BID<"<<m_first.toString()<<";"<<m_last.toString()<<">";
					return s.str();
				}

				/**
				 * @brief Check whether the starting point and the end point of the BorderID are in a certain area
				 * 
				 * @param x_min lower x boundary of the area
				 * @param x_max upper x boundary of the area
				 * @param y_min lower y boundary of the area
				 * @param y_max upper y boundary of the area
				 * @return true if both the starting and the end point are in the specified area
				 * @return false in all other cases
				 */
				bool isInArea(double x_min, double x_max, double y_min, double y_max) const
				{
					return m_first.isInArea(x_min, x_max, y_min, y_max) && m_last.isInArea(x_min, x_max, y_min, y_max);
				}
				/**
				 * @brief Get a BorderID with the reverse direction of a given BorderID
				 * 
				 * The starting point and the end point of the resulting BorderID are switched.
				 * 
				 * @param b basis for the BorderID with reverse direction
				 * @return BorderID witch switched starting and end point
				 */
				BorderID getReverseDirectionID(const BorderID& b)
				{
					auto test = BorderID(b.m_last,b.m_first);
					return test;
				}
				BorderID getReverseDirectionID()const
				{
					return BorderID(m_last,m_first);
				}				
			};

			/**
			 * @brief a functor, which hashes a BorderID object -> std::unordered_set<BorderID,BorderIDHasher> amap(0);
			 * 
			 */
			struct BorderIDHasher
			{
				CoordinateHasher ch;
				std::size_t operator()(const BorderID& b) const
				{
					std::size_t first = ch(b.m_first);
					std::size_t last = ch(b.m_last);
					return first ^ (last << 16) ^ (last >> 16);//first=[fhi,flo], last=[lhi,llo] --> return [fhi xor llo, flo xor lhi]
				}
			};
		}
	}
}