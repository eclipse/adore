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
#include <adore/env/borderbased/borderid.h>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * @brief This is a struct that contains a position defined by a BorderID and a progress on that border
			 * 
			 */
			struct LanePosition
			{
				BorderID m_rightID; /**< BorderID */
				double m_progress; /**< progress */
				/**
				 * @brief Construct a new LanePosition object
				 * 
				 */
				LanePosition() {}
				/**
				 * @brief Construct a new LanePosition
				 * 
				 * @param id BorderID to speify the Position
				 * @param progress Progress to specify the Position relative to the border
				 */
				LanePosition(const BorderID& id, double progress)
				{
					m_rightID = BorderID(id);
					m_progress = progress;
				}
				/**
				 * @brief Construct a new LanePosition
				 * 
				 * @param pos copy this LanePosition
				 */
				LanePosition(const LanePosition& pos)
				{
					m_rightID = BorderID(pos.m_rightID);
					m_progress = pos.m_progress;
				}
				/**
				 * @brief Translate the LanePosition by translating the BorderID
				 * 
				 * @param dx move x-coordinate by this value
				 * @param dy move y-coordinate by this value
				 * @param dz move z-coordinate by this value
				 */
				void translate(double dx,double dy,double dz)
				{
					m_rightID.translate(dx,dy,dz);
				}
				void rotate(double angle, double x0=0, double y0=0)
				{
					m_rightID.rotate(angle, x0, y0);
				}
				/**
				 * @brief Check two LanePositions for equality
				 * 
				 * @param other second LanePosition for the comparison
				 * @return true if the LanePositions are equal
				 * @return false if the LanePositions are different
				 */
				bool operator==(const LanePosition& other)const
				{
					return this->m_progress == other.m_progress &&
						this->m_rightID.m_first == other.m_rightID.m_first &&
						this->m_rightID.m_last == other.m_rightID.m_last;
				}
				/**
				 * @brief Extract the information of the LanePosition to a string
				 * 
				 * @return std::string that holds the information
				 */
				std::string toString() const
				{
					std::stringstream s;
					s<<m_rightID.toString()<<"@"<<std::setprecision(3)<<m_progress;
					return s.str();
				}
			};
			struct LanePositionHasher
			{
				std::size_t operator()(const LanePosition& c) const
				{
					using::std::hash;
					using::std::size_t;

					BorderIDHasher BIDhasher;
					size_t hasherBID = BIDhasher(c.m_rightID);

					return (hash<double>()(c.m_progress) ^ hasherBID);
				}

				std::size_t operator()(const LanePosition* c) const
				{
					using::std::hash;
					using::std::size_t;

					BorderIDHasher BIDhasher;
					size_t hasherBID = BIDhasher(c->m_rightID);

					return (hash<double>()(c->m_progress) ^ hasherBID);
				}
			};
		}
	}
}