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
#include <adore/env/borderbased/laneposition.h>
#include <adore/env/borderbased/alanepositionedobject.h>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * @brief This class provide information about stoplines
			 * 
			 */
			class StopLine:public ALanePositionedObject
			{
			public:
			/**
			 * @brief the enum State holds the different possible states of a stopline
			 * 
			 */
				enum State
				{
					UNKNOWN = 0,
					STOP = 1,
					CONFLICT = 2,
					GO = 3
				} m_state;
				
								
				std::map<float,float> m_speedAdvisories; /**< advised speed */
				
				double m_tswitch; /**< time to next state change */
				
				LanePosition m_position; /**< LanePosition of the stopline */
				
				int m_signalGroupID; /*<< id of the signal group */

				/**
				 * @brief Construct a new StopLine object
				 * 
				 */
				StopLine()
				{
					m_position=LanePosition();
					m_state = UNKNOWN;
					m_tswitch = 1.0e6;
				}
				/**
				 * @brief Construct a new StopLine object
				 * 
				 * @param other copy that stopline
				 */
				StopLine(const StopLine& other)
				{
					this->m_position = other.m_position;
					this->m_state = other.m_state;
					this->m_tswitch = other.m_tswitch;
				}
				/**
				 * @brief Construct a new StopLine object
				 * 
				 * @param pos LanePosition of the new stopline
				 */
				StopLine(const LanePosition& pos)
				{
					m_position = LanePosition(pos);
					m_state = UNKNOWN;
					m_tswitch = 1.0e6;
				}
				/**
				 * @brief Construct a new StopLine object
				 * 
				 * @param pos LanePosition of the new stopline
				 * @param state state of the new stopline
				 * @param tswitch time to state switch
				 */
				StopLine(const LanePosition& pos, State state,double tswitch = 1.0e6)
				{
					m_position = LanePosition(pos);
					m_state = state;
					m_tswitch = tswitch;
				}
				/**
				 * @brief Get the LanePosition
				 * 
				 * @return const LanePosition& position of the stopline
				 */
				virtual const LanePosition& getLanePosition()
				{
					return m_position;
				}
				/**
				 * @brief Translate the stop line
				 * 
				 * @param dx move the x-coordinate by this value
				 * @param dy move the y-coordinate by this value
				 * @param dz move the z-coordinate by this value
				 */
				void translate(double dx,double dy,double dz)
				{
					m_position.translate(dx,dy,dz);
				}
				void rotate(double angle, double x0=0, double y0=0)
				{
					m_position.rotate(angle, x0, y0);
				}
				/**
				 * @brief Get the switching time
				 * 
				 * @return double time when the next time switch occurs
				 */
				double getSwitchingTime()
				{
					return m_tswitch;
				}
				/**
				 * @brief Set the switching time
				 * 
				 * @param t time when the next state switch occurs
				 */
				void setSwitchingTime(double t)
				{
					m_tswitch = t;
				}
			};

		}
	}
}