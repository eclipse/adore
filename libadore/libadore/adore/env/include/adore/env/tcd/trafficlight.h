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

#include <adore/env/tcd/trafficcontroldevice.h>
#include <unordered_map>

namespace adore
{
	namespace env
	{
		/**  
		* Definition for the traffic light phases based on the ISO/TS 19091:2017
		* F.4.4.4 DE_BallLight
		* permissive-Movement-Allowed (1),
		* -- Driver Action:
		* -- Proceed with caution,
		* -- must yield to all conﬂicting traffc
		* -- Conﬂicting traffc may be present
		*  permissive-clearance (2),
		* -- Driver Action:
		* -- Prepare to stop.
		* -- Proceed if unable to stop,
		* -- Clear Intersection.
		* -- Conﬂicting traffc may be present
		*  stop-And-Remain (3),
		* -- Driver Action:
		* -- Stop vehicle at stop line.
		* -- Do not proceed.
		*  caution-Conﬂicting-Traffc (4),
		* -- Driver Action:
		* -- Proceed with caution
		*  stop-Then-Proceed (5),
		* -- Driver Action:
		* -- Stop vehicle at stop line.
		* -- Do not proceed unless it is safe.
		*/
		enum class TrafficLightColor
		{
		GREEN = 1,
		YELLOW = 2,
		RED = 3,
		RED_YELLOW = 4,
		YELLOW_FLASHING = 5,
		UNDEFINED_COLOR = 99
		};

		/**
		* holds traffic light state and validity timestamp
		*/
		class TrafficLightStatus
		{
		private:

			TrafficLightColor m_currentColor;

			long m_validUntilTimestamp;

		public:

			/*
			* saving zones with speed advisory if supported by traffic light (e.g. using MAPEM/SPATEM)
			* first parameter is the distance limit starting from position of limitline, second the allowed velocity e.g. [0 - ]100m : 3m/s ; 200m : 5m/s, etc. 
			*/
			std::map<float,float> distanceSpeedTuples;

            TrafficLightStatus(TrafficLightColor currentcolor = TrafficLightColor::UNDEFINED_COLOR,
                               long validUntilTimestamp = 0)
              : m_currentColor(currentcolor), m_validUntilTimestamp(validUntilTimestamp)
            {
            }
		
			TrafficLightColor getCurrentColor() const
			{
				return m_currentColor;
			}

			void setCurrentColor(TrafficLightColor color)
			{
				m_currentColor = color;
			}
			void setValidUntilTimestamp(long timestamp)
			{
				m_validUntilTimestamp = timestamp;
			}
			long getValidUntilTimestamp() const
			{
				return m_validUntilTimestamp;
			}

			std::string toString()
			{
				switch (getCurrentColor())
				{
				case TrafficLightColor::RED:
					return "RED";
				case TrafficLightColor::RED_YELLOW:
					return "RED_YELLOW";
				case TrafficLightColor::YELLOW:
					return "YELLOW";
				case TrafficLightColor::GREEN:
					return "GREEN";
				default:
					return "UNDEFINED_COLOR";
				}
			}


			class TrafficLight;
			/**
			 * generates speedadvisories based on the durations defined in the TrafficLight
			 * exception: 50 m/s is defined as the velocity value that shall inform the driver 
			 * that there is no possibility to reach the traffic light while tl is in green state.
			 *    
			 * @param tl reference to the traffic light that shall hold the speed advisories
			 * @param speedLimit preference for vmax - default is 10 m/s  
			 */ 
			static void GenerateSpeedAdvisories(adore::env::TrafficLightStatus & state, float speedLimit = 10.0f)
			{
				state.distanceSpeedTuples.clear();

				float minimumSpeed = speedLimit * 0.6;
				float timeToOther = state.getValidUntilTimestamp()/1000;

				switch(state.getCurrentColor())
				{
				case adore::env::TrafficLightColor::GREEN:
					state.distanceSpeedTuples[speedLimit * timeToOther * 0.5] = minimumSpeed;
					state.distanceSpeedTuples[speedLimit * timeToOther * 0.6] = speedLimit * 0.7;
					state.distanceSpeedTuples[speedLimit * timeToOther * 0.7] = speedLimit * 0.8;
					state.distanceSpeedTuples[speedLimit * timeToOther * 0.95] = speedLimit;
					state.distanceSpeedTuples[speedLimit * timeToOther * 5] = 50.0f;
					break;
				case adore::env::TrafficLightColor::YELLOW:
					break;
				case adore::env::TrafficLightColor::RED:
					state.distanceSpeedTuples[minimumSpeed * timeToOther] = 50.0f;
					state.distanceSpeedTuples[speedLimit * 0.7 * timeToOther] = speedLimit * 0.7;
					state.distanceSpeedTuples[speedLimit * 0.8 * timeToOther] = speedLimit * 0.8;
					state.distanceSpeedTuples[speedLimit * 0.9 * timeToOther] = speedLimit * 0.9;
					state.distanceSpeedTuples[speedLimit * 1.5 * timeToOther] = speedLimit;
					break;
				case adore::env::TrafficLightColor::RED_YELLOW:
					break;
				case adore::env::TrafficLightColor::YELLOW_FLASHING:
					break;
				case adore::env::TrafficLightColor::UNDEFINED_COLOR:
					break;
				}
			}
		};

		/**
		*	A traffic light is basically a traffic control device with a color status.
		*/
		class TrafficLight : public TrafficControlDevice
		{
		private:

			TrafficLightStatus status_;

		public:

			/*
			* used as signal group ID, controlling multiple lanes via one tl-controller
			*/
			int movement_id_;

			/*
			* deescribes the intersection that this traffic light is applied to
			*/
			int intersection_id_;

			TrafficLight()
			{
				setType(TrafficControlDevice::TRAFFIC_LIGHT);
			}

			virtual TrafficLightStatus* getStatus()
			{
				return &status_;
			}

			virtual TrafficLightStatus const * getStatus() const
			{
				return &status_;
			}

			bool operator==(const TrafficLight& other) {

				if(getCoordinate() == other.getCoordinate()
					&& intersection_id_ == other.intersection_id_
					&& movement_id_ == other.movement_id_
					&& getID() == other.getID())
					return true;

				return false;
			}

		};

		/**
		 * SimTrafficLight is an extension of TrafficLight with additional information for simulations
		 * like the durations (in ms) for the 4 different states of the traffic light and the start state at the
		 * beginning of a simulation.
		 */
		class SimTrafficLight : public TrafficLight
		{
			public:
			SimTrafficLight() : TrafficLight() {};
				
			// all durations are specified in ms 	
			int red_duration_;

			int yellow_duration_;

			int green_duration_;

			int yellow_red_duration_;

			TrafficLightColor start_state_;

			int time_to_red_;

			int time_to_red_yellow_;

			int time_to_yellow_;

			int time_to_green_;

		};

		typedef std::unordered_map<adore::env::BorderBased::Coordinate, adore::env::SimTrafficLight,adore::env::BorderBased::CoordinateHasher> SimTrafficLightMap;
		typedef std::unordered_map<adore::env::BorderBased::Coordinate, adore::env::TrafficLight,adore::env::BorderBased::CoordinateHasher> TrafficLightMap;
  }
}
