<!--
********************************************************************************
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
*   Daniel Heß
*   Eric Neidhardt
********************************************************************************
-->
# keepmoving_if_ros

A catkin package, which interfaces between KeepMoving-OnDemand (a public transportation dispatching system) and ROS: 
- KeepMoving disposition server REST API is queried 
- adore_if_ros_msgs are used to communicate information to automated driving system
- goals of automated driving system are set in such a way that vehicle acts as a public transport shuttle

## Journey API

The journey API is responsible to received planned journeys from the keepmoving backend. The node module is responsible for receiving the currently
planned mission for this vehicle from the backend. Note that for the sake of simplicity the module uses polling to receive the mission information.

Transmitted information:
- Shuttle id
- Timestamp

Received information:
- Current journey of shuttle: Legs with navigation goals
- Query the passenger state: Are all passengers required for Leg boarded on shuttle? Are all passengers of previous and not current Leg unboarded?

## Shuttle API

The shuttle API handles communication from the shuttle to the keepmoving backend. The node module is responsible for sending the current
shuttle state to the backend.

Transmitted information:

- Shuttle id
- Shuttle position
- Timestamp
- Shuttle state information (Driving/Parking...)

Data is transmitted on a regular basis, once per second.

## Vehicle goal-setting and vehicle behavior

- If the boarding conditions of next Leg is fulfilled, keepmoving_if_ros sends a adore_if_ros_msg::NavigationGoal, with coordinate of next Leg's endpoint.
- After receiving new NavigationGoal, vehicle will check all  safety conditions for going to Driving state
- If all conditions are fulfilled, vehicle will drive towards NavigationGoal, until target has been reached. Vehicle will switch to Parked mode and stop, if distance to target is small enough and velocity is small enough.
- Journey API can either be continuously monitored to check for changes to the current leg/journey or it can be queried, as soon as vehicle is in Parked mode.
