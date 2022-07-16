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
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/

#pragma once
namespace adore
{
	namespace params
	{
		/**
		 * @brief abstract class containing parameters concerning emergency operation behaviour
		 * 
		 */
		class APEmergencyOperation
		{
		public:
			//lateral path tracking gain for terminal maneuver
			virtual double getKy()const =0;
			//lateral path tracking gain for terminal maneuver
			virtual double getKpsi()const =0;
			//hard coded longitudinal minimum acceleration
			virtual double getamin()const =0;
			//hard coded maximum longitudinal acceleration
			virtual double getamax()const =0;
			virtual double getDeltaMax()const=0;
			virtual double getDeltaMin()const =0;
			virtual double getEmergencyManeuverAMin()const =0;
			virtual double getEmergencyManeuverAMax()const =0;
			virtual double getEmergencyManeuverAStall()const =0;
			virtual double getEmergencyManeuverTStall()const =0;
			virtual double getEmergencyManeuverJMin()const =0;
		};
	}
}