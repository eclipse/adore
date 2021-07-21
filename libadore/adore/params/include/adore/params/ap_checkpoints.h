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
		 * @brief abstract class containing parameters for checkpoint handling
		 * 
		 */
		class APCheckpoints
		{
		public:
			///returns distance value for resetting current checkpoint
			virtual double getClearDistance()const =0;

			///returns timeout for resetting current checkpoint
			virtual double getClearTimeout()const =0;
		};
	}
}