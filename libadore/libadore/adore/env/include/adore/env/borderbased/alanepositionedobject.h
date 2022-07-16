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
#include <adore/env/borderbased/laneposition.h>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * @brief Abstract class for Objects that are positioned by a connection to a certain lane
			 * 
			 */
			class ALanePositionedObject
			{
			public:
			/**
			 * @brief Get the LanePosition of the Object
			 * 
			 * @return const LanePosition& LanePosition of the Object
			 */
				virtual const LanePosition& getLanePosition() = 0;
				/**
				 * @brief Destroy the ALanePositionedObject
				 * 
				 */
				virtual ~ALanePositionedObject(){}
			};
		}
	}
}