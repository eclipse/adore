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
 ********************************************************************************/


#pragma once

#include <vector>
#include "adore/env/borderbased/border.h"

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			namespace TYPEDEFS
			{
				typedef std::pair<Coordinate,Coordinate> TPointPair;
				typedef std::pair<TPointPair,BorderType::TYPE> TPointPairAttributes;
				typedef std::vector<TPointPairAttributes> TVectorPointsWithAttributes;
			}
		}
	}
}