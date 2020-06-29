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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/


#pragma once

#include <vector>

namespace adore
{
	namespace env
	{
		namespace MAP_AUX
		{
			/**
			 * @brief Get corner points vector from center point and radius
			 * 
			 * @param x 
			 * @param y 
			 * @param r 
			 * @return std::vector<double> 
			 */
			inline std::vector<double> getCornerPoints(double x, double y, double r)
			{
				std::vector<double> value;
				
				value.push_back(x-r);
				value.push_back(x+r);
				value.push_back(y-r);
				value.push_back(y+r);
				
				return value;
			}
		}
	}
}