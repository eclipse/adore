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
#include <adore/mad/alfunction.h>
#include <adore/mad/rotations.h>

namespace adore
{
	namespace mad
	{
		/**
		 * Create a 2d-path, which is normally offset at a given distance from another path.
		 * @param path the reference path
		 * @param heading the heading of the reference path
		 * @param distance the offset
		 * @return the resulting path
		 */
		template<typename T>
		ALFunction<T, adoreMatrix<T, 2, 1>>* create_normalOffset(ALFunction<T, adoreMatrix<T, 2, 1>>* path, ALFunction<T, T>* heading, ALFunction<T, T>* distance)
		{
			return funop::add(
				path,
				funop::rotate(
					heading,
					funop::stack(
						new LConstFun<T, T>((T)0,path->limitLo(),path->limitHi()),  //0
						distance
					)
				)
			);
		}
	}
}