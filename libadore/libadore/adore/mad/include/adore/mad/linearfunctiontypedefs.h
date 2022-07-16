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
 *   Thomas Lobig - refactored types out of lanefollowinggeometry.h
 ********************************************************************************/

#include <adore/mad/llinearpiecewisefunction.h>

 namespace adore
{
	namespace mad
	{       
        typedef adore::mad::LLinearPiecewiseFunctionM<double, 1> velocity_profile;
        typedef adore::mad::LLinearPiecewiseFunctionM<double, 3> function_type_xyz;
        typedef adore::mad::LLinearPiecewiseFunctionM<double, 2> function_type2d;
        typedef adore::mad::LLinearPiecewiseFunctionM<double, 1> function_type_scalar;
    }
}