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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "adore/mad/alfunction.h"

adore::mad::LConstFun<double,double> lconstfun {1.0, -10.0, 10.0};

TEST_CASE( "testing LConstFun", "[alfunction]" ) {
    REQUIRE( lconstfun.limitHi() == 10.0 );
    lconstfun.setLimits(-0.5,0.5);
    REQUIRE( lconstfun.limitHi() == .5 );
}

TEST_CASE( "testing LConstFun case 2", "[alfunction]" ) {
    REQUIRE( lconstfun.limitHi() == 10.0 );
}