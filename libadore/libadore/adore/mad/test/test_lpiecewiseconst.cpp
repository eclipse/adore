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
 *   Daniel Heß 
 ********************************************************************************/

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "adore/mad/lpiecewiseconst.h"

TEST_CASE( "testing insert(x,y) and f(x)", "[LPiecewiseConst]" ) 
{
    adore::mad::LPiecewiseConst<double,double> pwc;
    pwc.insert(0.0,1.0);
    pwc.insert(1.0,2.0);
    pwc.insert(2.0,5.0);
    REQUIRE( lconstfun.f(0.0) == 1.0 );
    REQUIRE( lconstfun.f(0.5) == 1.0 );
    REQUIRE( lconstfun.f(1.0) == 2.0 );
    REQUIRE( lconstfun.f(1.5) == 2.0 );
    REQUIRE( lconstfun.f(2.0) == 5.0 );
    REQUIRE( lconstfun.f(3.0) == 5.0 );
}

TEST_CASE( "testing limitHi()", "[LPiecewiseConst]" ) 
{
    adore::mad::LPiecewiseConst<double,double> pwc;
    pwc.insert(-1.0,1.0);
    pwc.insert(1.0,2.0);
    pwc.insert(2.0,5.0);
    REQUIRE( lconstfun.limitLo() == -1.0 );
}

TEST_CASE( "testing clear_below()", "[LPiecewiseConst]" ) 
{
    adore::mad::LPiecewiseConst<double,double> pwc;
    pwc.insert(-1.0,1.0);
    pwc.insert(1.0,2.0);
    pwc.insert(2.0,5.0);
    pwc.insert(3.0,6.0);
    pwc.clear_below(2.0);
    REQUIRE( lconstfun.limitLo() == 2.0 );
    pwc.insert(-1.0,1.0);
    pwc.insert(1.0,2.0);
    pwc.clear_below(0.5);
    REQUIRE( lconstfun.limitLo() == 1.0 );
}

TEST_CASE( "testing clear_above()", "[LPiecewiseConst]" ) 
{
    adore::mad::LPiecewiseConst<double,double> pwc;
    pwc.insert(-1.0,1.0);
    pwc.insert(1.0,2.0);
    pwc.insert(2.0,5.0);
    pwc.insert(3.0,6.0);
    pwc.clear_above(2.0);
    REQUIRE( lconstfun.f(3.5) == 5.0 );
}


