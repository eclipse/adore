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
 *   Daniel Heß - unit tests for controlledconnection.h
 *********************************************************************************/

#include <catch2/catch.hpp>
#include "test_controlledconnection.h"

TEST_CASE( "ControlledConnectionSet: add one", "[controlledconnection]" ) 
{
    adore::env::test::TestConnectionFeed* f = new adore::env::test::TestConnectionFeed();
    adore::env::ControlledConnectionSet o(f);
    f->data_.push_back(adore::env::ControlledConnection(0.0,0.0,0.0,10.0,10.0,0.0));
    o.update(1.0,0.0,0.0);
    REQUIRE( o.size() == 1 );
}

TEST_CASE( "ControlledConnectionSet: add twins", "[controlledconnection]" ) {
    adore::env::test::TestConnectionFeed* f = new adore::env::test::TestConnectionFeed();
    adore::env::ControlledConnectionSet o(f);
    f->data_.push_back(adore::env::ControlledConnection(0.0,0.0,0.0,10.0,10.0,0.0));
    f->data_.push_back(adore::env::ControlledConnection(0.0,0.0,0.0,10.0,10.0,0.0));
    o.update(1.0,0.0,0.0);
    REQUIRE( o.size() == 1 );
}

TEST_CASE( "ControlledConnectionSet: add opposite", "[controlledconnection]" ) {
    adore::env::test::TestConnectionFeed* f = new adore::env::test::TestConnectionFeed();
    adore::env::ControlledConnectionSet o(f);
    f->data_.push_back(adore::env::ControlledConnection(0.0,0.0,0.0,10.0,10.0,0.0));
    f->data_.push_back(adore::env::ControlledConnection(10.0,10.0,0.0,0.0,0.0,0.0));
    o.update(1.0,0.0,0.0);
    REQUIRE( o.size() == 2 );
}

TEST_CASE( "ControlledConnectionSet: add many", "[controlledconnection]" ) {
    adore::env::test::TestConnectionFeed* f = new adore::env::test::TestConnectionFeed();
    adore::env::ControlledConnectionSet o(f);
    for(int i=0;i<100;i++)f->data_.push_back(adore::env::ControlledConnection(0.0,0.0,0.0,10.0,10.0,0.0));
    o.update(1.0,0.0,0.0);
    REQUIRE( o.size() == 1 );
}

TEST_CASE( "ControlledConnectionSet: scope", "[controlledconnection]" ) {
    adore::env::test::TestConnectionFeed* f = new adore::env::test::TestConnectionFeed();
    adore::env::ControlledConnectionSet o(f);
    f->data_.push_back(adore::env::ControlledConnection(0.0,0.0,0.0,10.0,10.0,0.0));
    f->data_.push_back(adore::env::ControlledConnection(100.100,0.0,0.0,110.0,110.0,0.0));
    o.update(50.0,0.0,0.0);
    REQUIRE( o.size() == 1 );
}

