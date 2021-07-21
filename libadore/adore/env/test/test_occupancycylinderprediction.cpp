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
 *   Daniel Heß - unit tests for occupancycylinderprediction.h
 *********************************************************************************/
#include <catch2/catch.hpp>
#include <adore/env/traffic/occupancycylinderprediction.h>

TEST_CASE( "OCStraightLinePrediction: predict constant speed", "[occupancycylinderprediction]" ) 
{
    adore::env::OCStraightLinePrediction slp;
    adore::env::traffic::Participant p;
    adore::env::OccupancyCylinderPredictionSet set;
    slp.setTMaxUTC(15.0);
    slp.setTimeHeadway(0.0);//asserts below are assuming this value to be 0
    slp.setTimeLeeway(0.0);//asserts below are assuming this value to be 0
    p.yaw_ = 0.0;
    p.length_ = 4.0;
    p.width_ = 2.0;
    p.center_(0) = 0.0;
    p.center_(1) = 0.0;
    p.vx_ = 20.0;
    p.vy_ = 0.0;
    p.observation_time_ = 5.0;
    //compute the prediciton:
    slp.predict(p,set);
    REQUIRE( set.size()==1 );
    adore::env::OccupancyCylinderPrediction prediction = set[0];
    REQUIRE( prediction.occupancy_.getOccupancyCount() >0 );
    double t0_test = p.observation_time_;
    double t1_test = p.observation_time_;
    //test time monotony of prediction
    for(auto it = prediction.occupancy_.getLevel(0).begin(); it != prediction.occupancy_.getLevel(0).end(); it++)
    {
        REQUIRE(  it->second.t0_ >= t0_test );
        REQUIRE(  it->second.t1_ >= t1_test );
        REQUIRE(  it->second.t1_ >= it->second.t0_ );
        t0_test = it->second.t0_;
        t1_test = it->second.t1_;
    }
}


TEST_CASE( "OCStraightLinePrediction: predict optional deceleration", "[occupancycylinderprediction]" ) 
{
    adore::env::OCStraightLinePrediction slp;
    adore::env::traffic::Participant p;
    adore::env::OccupancyCylinderPredictionSet set;
    slp.setTMaxUTC(15.0);
    slp.setTimeHeadway(0.0);//asserts below are assuming this value to be 0
    slp.setTimeLeeway(0.0);//asserts below are assuming this value to be 0
    p.yaw_ = 0.0;
    p.length_ = 4.0;
    p.width_ = 2.0;
    p.center_(0) = 0.0;
    p.center_(1) = 0.0;
    p.vx_ = 20.0;
    p.vy_ = 0.0;
    p.observation_time_ = 5.0;
    slp.setAMin(-4);
    //compute the prediciton:
    slp.predict(p,set);
    REQUIRE( set.size()==1 );
    adore::env::OccupancyCylinderPrediction prediction = set[0];
    REQUIRE( prediction.occupancy_.getOccupancyCount() >0 );
    double t0_test = p.observation_time_;
    double t1_test = p.observation_time_;
    //test time monotony of prediction
    for(auto it = prediction.occupancy_.getLevel(0).begin(); it != prediction.occupancy_.getLevel(0).end(); it++)
    {
        REQUIRE(  it->second.t0_ >= t0_test );
        REQUIRE(  it->second.t1_ >= t1_test );
        REQUIRE(  it->second.t1_ >= it->second.t0_ );
        t0_test = it->second.t0_;
        t1_test = it->second.t1_;
    }
}


TEST_CASE( "OCStraightLinePrediction: predict optional acceleration", "[occupancycylinderprediction]" ) 
{
    adore::env::OCStraightLinePrediction slp;
    adore::env::traffic::Participant p;
    adore::env::OccupancyCylinderPredictionSet set;
    slp.setTMaxUTC(15.0);
    slp.setTimeHeadway(0.0);//asserts below are assuming this value to be 0
    slp.setTimeLeeway(0.0);//asserts below are assuming this value to be 0
    p.yaw_ = 0.0;
    p.length_ = 4.0;
    p.width_ = 2.0;
    p.center_(0) = 0.0;
    p.center_(1) = 0.0;
    p.vx_ = 10.0;
    p.vy_ = 0.0;
    p.observation_time_ = 5.0;
    slp.setAMax(2.0);
    slp.setVMax(20.0);
    //compute the prediciton:
    slp.predict(p,set);
    REQUIRE( set.size()==1 );
    adore::env::OccupancyCylinderPrediction prediction = set[0];
    REQUIRE( prediction.occupancy_.getOccupancyCount() >0 );
    double t0_test = p.observation_time_;
    double t1_test = p.observation_time_;
    //test time monotony of prediction
    for(auto it = prediction.occupancy_.getLevel(0).begin(); it != prediction.occupancy_.getLevel(0).end(); it++)
    {
        REQUIRE(  it->second.t0_ >= t0_test );
        REQUIRE(  it->second.t1_ >= t1_test );
        REQUIRE(  it->second.t1_ >= it->second.t0_ );
        t0_test = it->second.t0_;
        t1_test = it->second.t1_;
    }
}
