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
 *   Daniel Heß - unit tests for setpointrequestswath.h
 *********************************************************************************/

#include <catch2/catch.hpp>
#include <adore/fun/safety/setpointrequestswath.h>

TEST_CASE( "SetPointRequestSwath: straight SPR, time monotony", "[setpointrequestswath]" ) 
{
    double v = 10.0;
    double psi = -2.1;
    double cpsi = std::cos(psi);
    double spsi = std::sin(psi);
    adore::fun::SetPointRequest spr;
    for(double t=0;t<10.0;t+=0.1)
    {
        adore::fun::SetPoint sp;
        sp.tStart = t;
        sp.tEnd = t+0.1;
        sp.x0ref.setX(v*t*cpsi);
        sp.x0ref.setY(v*t*spsi);
        sp.x0ref.setPSI(psi);
        sp.x0ref.setvx(v);
        spr.setPoints.push_back(sp);
    }
    double l = 5;
    double w = 1.8;
    double p = 1;
    double lat_precision = 0.2;
    double lat_error = 0.1;
    double lon_error = 1.0;
    adore::fun::SetPointRequestSwath sprs(l,w,p,lat_precision,lat_error,lon_error);
    adore::mad::OccupancyCylinderTree prediction;
    sprs.append_cylinder_swath_linear(spr,prediction);
    //test time monotony of prediction
    double t0_test = spr.setPoints.front().tStart;
    double t1_test = spr.setPoints.front().tStart;
    for(auto it = prediction.getLevel(0).begin(); it != prediction.getLevel(0).end(); it++)
    {
        //note: INFO output misses when executing "make test", but is reported when executing binary in build/adore/fun/test
        INFO("t0="<<it->second.t0_);
        INFO("t1="<<it->second.t1_);
        REQUIRE(  it->second.t0_ >= t0_test );
        REQUIRE(  it->second.t1_ >= t1_test );
        REQUIRE(  it->second.t1_ >= it->second.t0_ );
        t0_test = it->second.t0_;
        t1_test = it->second.t1_;
    }
}

TEST_CASE( "SetPointRequestSwath: very short spr", "[setpointrequestswath]" ) 
{
    double v = 10.0;
    double psi = -2.1;
    double cpsi = std::cos(psi);
    double spsi = std::sin(psi);
    adore::fun::SetPointRequest spr;
    for(double t=0;t<0.2;t+=0.1)
    {
        adore::fun::SetPoint sp;
        sp.tStart = t;
        sp.tEnd = t+0.1;
        sp.x0ref.setX(v*t*cpsi);
        sp.x0ref.setY(v*t*spsi);
        sp.x0ref.setPSI(psi);
        sp.x0ref.setvx(v);
        spr.setPoints.push_back(sp);
    }
    double l = 5;
    double w = 1.8;
    double p = 1;
    double lat_precision = 0.2;
    double lat_error = 0.1;
    double lon_error = 1.0;
    adore::fun::SetPointRequestSwath sprs(l,w,p,lat_precision,lat_error,lon_error);
    adore::mad::OccupancyCylinderTree prediction;
    sprs.append_cylinder_swath_linear(spr,prediction);
    //test time monotony of prediction
    double t0_test = spr.setPoints.front().tStart;
    double t1_test = spr.setPoints.front().tStart;
    for(auto it = prediction.getLevel(0).begin(); it != prediction.getLevel(0).end(); it++)
    {
        //note: INFO output misses when executing "make test", but is reported when executing binary in build/adore/fun/test
        INFO("t0="<<it->second.t0_);
        INFO("t1="<<it->second.t1_);
        REQUIRE(  it->second.t0_ >= t0_test );
        REQUIRE(  it->second.t1_ >= t1_test );
        REQUIRE(  it->second.t1_ >= it->second.t0_ );
        t0_test = it->second.t0_;
        t1_test = it->second.t1_;
    }
}