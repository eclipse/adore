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
 *   Daniel Heß - unit tests for occupancycylinder.h
 *********************************************************************************/

#include <catch2/catch.hpp>
#include <adore/mad/occupancycylinder.h>

TEST_CASE( "OccupancyCylinderTree: add one", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinderTree oct;
    adore::mad::OccupancyCylinder oc(5.0,0.0,0.0,1.0,2.0);
    oct.insert(oc);
    REQUIRE( oct.getOccupancyCount() == 1 );
}

TEST_CASE( "OccupancyCylinderTree: add 16", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinderTree oct;
    adore::mad::OccupancyCylinder oc(5.0,0.0,0.0,1.0,2.0);
    for(int i=0;i<16;i++)oct.insert(oc);
    REQUIRE( oct.getOccupancyCount() == 16 );
}

TEST_CASE( "OccupancyCylinderTree: compute_all_levels 00", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinderTree oct;
    oct.setPreferredBranchingFactor(2);
    adore::mad::OccupancyCylinder oc(5.0,0.0,0.0,1.0,2.0);
    for(int i=0;i<16;i++)oct.insert(oc);
    oct.compute_all_levels();
    REQUIRE( oct.getLevelCount() == 4+1 );
}

TEST_CASE( "OccupancyCylinderTree: compute_all_levels 01", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinderTree oct;
    oct.setPreferredBranchingFactor(2);
    adore::mad::OccupancyCylinder oc01(5.0,0.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(5.0,10.0,0.0,1.0,2.0);
    oct.insert(oc01);
    oct.insert(oc02);
    oct.compute_all_levels();
    REQUIRE( oct.getLevelCount() == 2 );
}

TEST_CASE( "OccupancyCylinderTree: compute_all_levels 02", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinderTree oct;
    oct.setPreferredBranchingFactor(2);
    adore::mad::OccupancyCylinder oc01(5.0,0.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(5.0,10.0,0.0,1.0,2.0);
    oct.insert(oc01);
    oct.insert(oc02);
    oct.compute_all_levels();
    REQUIRE( oct.getLevel(1).size() == 1 );
}

TEST_CASE( "OccupancyCylinderTree: compute_all_levels 03", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinderTree oct;
    oct.setPreferredBranchingFactor(2);
    adore::mad::OccupancyCylinder oc01(5.0,0.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(5.0,10.0,0.0,1.0,2.0);
    oct.insert(oc01);
    oct.insert(oc02);
    oct.compute_all_levels();
    auto data = oct.getLevel(1)[0];
    REQUIRE( data.first.first == 0 );
    REQUIRE( data.first.second == 1 );
}

TEST_CASE( "OccupancyCylinderTree: compute_all_levels 04", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinderTree oct;
    oct.setPreferredBranchingFactor(2);
    adore::mad::OccupancyCylinder oc01(5.0,0.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(5.0,10.0,0.0,1.0,2.0);
    oct.insert(oc01);
    oct.insert(oc02);
    oct.compute_all_levels();
    auto data = oct.getLevel(1)[0];
    REQUIRE( data.second.x_ == 5.0 );
    REQUIRE( data.second.y_ == 0.0 );
}

TEST_CASE( "OccupancyCylinderTree: compute_all_levels 05", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinderTree oct;
    oct.setPreferredBranchingFactor(2);
    adore::mad::OccupancyCylinder oc01(5.0,0.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(5.0,10.0,0.0,1.0,2.0);
    oct.insert(oc01);
    oct.insert(oc02);
    oct.compute_all_levels();
    auto data = oct.getLevel(1)[0];
    REQUIRE( data.second.rxy_ == 10.0 );
}
TEST_CASE( "OccupancyCylinderTree: collidesWith case-01", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinder oc01(2.0,0.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(2.0,0.0,0.0,1.0,2.0);
    
    adore::mad::OccupancyCylinderTree oct1;
    oct1.setPreferredBranchingFactor(2);
    oct1.insert(oc01);

    adore::mad::OccupancyCylinderTree oct2;
    oct2.setPreferredBranchingFactor(2);
    oct2.insert(oc02);

    REQUIRE( oct1.collidesWith(oct2) == true );
}
TEST_CASE( "OccupancyCylinderTree: collidesWith case-02", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinder oc01(2.0,0.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(2.0,3.5,0.0,1.0,2.0);
    
    adore::mad::OccupancyCylinderTree oct1;
    oct1.setPreferredBranchingFactor(2);
    oct1.insert(oc01);

    adore::mad::OccupancyCylinderTree oct2;
    oct2.setPreferredBranchingFactor(2);
    oct2.insert(oc02);

    REQUIRE( oct1.collidesWith(oct2) == true );
}
TEST_CASE( "OccupancyCylinderTree: collidesWith case-03", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinder oc01(2.0,0.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(2.0,2.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc03(2.0,4.0,0.0,1.0,2.0);
    
    adore::mad::OccupancyCylinder oc04(2.0,2.0,0.0,1.0,2.0);

    adore::mad::OccupancyCylinderTree oct1;
    oct1.setPreferredBranchingFactor(3);
    oct1.insert(oc01);
    oct1.insert(oc02);
    oct1.insert(oc03);
    oct1.compute_all_levels();

    adore::mad::OccupancyCylinderTree oct2;
    oct2.insert(oc04);

    REQUIRE( oct1.collidesWith(oct2) == true );
}

TEST_CASE( "OccupancyCylinderTree: collidesWith case-04", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinder oc01(2.0,0.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(2.0,2.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc03(2.0,4.0,0.0,1.0,2.0);
    
    adore::mad::OccupancyCylinder oc04(2.0,2.0,-2.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc05(2.0,2.0,0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc06(2.0,2.0,2.0,1.0,2.0);

    adore::mad::OccupancyCylinderTree oct1;
    oct1.setPreferredBranchingFactor(3);
    oct1.insert(oc01);
    oct1.insert(oc02);
    oct1.insert(oc03);
    oct1.compute_all_levels();

    adore::mad::OccupancyCylinderTree oct2;
    oct2.setPreferredBranchingFactor(3);
    oct2.insert(oc04);
    oct2.insert(oc05);
    oct2.insert(oc06);
    oct2.compute_all_levels();

    REQUIRE( oct1.collidesWith(oct2) == true );
}

TEST_CASE( "OccupancyCylinderTree: collidesWith case 00", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinder oc01(2.0,0.0,-3.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(2.0,0.0,-2.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc03(2.0,0.0,-1.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc04(2.0,0.0, 0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc05(2.0,0.0, 1.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc06(2.0,0.0, 2.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc07(2.0,0.0, 3.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc08(2.0,0.0, 4.0,1.0,2.0);
    
    adore::mad::OccupancyCylinder oc09(2.0,-3,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc10(2.0,-2,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc11(2.0,-1,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc12(2.0, 0,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc13(2.0, 1,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc14(2.0, 2,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc15(2.0, 3,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc16(2.0, 4,.0,1.0,2.0);

    adore::mad::OccupancyCylinderTree oct1;
    oct1.setPreferredBranchingFactor(2);
    oct1.insert(oc01);oct1.insert(oc02);oct1.insert(oc03);oct1.insert(oc04);oct1.insert(oc05);oct1.insert(oc06);oct1.insert(oc07);oct1.insert(oc08);

    adore::mad::OccupancyCylinderTree oct2;
    oct2.setPreferredBranchingFactor(2);
    oct2.insert(oc09);oct2.insert(oc10);oct2.insert(oc11);oct2.insert(oc12);oct2.insert(oc13);oct2.insert(oc14);oct2.insert(oc15);oct2.insert(oc16);

    REQUIRE( oct1.collidesWith(oct2) == true );
}


TEST_CASE( "OccupancyCylinderTree: collidesWith case 01", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinder oc01(2.0,0.0,-3.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(2.0,0.0,-2.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc03(2.0,0.0,-1.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc04(2.0,0.0, 0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc05(2.0,0.0, 1.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc06(2.0,0.0, 2.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc07(2.0,0.0, 3.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc08(2.0,0.0, 4.0,1.0,2.0);
    
    adore::mad::OccupancyCylinder oc09(2.0,-3,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc10(2.0,-2,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc11(2.0,-1,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc12(2.0, 0,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc13(2.0, 1,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc14(2.0, 2,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc15(2.0, 3,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc16(2.0, 4,.0,1.0,2.0);

    adore::mad::OccupancyCylinderTree oct1;
    oct1.setPreferredBranchingFactor(2);
    oct1.insert(oc01);oct1.insert(oc02);oct1.insert(oc03);oct1.insert(oc04);oct1.insert(oc05);oct1.insert(oc06);oct1.insert(oc07);oct1.insert(oc08);
    oct1.compute_all_levels();

    adore::mad::OccupancyCylinderTree oct2;
    oct2.setPreferredBranchingFactor(2);
    oct2.insert(oc09);oct2.insert(oc10);oct2.insert(oc11);oct2.insert(oc12);oct2.insert(oc13);oct2.insert(oc14);oct2.insert(oc15);oct2.insert(oc16);
    oct2.compute_all_levels();

    REQUIRE( oct1.collidesWith(oct2) == true );
}


TEST_CASE( "OccupancyCylinderTree: collidesWith case 02", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinder oc01(2.0,0.0,-3.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(2.0,0.0,-2.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc03(2.0,0.0,-1.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc04(2.0,0.0, 0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc05(2.0,0.0, 1.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc06(2.0,0.0, 2.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc07(2.0,0.0, 3.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc08(2.0,0.0, 4.0,1.0,2.0);
    
    adore::mad::OccupancyCylinder oc09(2.0,-3,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc10(2.0,-2,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc11(2.0,-1,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc12(2.0, 0,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc13(2.0, 1,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc14(2.0, 2,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc15(2.0, 3,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc16(2.0, 4,.0,1.0,2.0);

    adore::mad::OccupancyCylinderTree oct1;
    oct1.setPreferredBranchingFactor(2);
    oct1.insert(oc01);oct1.insert(oc02);oct1.insert(oc03);oct1.insert(oc04);oct1.insert(oc05);oct1.insert(oc06);oct1.insert(oc07);oct1.insert(oc08);
    oct1.setLevelCount(2);

    adore::mad::OccupancyCylinderTree oct2;
    oct2.setPreferredBranchingFactor(2);
    oct2.insert(oc09);oct2.insert(oc10);oct2.insert(oc11);oct2.insert(oc12);oct2.insert(oc13);oct2.insert(oc14);oct2.insert(oc15);oct2.insert(oc16);
    oct2.setLevelCount(2);

    REQUIRE( oct1.collidesWith(oct2) == true );
}


TEST_CASE( "OccupancyCylinderTree: collidesWith case 03", "[occupancycylinder]" ) 
{
    adore::mad::OccupancyCylinder oc01(2.0,0.0,-3.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc02(2.0,0.0,-2.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc03(2.0,0.0,-1.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc04(2.0,0.0, 0.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc05(2.0,0.0, 1.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc06(2.0,0.0, 2.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc07(2.0,0.0, 3.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc08(2.0,0.0, 4.0,1.0,2.0);
    
    adore::mad::OccupancyCylinder oc09(2.0,-3,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc10(2.0,-2,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc11(2.0,-1,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc12(2.0, 0,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc13(2.0, 1,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc14(2.0, 2,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc15(2.0, 3,.0,1.0,2.0);
    adore::mad::OccupancyCylinder oc16(2.0, 4,.0,1.0,2.0);

    adore::mad::OccupancyCylinderTree oct1;
    oct1.setPreferredBranchingFactor(2);
    oct1.insert(oc01);oct1.insert(oc02);oct1.insert(oc03);oct1.insert(oc04);oct1.insert(oc05);oct1.insert(oc06);oct1.insert(oc07);oct1.insert(oc08);
    oct1.setLevelCount(2);

    adore::mad::OccupancyCylinderTree oct2;
    oct2.setPreferredBranchingFactor(3);
    oct2.insert(oc09);oct2.insert(oc10);oct2.insert(oc11);oct2.insert(oc12);oct2.insert(oc13);oct2.insert(oc14);oct2.insert(oc15);oct2.insert(oc16);
    oct2.setLevelCount(2);

    REQUIRE( oct1.collidesWith(oct2) == true );
}

TEST_CASE( "OccupancyCylinderTree: getPostEncroachmentTime 01", "[occupancycylinder]" ) 
{
    double s1 = -10; 
    double v1 = 1;
    double r1 = 1;
    double s2 = -11;
    double v2 = 1;
    double r2 = 1;

    adore::mad::OccupancyCylinderTree oct1;
    for(int i=0;i<20;i++)
    {
        oct1.insert(adore::mad::OccupancyCylinder(2.0, i-10,0,i,i+1));
    }
    oct1.setPreferredBranchingFactor(2);
    oct1.compute_all_levels();

    adore::mad::OccupancyCylinderTree oct2;
    for(int i=0;i<20;i++)
    {
        oct2.insert(adore::mad::OccupancyCylinder(2.0, 0,i-12,i,i+1));
    }
    oct2.setPreferredBranchingFactor(2);
    oct2.compute_all_levels();

    double result;
    bool rv = oct1.getPostEncroachmentTime(oct2,5.0,result);

    REQUIRE( rv == true );
    // REQUIRE( result==1.0 );
}

