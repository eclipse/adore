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
 *   Daniel Heß - unit tests for borderbased/lanechangeborders.h
 *********************************************************************************/

#include <adore/env/borderbased/lanechangeborders.h>
#include <catch2/catch.hpp>
#include <vector>

TEST_CASE( "lanechangeborders::findBestGateEntryPoint: minimal turnout left a)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *       08|09
     *      / *   \
     *    04|05|06|07
     *    ->
     *    00|01|02|03
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0,  0,4, 10,4) );
    b.push_back( new Border(10,0, 20,0, 10,4, 20,4) );
    b.push_back( new Border(20,0, 30,0, 20,4, 30,4) );
    b.push_back( new Border(30,0, 40,0, 30,4, 40,4) );
    b.push_back( new Border( 0,4, 10,4) );
    b.push_back( new Border(10,4, 20,4, 10,4, 20,8) );
    b.push_back( new Border(20,4, 30,4, 20,8, 30,4) );
    b.push_back( new Border(30,4, 40,4) );
    b.push_back( new Border(10,4, 20,8) );
    b.push_back( new Border(20,8, 30,4) );
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[5]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(true,&borderSet,&borderCostMap);
    auto result = lcb.findBestGateEntryPoint(b.begin(),std::next(b.begin(),3));//03 is last index of lfb
    // std::cout<<" result.second: "<<result.second->m_id.toString();
    // std::cout<<" result.first: "<<(*result.first)->m_id.toString();
    // REQUIRE_THROWS_WITH( result.second!=nullptr, "gate entry border is 0");
    // REQUIRE_THROWS_WITH( b[1]==*result.first, "b[1]: "+b[1]->m_id.toString()+" unequals *result.first: "+ (*result.first)->m_id.toString() );
    // REQUIRE_THROWS_WITH( b[5]==result.second, "b[5]: "+b[5]->m_id.toString()+" unequals result.second: "+ (result.second)->m_id.toString() );
    REQUIRE( result.second!=nullptr );
    REQUIRE( b[1]==*result.first );
    REQUIRE( b[5]==result.second );
    
}
TEST_CASE( "lanechangeborders::findBestGateEntryPoint: minimal turnout left b)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *       08|09
     *      /   * \
     *    04|05|06|07
     *    ->
     *    00|01|02|03
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0,  0,4, 10,4) );
    b.push_back( new Border(10,0, 20,0, 10,4, 20,4) );
    b.push_back( new Border(20,0, 30,0, 20,4, 30,4) );
    b.push_back( new Border(30,0, 40,0, 30,4, 40,4) );
    b.push_back( new Border( 0,4, 10,4) );
    b.push_back( new Border(10,4, 20,4, 10,4, 20,8) );
    b.push_back( new Border(20,4, 30,4, 20,8, 30,4) );
    b.push_back( new Border(30,4, 40,4) );
    b.push_back( new Border(10,4, 20,8) );
    b.push_back( new Border(20,8, 30,4) );
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[6]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(true,&borderSet,&borderCostMap);
    auto result = lcb.findBestGateEntryPoint(b.begin(),std::next(b.begin(),3));//03 is last index of lfb
    REQUIRE( result.second!=nullptr );
    REQUIRE( b[2]==*result.first );
    REQUIRE( b[6]==result.second );
}
TEST_CASE( "lanechangeborders::getGateRegion: minimal turnout left a)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *       08|09
     *      / *   \
     *    04|05|06|07
     *    ->
     *    00|01|02|03
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0,  0,4, 10,4) );
    b.push_back( new Border(10,0, 20,0, 10,4, 20,4) );
    b.push_back( new Border(20,0, 30,0, 20,4, 30,4) );
    b.push_back( new Border(30,0, 40,0, 30,4, 40,4) );
    b.push_back( new Border( 0,4, 10,4) );
    b.push_back( new Border(10,4, 20,4, 10,4, 20,8) );
    b.push_back( new Border(20,4, 30,4, 20,8, 30,4) );
    b.push_back( new Border(30,4, 40,4) );
    b.push_back( new Border(10,4, 20,8) );
    b.push_back( new Border(20,8, 30,4) );
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[5]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(true,&borderSet,&borderCostMap);
    auto egoborder =  b.begin();//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),3);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    REQUIRE( result.first!=b.end() );
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( b[1]==*result.first );
    REQUIRE( b[2]==*result.second );
}
TEST_CASE( "lanechangeborders::getGateRegion: minimal turnout left b)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *       08|09
     *      /   * \
     *    04|05|06|07
     *    ->
     *    00|01|02|03
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0,  0,4, 10,4) );
    b.push_back( new Border(10,0, 20,0, 10,4, 20,4) );
    b.push_back( new Border(20,0, 30,0, 20,4, 30,4) );
    b.push_back( new Border(30,0, 40,0, 30,4, 40,4) );
    b.push_back( new Border( 0,4, 10,4) );
    b.push_back( new Border(10,4, 20,4, 10,4, 20,8) );
    b.push_back( new Border(20,4, 30,4, 20,8, 30,4) );
    b.push_back( new Border(30,4, 40,4) );
    b.push_back( new Border(10,4, 20,8) );
    b.push_back( new Border(20,8, 30,4) );
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[6]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(true,&borderSet,&borderCostMap);
    auto egoborder =  b.begin();//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),3);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    REQUIRE( result.first!=b.end() );
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( b[1]==*result.first );
    REQUIRE( b[2]==*result.second );
}
TEST_CASE( "lanechangeborders::getGateRegion: minimal turnout left c)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *       08|09
     *      /   * \
     *    04|05|06|07
     *          ->
     *    00|01|02|03
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0,  0,4, 10,4) );
    b.push_back( new Border(10,0, 20,0, 10,4, 20,4) );
    b.push_back( new Border(20,0, 30,0, 20,4, 30,4) );
    b.push_back( new Border(30,0, 40,0, 30,4, 40,4) );
    b.push_back( new Border( 0,4, 10,4) );
    b.push_back( new Border(10,4, 20,4, 10,4, 20,8) );
    b.push_back( new Border(20,4, 30,4, 20,8, 30,4) );
    b.push_back( new Border(30,4, 40,4) );
    b.push_back( new Border(10,4, 20,8) );
    b.push_back( new Border(20,8, 30,4) );
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[6]->m_id]=adore::env::NavigationCost(0);//placing goal
    adore::env::BorderBased::LaneChangeBorders lcb(true,&borderSet,&borderCostMap);
    auto egoborder =  std::next(b.begin(),2);//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),3);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    REQUIRE( result.first!=b.end() );
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( b[1]==*result.first );
    REQUIRE( b[2]==*result.second );
}

TEST_CASE( "lanechangeborders::getGateRegion: minimal turnout right a)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *       08|09
     *      /    \
     *    04|05|06|07
     *    ->
     *    00|01|02|03
     *      |   * |
     *       10|11
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0,  0,4, 10,4) );
    b.push_back( new Border(10,0, 20,0, 10,4, 20,4) );
    b.push_back( new Border(20,0, 30,0, 20,4, 30,4) );
    b.push_back( new Border(30,0, 40,0, 30,4, 40,4) );
    b.push_back( new Border( 0,4, 10,4) );
    b.push_back( new Border(10,4, 20,4, 10,4, 20,8) );
    b.push_back( new Border(20,4, 30,4, 20,8, 30,4) );
    b.push_back( new Border(30,4, 40,4) );
    b.push_back( new Border(10,4, 20,8) );
    b.push_back( new Border(20,8, 30,4) );
    b.push_back( new Border(10,-4, 20,-4, 10,0, 20,0) );
    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0) );
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[11]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(false,&borderSet,&borderCostMap);
    auto egoborder =  b.begin();//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),3);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    REQUIRE( result.first!=b.end() );
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( b[1]==*result.first );
    REQUIRE( b[2]==*result.second );
}

TEST_CASE( "lanechangeborders::getGateRegion: exit right a)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    ?????????????????? (irrelevant)
     *    ->
     *    00|01|02|03|04|05|
     *      |       * \
     *       06|07|08  09|10|
     *               \    
     *                |11|12|
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0 ) );//00
    b.push_back( new Border(10,0, 20,0 ) );//01
    b.push_back( new Border(20,0, 30,0 ) );//02
    b.push_back( new Border(30,0, 40,0 ) );//03
    b.push_back( new Border(40,0, 50,0 ) );//04
    b.push_back( new Border(50,0, 60,0 ) );//05
    b.push_back( new Border(10,-4, 20,-4, 10,0, 20,0 ) );//06
    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//07
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//08
    b.push_back( new Border(40,0, 50,-3 ) );//09
    b.push_back( new Border(50,-3, 60,-3 ) );//10
    b.push_back( new Border(40,-4, 50,-7, 40,0, 50,-3 ) );//11
    b.push_back( new Border(50,-7, 60,-7, 50,-3, 60,-3 ) );//12

    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[8]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(false,&borderSet,&borderCostMap);
    auto egoborder =  b.begin();//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),5);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    REQUIRE( result.first!=b.end() );
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( b[1]==*result.first );
    REQUIRE( b[3]==*result.second );
}


TEST_CASE( "lanechangeborders::getGateRegion: exit right b)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    ?????????????????? (irrelevant)
     *             ->
     *    00|01|02|03|04|05|
     *      |       * \
     *       06|07|08  09|10|
     *               \    
     *                |11|12|
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0 ) );//00
    b.push_back( new Border(10,0, 20,0 ) );//01
    b.push_back( new Border(20,0, 30,0 ) );//02
    b.push_back( new Border(30,0, 40,0 ) );//03
    b.push_back( new Border(40,0, 50,0 ) );//04
    b.push_back( new Border(50,0, 60,0 ) );//05
    b.push_back( new Border(10,-4, 20,-4, 10,0, 20,0 ) );//06
    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//07
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//08
    b.push_back( new Border(40,0, 50,-3 ) );//09
    b.push_back( new Border(50,-3, 60,-3 ) );//10
    b.push_back( new Border(40,-4, 50,-7, 40,0, 50,-3 ) );//11
    b.push_back( new Border(50,-7, 60,-7, 50,-3, 60,-3 ) );//12

    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[8]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(false,&borderSet,&borderCostMap);
    auto egoborder =  std::next(b.begin(),3);//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),5);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    REQUIRE( result.first!=b.end() );
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( b[1]==*result.first );
    REQUIRE( b[3]==*result.second );
}

TEST_CASE( "lanechangeborders::getGateRegion: exit right c)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    ?????????????????? (irrelevant)
     *                ->
     *    00|01|02|03|04|05|
     *      |       * \
     *       06|07|08  09|10|
     *               \    
     *                |11|12|
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0 ) );//00
    b.push_back( new Border(10,0, 20,0 ) );//01
    b.push_back( new Border(20,0, 30,0 ) );//02
    b.push_back( new Border(30,0, 40,0 ) );//03
    b.push_back( new Border(40,0, 50,0 ) );//04
    b.push_back( new Border(50,0, 60,0 ) );//05
    b.push_back( new Border(10,-4, 20,-4, 10,0, 20,0 ) );//06
    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//07
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//08
    b.push_back( new Border(40,0, 50,-3 ) );//09
    b.push_back( new Border(50,-3, 60,-3 ) );//10
    b.push_back( new Border(40,-4, 50,-7, 40,0, 50,-3 ) );//11
    b.push_back( new Border(50,-7, 60,-7, 50,-3, 60,-3 ) );//12

    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[8]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(false,&borderSet,&borderCostMap);
    auto egoborder =  std::next(b.begin(),4);//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),5);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( std::next(lfbend)==result.first );
    REQUIRE( std::next(lfbend)==result.second );
}

TEST_CASE( "lanechangeborders::getGateRegion: exit right d)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    ?????????????????? (irrelevant)
     *                ->
     *    07|08|09|10|11|12|13|
     *      |       * \
     *       00|01|02  03|04|
     *               \    
     *                |05|06|
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;

    b.push_back( new Border(10,-4, 20,-4, 10,0, 20,0 ) );//00
    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//01
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//02
    b.push_back( new Border(40,0, 50,-3 ) );//03
    b.push_back( new Border(50,-3, 60,-3 ) );//04
    b.push_back( new Border(40,-4, 50,-7, 40,0, 50,-3 ) );//05
    b.push_back( new Border(50,-7, 60,-7, 50,-3, 60,-3 ) );//06

    b.push_back( new Border( 0,0, 10,0 ) );//07
    b.push_back( new Border(10,0, 20,0 ) );//08
    b.push_back( new Border(20,0, 30,0 ) );//09
    b.push_back( new Border(30,0, 40,0 ) );//10
    b.push_back( new Border(40,0, 50,0 ) );//11
    b.push_back( new Border(50,0, 60,0 ) );//12
    b.push_back( new Border(60,0, 70,0 ) );//13

    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[2]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(false,&borderSet,&borderCostMap);
    auto egoborder =  std::next(b.begin(),11);//search start for best gate entry
    auto lfbbegin = std::next(b.begin(),07);//beginning of lane following borders
    auto lfbend = std::next(b.begin(),13);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( std::next(lfbend)==result.first );
    REQUIRE( std::next(lfbend)==result.second );
}


TEST_CASE( "lanechangeborders::getGateRegion: multi turnout right a)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    ?????????????????? (irrelevant)
     *    ->          
     *    00|01|02|03|04|05|06|07|08|09|10
     *         \      * /        \
     *          11|12|13          14|15|16
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0 ) );//00
    b.push_back( new Border(10,0, 20,0 ) );//01
    b.push_back( new Border(20,0, 30,0 ) );//02
    b.push_back( new Border(30,0, 40,0 ) );//03
    b.push_back( new Border(40,0, 50,0 ) );//04
    b.push_back( new Border(50,0, 60,0 ) );//05
    b.push_back( new Border(60,0, 70,0 ) );//06
    b.push_back( new Border(70,0, 80,0 ) );//07
    b.push_back( new Border(80,0, 90,0 ) );//08
    b.push_back( new Border(90,0, 100,0 ) );//09
    b.push_back( new Border(100,0, 110,0 ) );//10

    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//11
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//12
    b.push_back( new Border(40,-4, 50,-4, 40,0, 50,0 ) );//13

    b.push_back( new Border(80,-4, 90,-4, 80,0, 90,0 ) );//14
    b.push_back( new Border(90,-4, 100,-4, 90,0, 100,0 ) );//15
    b.push_back( new Border(100,-4, 110,-4, 100,0, 110,0 ) );//16

    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[13]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(false,&borderSet,&borderCostMap);
    auto egoborder =  std::next(b.begin(),0);//search start for best gate entry
    auto lfbbegin = std::next(b.begin(),0);//beginning of lane following borders
    auto lfbend = std::next(b.begin(),10);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    REQUIRE( result.first!=b.end() );
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( b[2]==*result.first );
    REQUIRE( b[4]==*result.second );
}

TEST_CASE( "lanechangeborders::getGateRegion: multi turnout right b)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    ?????????????????? (irrelevant)
     *    ->          
     *    00|01|02|03|04|05|06|07|08|09|10
     *         \        /        \    *
     *          11|12|13          14|15|16
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0 ) );//00
    b.push_back( new Border(10,0, 20,0 ) );//01
    b.push_back( new Border(20,0, 30,0 ) );//02
    b.push_back( new Border(30,0, 40,0 ) );//03
    b.push_back( new Border(40,0, 50,0 ) );//04
    b.push_back( new Border(50,0, 60,0 ) );//05
    b.push_back( new Border(60,0, 70,0 ) );//06
    b.push_back( new Border(70,0, 80,0 ) );//07
    b.push_back( new Border(80,0, 90,0 ) );//08
    b.push_back( new Border(90,0, 100,0 ) );//09
    b.push_back( new Border(100,0, 110,0 ) );//10

    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//11
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//12
    b.push_back( new Border(40,-4, 50,-4, 40,0, 50,0 ) );//13

    b.push_back( new Border(80,-4, 90,-4, 80,0, 90,0 ) );//14
    b.push_back( new Border(90,-4, 100,-4, 90,0, 100,0 ) );//15
    b.push_back( new Border(100,-4, 110,-4, 100,0, 110,0 ) );//16

    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[15]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(false,&borderSet,&borderCostMap);
    auto egoborder =  std::next(b.begin(),0);//search start for best gate entry
    auto lfbbegin = std::next(b.begin(),0);//beginning of lane following borders
    auto lfbend = std::next(b.begin(),10);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    REQUIRE( result.first!=b.end() );
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( b[8]==*result.first );
    REQUIRE( b[10]==*result.second );
}

TEST_CASE( "lanechangeborders::getGateRegion: multi turnout right c)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    ?????????????????? (irrelevant)
     *                   ->          
     *    00|01|02|03|04|05|06|07|08|09|10
     *         \      * /        \     
     *          11|12|13          14|15|16
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0 ) );//00
    b.push_back( new Border(10,0, 20,0 ) );//01
    b.push_back( new Border(20,0, 30,0 ) );//02
    b.push_back( new Border(30,0, 40,0 ) );//03
    b.push_back( new Border(40,0, 50,0 ) );//04
    b.push_back( new Border(50,0, 60,0 ) );//05
    b.push_back( new Border(60,0, 70,0 ) );//06
    b.push_back( new Border(70,0, 80,0 ) );//07
    b.push_back( new Border(80,0, 90,0 ) );//08
    b.push_back( new Border(90,0, 100,0 ) );//09
    b.push_back( new Border(100,0, 110,0 ) );//10

    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//11
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//12
    b.push_back( new Border(40,-4, 50,-4, 40,0, 50,0 ) );//13

    b.push_back( new Border(80,-4, 90,-4, 80,0, 90,0 ) );//14
    b.push_back( new Border(90,-4, 100,-4, 90,0, 100,0 ) );//15
    b.push_back( new Border(100,-4, 110,-4, 100,0, 110,0 ) );//16

    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[13]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(false,&borderSet,&borderCostMap);
    auto egoborder =  std::next(b.begin(),5);//search start for best gate entry
    auto lfbbegin = std::next(b.begin(),0);//beginning of lane following borders
    auto lfbend = std::next(b.begin(),10);//end of lane following borders
    auto result = lcb.getGateRegion(egoborder,lfbbegin,lfbend);
    REQUIRE( result.first!=b.end() );
    // std::cout<<" gate start: "<<(*result.first)->m_id.toString();
    // std::cout<<" gate end: "<<(*result.second)->m_id.toString();
    REQUIRE( b[8]==*result.first );
    REQUIRE( b[10]==*result.second );
}



TEST_CASE( "lanechangeborders::update: exit right b)", "[lanechangeborders]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    ?????????????????? (irrelevant)
     *             ->
     *    00|01|02|03|04|05|
     *      |       * \
     *       06|07|08  09|10|
     *               \    
     *                |11|12|
     */
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;
    typedef adore::env::BorderBased::Coordinate Coordinate;
    
    std::vector<Border*> b;
    b.push_back( new Border( 0,0, 10,0 ) );//00
    b.push_back( new Border(10,0, 20,0 ) );//01
    b.push_back( new Border(20,0, 30,0 ) );//02
    b.push_back( new Border(30,0, 40,0 ) );//03
    b.push_back( new Border(40,0, 50,0 ) );//04
    b.push_back( new Border(50,0, 60,0 ) );//05
    b.push_back( new Border(10,-4, 20,-4, 10,0, 20,0 ) );//06
    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//07
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//08
    b.push_back( new Border(40,0, 50,-3 ) );//09
    b.push_back( new Border(50,-3, 60,-3 ) );//10
    b.push_back( new Border(40,-4, 50,-7, 40,0, 50,-3 ) );//11
    b.push_back( new Border(50,-7, 60,-7, 50,-3, 60,-3 ) );//12

    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[8]->m_id]=adore::env::NavigationCost(0.66);
    borderCostMap[b[11]->m_id]=adore::env::NavigationCost(0.33);
    borderCostMap[b[12]->m_id]=adore::env::NavigationCost(0);//placing goal 
    adore::env::BorderBased::LaneChangeBorders lcb(false,&borderSet,&borderCostMap);
    auto egoborder =  std::next(b.begin(),3);//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),5);//end of lane following borders
    double position_on_current = 5.0;
    lcb.update(position_on_current,egoborder,lfbbegin,lfbend);
    REQUIRE(lcb.getProgressInGate()== Approx(25.0) );
    REQUIRE(lcb.getRemainingInGate()== Approx(5.0) );
    REQUIRE(lcb.upstream_borders_.size()==1);
    REQUIRE(lcb.upstream_borders_[0]==b[0]);
    REQUIRE(lcb.gate_source_borders_.size()==3);
    REQUIRE(lcb.gate_source_borders_[0]==b[1]);
    REQUIRE(lcb.gate_source_borders_[1]==b[2]);
    REQUIRE(lcb.gate_source_borders_[2]==b[3]);
    REQUIRE(lcb.gate_target_borders_.size()==3);
    REQUIRE(lcb.gate_target_borders_[0]==b[6]);
    REQUIRE(lcb.gate_target_borders_[1]==b[7]);
    REQUIRE(lcb.gate_target_borders_[2]==b[8]);
    REQUIRE(lcb.downstream_borders_.size()==2);
    REQUIRE(lcb.downstream_borders_[0]==b[11]);
    REQUIRE(lcb.downstream_borders_[1]==b[12]);
}