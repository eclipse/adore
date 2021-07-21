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
 *   Daniel Heß - unit tests for borderbased/independentlanechangegeometry.h
 *********************************************************************************/

#include <adore/env/borderbased/independentlanechangegeometry.h>
#include <catch2/catch.hpp>


TEST_CASE( "independentlanechangegeometry::collect*Borders: minimal turnout left a)", "[independentlanechangegeometry]" ) 
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
    double pos = 5.0;
    auto ego_start = std::next(b.begin(),0);
    auto lf_start = std::next(b.begin(),0);
    auto lf_end = std::next(b.begin(),3);
    adore::env::BorderBased::IndependentLaneChangeGeometry lcg(true,&borderSet,&borderCostMap);
    lcg.lcb_.update(pos,ego_start,lf_start,lf_end);
    REQUIRE( lcg.collectSeparatingBorders() );
    REQUIRE( lcg.collectSourceOuterBorders() );
    REQUIRE( lcg.collectTargetOuterBorders() );
    REQUIRE( lcg.separatingBorders_.size()==3 );
    REQUIRE( lcg.separatingBorders_[0]==b[4] );
    REQUIRE( lcg.separatingBorders_[1]==b[5] );
    REQUIRE( lcg.separatingBorders_[2]==b[6] );
    REQUIRE( lcg.sourceOuterBorders_.size()==3 );
    REQUIRE( lcg.sourceOuterBorders_[0]==b[0] );
    REQUIRE( lcg.sourceOuterBorders_[1]==b[1] );
    REQUIRE( lcg.sourceOuterBorders_[2]==b[2] );
    REQUIRE( lcg.targetOuterBorders_.size()==3 );
    REQUIRE( lcg.targetOuterBorders_[0]==b[4] );
    REQUIRE( lcg.targetOuterBorders_[1]==b[8] );
    REQUIRE( lcg.targetOuterBorders_[2]==b[9] );
}

TEST_CASE( "independentlanechangegeometry::collect*Borders: exit right b)", "[independentlanechangegeometry]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    13|14|15|16|17|18|
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
    b.push_back( new Border( 0,0, 10,0, 0,4, 10,4 ) );//00
    b.push_back( new Border(10,0, 20,0, 10,4, 20,4 ) );//01
    b.push_back( new Border(20,0, 30,0, 20,4, 30,4 ) );//02
    b.push_back( new Border(30,0, 40,0, 30,4, 40,4 ) );//03
    b.push_back( new Border(40,0, 50,0, 40,4, 50,4 ) );//04
    b.push_back( new Border(50,0, 60,0, 50,4, 60,4 ) );//05
    b.push_back( new Border(10,-4, 20,-4, 10,0, 20,0 ) );//06
    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//07
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//08
    b.push_back( new Border(40,0, 50,-3 ) );//09
    b.push_back( new Border(50,-3, 60,-3 ) );//10
    b.push_back( new Border(40,-4, 50,-7, 40,0, 50,-3 ) );//11
    b.push_back( new Border(50,-7, 60,-7, 50,-3, 60,-3 ) );//12
    b.push_back( new Border(0,4, 10,4 ) );//13
    b.push_back( new Border(10,4, 20,4 ) );//14
    b.push_back( new Border(20,4, 30,4 ) );//15
    b.push_back( new Border(30,4, 40,4 ) );//16
    b.push_back( new Border(40,4, 50,4 ) );//17
    b.push_back( new Border(50,4, 60,4 ) );//18


    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[8]->m_id]=adore::env::NavigationCost(0.66);
    borderCostMap[b[11]->m_id]=adore::env::NavigationCost(0.33);
    borderCostMap[b[12]->m_id]=adore::env::NavigationCost(0);//placing goal 
    auto egoborder =  std::next(b.begin(),3);//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),5);//end of lane following borders
    double position_on_current = 5.0;
    adore::env::BorderBased::IndependentLaneChangeGeometry lcg(false,&borderSet,&borderCostMap);
    lcg.lcb_.update(position_on_current,egoborder,lfbbegin,lfbend);
    REQUIRE( lcg.lcb_.isValid() );
    REQUIRE( lcg.collectSeparatingBorders() );
    REQUIRE( lcg.collectSourceOuterBorders() );
    REQUIRE( lcg.collectTargetOuterBorders() );
    REQUIRE( lcg.separatingBorders_.size()==6 );
    REQUIRE( lcg.separatingBorders_[0]==b[0] );
    REQUIRE( lcg.separatingBorders_[1]==b[1] );
    REQUIRE( lcg.separatingBorders_[2]==b[2] );
    REQUIRE( lcg.separatingBorders_[3]==b[3] );
    REQUIRE( lcg.separatingBorders_[4]==b[9] );
    REQUIRE( lcg.separatingBorders_[5]==b[10] );
    REQUIRE( lcg.sourceOuterBorders_.size()==6 );
    REQUIRE( lcg.sourceOuterBorders_[0]==b[13] );
    REQUIRE( lcg.sourceOuterBorders_[1]==b[14] );
    REQUIRE( lcg.sourceOuterBorders_[2]==b[15] );
    REQUIRE( lcg.sourceOuterBorders_[3]==b[16] );
    REQUIRE( lcg.sourceOuterBorders_[4]==b[9] );
    REQUIRE( lcg.sourceOuterBorders_[5]==b[10] );
    REQUIRE( lcg.targetOuterBorders_.size()==6 );
    REQUIRE( lcg.targetOuterBorders_[0]==b[0] );
    REQUIRE( lcg.targetOuterBorders_[1]==b[6] );
    REQUIRE( lcg.targetOuterBorders_[2]==b[7] );
    REQUIRE( lcg.targetOuterBorders_[3]==b[8] );
    REQUIRE( lcg.targetOuterBorders_[4]==b[11] );
    REQUIRE( lcg.targetOuterBorders_[5]==b[12] );
}


TEST_CASE( "independentlanechangegeometry::update: exit right b)", "[independentlanechangegeometry]" ) 
{
    adore::env::BorderBased::BorderSet borderSet;
    adore::env::BorderBased::BorderCostMap borderCostMap;
    /**
     * Border layout:
     *    13|14|15|16|17|18|
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
    b.push_back( new Border( 0,0, 10,0, 0,4, 10,4 ) );//00
    b.push_back( new Border(10,0, 20,0, 10,4, 20,4 ) );//01
    b.push_back( new Border(20,0, 30,0, 20,4, 30,4 ) );//02
    b.push_back( new Border(30,0, 40,0, 30,4, 40,4 ) );//03
    b.push_back( new Border(40,0, 50,0, 40,4, 50,4 ) );//04
    b.push_back( new Border(50,0, 60,0, 50,4, 60,4 ) );//05
    b.push_back( new Border(10,-4, 20,-4, 10,0, 20,0 ) );//06
    b.push_back( new Border(20,-4, 30,-4, 20,0, 30,0 ) );//07
    b.push_back( new Border(30,-4, 40,-4, 30,0, 40,0 ) );//08
    b.push_back( new Border(40,0, 50,-3 ) );//09
    b.push_back( new Border(50,-3, 60,-3 ) );//10
    b.push_back( new Border(40,-4, 50,-7, 40,0, 50,-3 ) );//11
    b.push_back( new Border(50,-7, 60,-7, 50,-3, 60,-3 ) );//12
    b.push_back( new Border(0,4, 10,4 ) );//13
    b.push_back( new Border(10,4, 20,4 ) );//14
    b.push_back( new Border(20,4, 30,4 ) );//15
    b.push_back( new Border(30,4, 40,4 ) );//16
    b.push_back( new Border(40,4, 50,4 ) );//17
    b.push_back( new Border(50,4, 60,4 ) );//18


    
    for(unsigned int i=0;i<b.size();i++)
    {
        borderSet.insert_border(b[i]);
        borderCostMap.insert(std::make_pair(b[i]->m_id,adore::env::NavigationCost(1)));
    }
    borderCostMap[b[8]->m_id]=adore::env::NavigationCost(0.66);
    borderCostMap[b[11]->m_id]=adore::env::NavigationCost(0.33);
    borderCostMap[b[12]->m_id]=adore::env::NavigationCost(0);//placing goal 
    auto egoborder =  std::next(b.begin(),3);//search start for best gate entry
    auto lfbbegin = b.begin();//beginning of lane following borders
    auto lfbend = std::next(b.begin(),5);//end of lane following borders
    double position_on_current = 5.0;
    adore::env::BorderBased::IndependentLaneChangeGeometry lcg(false,&borderSet,&borderCostMap);
    lcg.update(position_on_current,egoborder,lfbbegin,lfbend);
    REQUIRE( lcg.lcb_.isValid() );
}