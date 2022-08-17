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
 *   Daniel Heß - unit tests for ocroadbasedprediction.h
 *********************************************************************************/
#include <catch2/catch.hpp>
#include <adore/env/traffic/ocroadbasedprediction.h>
#include <adore/if_xodr/xodr2borderbased.h>
#include <string>
// #include <plotlablib/afigurestub.h>
// #include <plotlablib/plcommands.h>
// #include <plotlablib/figurestubfactory.h>
// #include <adore/apps/if_plotlab/plot.h>
// #include <adore/apps/if_plotlab/fancy_config.h>
#include <filesystem>
#include <unistd.h>



TEST_CASE( "OCRoadBasedPrediction: predict on circle 50m", "[ocroadbasedprediction]" ) 
{
    //load borders
    adore::env::BorderBased::BorderSet borderSet;
    adore::if_xodr::XODR2BorderBasedConverter converter;
    std::string xodrFilename = "../../../../adore/env/test/circle50m.xodr";
    try{converter.convert(xodrFilename.c_str(),&borderSet,false);}catch(...){}
    if(borderSet.size()==0)
    {
        std::cout<<"Test fail probably due to unknown xodr file"<<std::endl;
        std::filesystem::path cwd = std::filesystem::current_path();
        std::cout<<"cwd: "<<cwd<<std::endl;
        std::cout<<"desired file: "<<xodrFilename<<std::endl;
    }
    CHECK(borderSet.size()>0);
    
    //generate traffic participants and match them to borders
    adore::env::traffic::TrafficMap trafficMap(&borderSet,nullptr);
    adore::env::traffic::Participant p1;
    p1.trackingID_ = 1;
    p1.center_(0) = 0.0;
    p1.center_(1) = -52.0;
    p1.center_(2) = 0.0;
    p1.yaw_ = 0.0;
    p1.length_ = 5.0;
    p1.width_ = 2.0;
    p1.vx_ = 15.0;
    p1.vy_ = 0.0;
    p1.yawrate_ = 0.0;
    trafficMap.getTrafficParticipantSet().push_back(p1);    
    trafficMap.matchBorders();

    adore::env::OCRoadBasedPrediction rbp(&trafficMap);
    rbp.setVMax(15.0);
    rbp.setTMaxUTC(20.0);
    rbp.setAMax(1.0);
    rbp.setAMin(-2.0);
    rbp.setAngleErrorMax(M_PI*0.25);
    adore::env::OccupancyCylinderPredictionSet set;
    rbp.predict(p1,set);
    REQUIRE( set.size()>0 );

    // DLR_TS::PlotLab::FigureStubFactory fig_factory;
    // auto figure = fig_factory.createFigureStub(2);
    // figure->show();
    // int count = 0;
    // std::stringstream ss;
    // ss << "testpredictions";
    // adore::PLOT::FancyBirdConfig::prediction_config config;
    // config.r_.start_ = 1.0;
    // config.r_.end_ = 1.0;
    // config.g_.start_ = 1.0;
    // adore::PLOT::plotPredictionSet(set,0,config,ss.str(),figure);

    // std::cout<<"asdf";
}

TEST_CASE( "OCRoadBasedPrediction: predict 3 branches at junction", "[ocroadbasedprediction]" ) 
{
    //load borders
    adore::env::BorderBased::BorderSet borderSet;
    adore::if_xodr::XODR2BorderBasedConverter converter;
    std::string xodrFilename = "../../../../adore/env/test/basic_test_track.xodr";
    try{converter.convert(xodrFilename.c_str(),&borderSet,false);}catch(...){}
    if(borderSet.size()==0)
    {
        std::cout<<"Test fail probably due to unknown xodr file"<<std::endl;
        std::filesystem::path cwd = std::filesystem::current_path();
        std::cout<<"cwd: "<<cwd<<std::endl;
        std::cout<<"desired file: "<<xodrFilename<<std::endl;
    }
    CHECK(borderSet.size()>0);
    
    //generate traffic participants and match them to borders
    adore::env::traffic::TrafficMap trafficMap(&borderSet,nullptr);
    adore::env::traffic::Participant p1;
    p1.trackingID_ = 1;
    p1.center_(0) = 100.0;
    p1.center_(1) = 162.0;
    p1.center_(2) = 0.0;
    p1.yaw_ = 0.0;
    p1.length_ = 5.0;
    p1.width_ = 2.0;
    p1.vx_ = 15.0;
    p1.vy_ = 0.0;
    p1.yawrate_ = 0.0;
    trafficMap.getTrafficParticipantSet().push_back(p1);    
    trafficMap.matchBorders();

    adore::env::OCRoadBasedPrediction rbp(&trafficMap);
    rbp.setVMax(15.0);
    rbp.setTMaxUTC(5.0);
    rbp.setAMax(1.0);
    rbp.setAMin(-2.0);
    rbp.setAngleErrorMax(M_PI*0.25);
    adore::env::OccupancyCylinderPredictionSet set;
    rbp.predict(p1,set);
    REQUIRE( set.size()>0 );

    // DLR_TS::PlotLab::FigureStubFactory fig_factory;
    // auto figure = fig_factory.createFigureStub(2);
    // figure->show();
    // int count = 0;
    // std::stringstream ss;
    // ss << "testpredictions";
    // adore::PLOT::FancyBirdConfig::prediction_config config;
    // config.r_.start_ = 1.0;
    // config.r_.end_ = 1.0;
    // config.g_.start_ = 1.0;
    // adore::PLOT::plotPredictionSet(set,0,config,ss.str(),figure);
    // usleep(2e6);

    //test different positions, which should be covered by prediction
    {
        adore::mad::OccupancyCylinder oc(0.1,140,162,0,5.0);
        adore::mad::OccupancyCylinderTree test_tree;
        test_tree.insert(oc);
        bool in_collision = false;
        for(auto& prediction:set)
        {
            in_collision = in_collision || prediction.occupancy_.collidesWith(test_tree);
        }
        REQUIRE( in_collision );
    }

    {
        adore::mad::OccupancyCylinder oc(0.1,138,180,0,5.0);
        adore::mad::OccupancyCylinderTree test_tree;
        test_tree.insert(oc);
        bool in_collision = false;
        for(auto& prediction:set)
        {
            in_collision = in_collision || prediction.occupancy_.collidesWith(test_tree);
        }
        // REQUIRE ( std::filesystem::current_path() == "");
        REQUIRE( in_collision );
    }

    {
        adore::mad::OccupancyCylinder oc(0.1,135,150,0,5.0);
        adore::mad::OccupancyCylinderTree test_tree;
        test_tree.insert(oc);
        bool in_collision = false;
        for(auto& prediction:set)
        {
            in_collision = in_collision || prediction.occupancy_.collidesWith(test_tree);
        }
        REQUIRE( in_collision );
    }

}

