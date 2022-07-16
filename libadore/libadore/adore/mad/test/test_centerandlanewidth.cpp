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
 *   Daniel Heß - tests for centerandlanewidth.h
 ********************************************************************************/

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "adore/mad/adoremath.h"
#include "adore/mad/arraymatrixtools.h"
#include "adore/mad/centerandlanewidth.h"
#include <iostream>
#include <fstream>


TEST_CASE( "testing centerandlanewidth with N=1 and M=2", "[centerandlanewidth]" ) 
{
    static const int N=1;
    static const int M=2;
    adoreMatrix<double, 4, N> left;
    adoreMatrix<double, 4, M> right;
    adoreMatrix<double, 0, 0> center;
    center.set_size(5, N+M-1);
    left(0,0) = 0.0; 
    left(1,0) = 0.0;
    left(2,0) = 1.0;
    left(3,0) = 0.0;
    right(0,0) = 0.0;   right(0,1) = 2.0;
    right(1,0) = 0.0;   right(1,1) = 2.0;
    right(2,0) = -1.0;  right(2,1) = -1.0;
    right(3,0) = 0.0;   right(3,0) = 0.0;
    int K = adore::mad::computeLaneWidthAndCenter(left,right,center);
    REQUIRE(K==2);
    REQUIRE(std::abs(center(1,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(2,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(4,0)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(1,1)-1.0)<1.0e-10);
    REQUIRE(std::abs(center(2,1)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(4,0)-std::sqrt(4.0))<1.0e-10);
}

TEST_CASE( "testing centerandlanewidth with N=2 and M=1", "[centerandlanewidth]" ) 
{
}

TEST_CASE( "testing centerandlanewidth with symmetric lane segment w/ N=2, M=2", "[centerandlanewidth]" ) 
{
    static const int N=2;
    static const int M=2;
    adoreMatrix<double, 4, N> left;
    adoreMatrix<double, 4, M> right;
    adoreMatrix<double, 0, 0> center;
    center.set_size(5, N+M-1);
    left(0,0) = 0.0;    left(0,1) = 2.0;   
    left(1,0) = 0.0;    left(1,1) = 2.0;  
    left(2,0) = 1.0;    left(2,1) = 1.0;    
    left(3,0) = 0.0;    left(3,1) = 0.0;   
    right(0,0) = 0.0;   right(0,1) = 2.0;
    right(1,0) = 0.0;   right(1,1) = 2.0;
    right(2,0) = -1.0;  right(2,1) = -1.0;
    right(3,0) = 0.0;   right(3,0) = 0.0;
    int K = adore::mad::computeLaneWidthAndCenter(left,right,center);
    {
        std::cout<<"K="<<K<<std::endl;
        for(int i=0;i<K;i++)
        {
            std::cout<<center(0,i)<<";"<<center(1,i)<<";"<<center(2,i)<<";"<<center(3,i)<<";"<<center(4,i)<<std::endl;
        }
    }
    REQUIRE(K==2);
    REQUIRE(std::abs(center(0,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(1,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(2,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(3,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(4,0)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(0,1)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(1,1)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(2,1)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(3,1)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(4,1)-2.0)<1.0e-10);
}


TEST_CASE( "testing centerandlanewidth misaligned rear w/ N==3, M==3", "[centerandlanewidth]" ) 
{
    static const int N=3;
    static const int M=3;
    adoreMatrix<double, 4, N> left;
    adoreMatrix<double, 4, M> right;
    adoreMatrix<double, 0, 0> center;
    center.set_size(5, 10);
    left(0,0) = 0.0;    left(0,1) = 2.0;    left(0,2) = 3.0;
    left(1,0) = 0.0;    left(1,1) = 2.0;    left(1,2) = 3.0;
    left(2,0) = 1.0;    left(2,1) = 1.0;    left(2,2) = 1.0;
    left(3,0) = 0.0;    left(3,1) = 0.0;    left(3,2) = 0.0;
    right(0,0) = 0.0;   right(0,1) = 1.0;   right(0,2) = 4.0;
    right(1,0) = 0.0;   right(1,1) = 1.0;   right(1,2) = 4.0;
    right(2,0) = -1.0;  right(2,1) = -1.0;  right(2,2) = -1.0;
    right(3,0) = 0.0;   right(3,1) = 0.0;   right(3,2) = 0.0;
    int K = adore::mad::computeLaneWidthAndCenter(left,right,center);
    {
        std::cout<<"K="<<K<<std::endl;
        for(int i=0;i<K;i++)
        {
            std::cout<<center(0,i)<<";"<<center(1,i)<<";"<<center(2,i)<<";"<<center(3,i)<<";"<<center(4,i)<<std::endl;
        }
    }
    REQUIRE(K==5);
    REQUIRE(std::abs(center(0,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(1,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(2,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(3,0)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(4,0)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(0,1)-1.0)<1.0e-10);
    REQUIRE(std::abs(center(1,1)-1.0)<1.0e-10);
    REQUIRE(std::abs(center(2,1)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(3,1)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(4,1)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(0,2)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(1,2)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(2,2)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(3,2)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(4,2)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(0,3)-3.0)<1.0e-10);
    REQUIRE(std::abs(center(1,3)-3.0)<1.0e-10);
    REQUIRE(std::abs(center(2,3)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(3,3)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(4,3)-2.0)<1.0e-10);
    REQUIRE(std::abs(center(0,4)-3.5)<1.0e-10);
    REQUIRE(std::abs(center(1,4)-3.5)<1.0e-10);
    REQUIRE(std::abs(center(2,4)-0.0)<1.0e-10);
    REQUIRE(std::abs(center(3,4)-0.0)<1.0e-10);
    // REQUIRE(std::abs(center(4,4)-2.0)<1.0e-10);
}

TEST_CASE( "testing centerandlanewidth with u shape for right N==2, M==4", "[centerandlanewidth]" ) 
{
    static const int N=2;
    static const int M=4;
    adoreMatrix<double, 4, N> left;
    adoreMatrix<double, 4, M> right;
    adoreMatrix<double, 0, 0> center;
    center.set_size(5,std::max(N,M)*2);
    left(0,0) = 0.0;    left(0,1) = 2.0;    
    left(1,0) = 0.0;    left(1,1) = 2.0;    
    left(2,0) = 0.0;    left(2,1) = 0.0;    
    left(3,0) = 0.0;    left(3,1) = 0.0;    
    right(0,0) = 0.0;   right(0,1) = 3.0;   right(0,2) = 5.0;  right(0,3) = 8.0;
    right(1,0) = 0.0;   right(1,1) = 3.0;   right(1,2) = 3.0;  right(1,3) = 0.0;
    right(2,0) =-2.0;   right(2,1) = -2.0;  right(2,2) = 2.0; right(2,3) = 2.0;
    right(3,0) = 0.0;   right(3,1) = 0.0;   right(3,2) = 0.0;  right(3,3) = 4.0;
    int K = adore::mad::computeLaneWidthAndCenter(left,right,center);
    {
        std::cout<<"K="<<K<<std::endl;
        for(int i=0;i<K;i++)
        {
            std::cout<<center(0,i)<<";"<<center(1,i)<<";"<<center(2,i)<<";"<<center(3,i)<<";"<<center(4,i)<<std::endl;
        }
    }
    REQUIRE(K==7);
}

TEST_CASE("testing centerandlanewidth with small real-world example, N=6, M=13")
{
    static const int N=6;
    static const int M=13;
    adoreMatrix<double, 4, N> left;
    adoreMatrix<double, 4, M> right;
    adoreMatrix<double, 0, 0> center;
    center.set_size(5, N+M-1);
    left(0,0)=0;left(1,0)=0;left(2,0)=0;left(3,0)=0;
    left(0,1)=4.31893;left(1,1)=-0.06701;left(2,1)=4.31841;left(3,1)=0;
    left(0,2)=5.07852;left(1,2)=-0.081901;left(2,2)=5.07786;left(3,2)=0;
    left(0,3)=6.26246;left(1,3)=-0.096792;left(2,3)=6.2617;left(3,3)=0;
    left(0,4)=8.30259;left(1,4)=-0.111683;left(2,4)=8.30177;left(3,4)=0;
    left(0,5)=8.5446;left(1,5)=-0.115406;left(2,5)=8.54375;left(3,5)=0;
    right(0,0)=0;right(1,0)=3.96835;right(2,0)=0.061578;right(3,0)=0;
    right(0,1)=1.53943;right(1,1)=3.99406;right(2,1)=1.60079;right(3,1)=0;
    right(0,2)=2.50746;right(1,2)=4.00896;right(2,2)=2.56871;right(3,2)=0;
    right(0,3)=3.43263;right(1,3)=4.06852;right(2,3)=3.49196;right(3,3)=0;
    right(0,4)=4.116;right(1,4)=4.20254;right(2,4)=4.16205;right(3,4)=0;
    right(0,5)=4.74107;right(1,5)=4.33656;right(2,5)=4.77259;right(3,5)=0;
    right(0,6)=5.45986;right(1,6)=4.50036;right(2,6)=5.47247;right(3,6)=0;
    right(0,7)=6.09918;right(1,7)=4.66416;right(2,7)=6.09045;right(3,7)=0;
    right(0,8)=6.93315;right(1,8)=4.95454;right(2,8)=6.87223;right(3,8)=0;
    right(0,9)=7.53051;right(1,9)=5.18535;right(2,9)=7.4232;right(3,9)=0;
    right(0,10)=8.10357;right(1,10)=5.42361;right(2,10)=7.94439;right(3,10)=0;
    right(0,11)=8.48007;right(1,11)=5.57996;right(2,11)=8.28688;right(3,11)=0;
    right(0,12)=8.86988;right(1,12)=5.75713;right(2,12)=8.6341;right(3,12)=0;
    int K = adore::mad::computeLaneWidthAndCenter(left,right,center);
    {
        std::cout<<"K="<<K<<std::endl;
        for(int i=0;i<K;i++)
        {
            std::cout<<center(0,i)<<";"<<center(1,i)<<";"<<center(2,i)<<";"<<center(3,i)<<";"<<center(4,i)<<std::endl;
        }
    }
    REQUIRE(K==17);
    //check last point x present
    REQUIRE(std::abs(center(1,16)-(left(1,N-1)+right(1,M-1))*0.5)<1.0e-10);
    //check last point y present
    REQUIRE(std::abs(center(2,16)-(left(2,N-1)+right(2,M-1))*0.5)<1.0e-10);
}
