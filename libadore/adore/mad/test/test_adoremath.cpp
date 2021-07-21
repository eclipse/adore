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
#include "adore/mad/adoremath.h"
#include "adore/mad/llinearpiecewisefunction.h"

TEST_CASE( "testing adore min", "[adoremath]" ) {
    REQUIRE( adore::mad::min<int>(1,2,3,4) == 1 );
}



TEST_CASE( "testing toRelativeWithNormalExtrapolation with constant curvature", "[adoremath]" ) 
{
    adoreMatrix<double, 4,4> p;
    adoreMatrix<double, 2,4> n;
    //a unit square (-1,-1) (1,1) approximating a unit circle
    p(0,0) = 0.0;
    p(1,0) = 1.0;
    p(2,0) = 1.0;
    p(0,1) = M_PI*0.5;
    p(1,1) = -1.0;
    p(2,1) = 1.0;
    p(0,2) = M_PI;
    p(1,2) = -1.0;
    p(2,2) = -1.0;
    p(0,3) = M_PI*1.5;
    p(1,3) = 1.0;
    p(2,3) = -1.0;
    p(0,4) = M_PI*2.0;
    p(1,4) = 1.0;
    p(2,4) = 1.0;
    const double d = std::sqrt(0.5);
    n(0,0) = -d;
    n(1,0) = -d;
    n(0,1) = d;
    n(1,1) = -d;
    n(0,2) = d;
    n(1,2) = d;
    n(0,3) = -d;
    n(1,3) = d;
    n(0,4) = -d;
    n(1,4) = -d;

    double s,t,qX,qY,rX,rY,rZ;int i,j;

    qX = 0.5; qY = 0.0; i=3; j=4;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);
    std::cout<<"s="<<s<<std::endl;
    std::cout<<"t="<<t<<std::endl;
    std::cout<<"qX="<<qX<<std::endl;
    std::cout<<"qY="<<qY<<std::endl;
    std::cout<<"rX="<<rX<<std::endl;
    std::cout<<"rY="<<rY<<std::endl;
    

    qX = 0.0; qY = 0.5; i=0; j=1;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);

    qX = -0.5; qY = 0.0; i=1; j=2;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);

    qX = 0.0; qY = -0.5; i=2; j=3;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);

    qX = 1.5; qY = 0.0; i=3; j=4;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);    

    qX = 0.0; qY = 1.5; i=0; j=1;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);

    qX = -1.5; qY = 0.0; i=1; j=2;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);

    qX = 0.0; qY = -1.5; i=2; j=3;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);

    qX = 1.5; qY = 1.0; i=3; j=4;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);    

    qX = 1.0; qY = 1.5; i=0; j=1;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);

    qX = -1.5; qY = -1.0; i=1; j=2;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);

    qX = -1.0; qY = -1.5; i=2; j=3;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
    REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
    REQUIRE(std::abs(qX-rX)<1.0e-10);
    REQUIRE(std::abs(qY-rY)<1.0e-10);
}


TEST_CASE( "testing toRelativeWithNormalExtrapolation with variable curvature", "[adoremath]" ) 
{
    {
        adoreMatrix<double, 4,2> p;
        adoreMatrix<double, 2,2> n;
        //a unit square (-1,-1) (1,1) approximating a unit circle
        p(0,0) = 0.0;
        p(1,0) = 0.0;
        p(2,0) = 0.0;
        p(0,1) = 10;
        p(1,1) = 9.0;
        p(2,1) = 0.0;
        n(0,0) = -1/std::sqrt(3);
        n(1,0) = 2/std::sqrt(3);
        n(0,1) = 0;
        n(1,1) = 1;

        double s,t,qX,qY,rX,rY,rZ;int i,j;

        qX = 3.5; qY = 1.2; i=0; j=1;
        REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
        REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
        REQUIRE(std::abs(qX-rX)<1.0e-10);
        REQUIRE(std::abs(qY-rY)<1.0e-10);
        std::cout<<"s="<<s<<std::endl;
        std::cout<<"t="<<t<<std::endl;
        std::cout<<"qX="<<qX<<std::endl;
        std::cout<<"qY="<<qY<<std::endl;
        std::cout<<"rX="<<rX<<std::endl;
        std::cout<<"rY="<<rY<<std::endl;
    }    
    {
        adoreMatrix<double, 4,2> p;
        adoreMatrix<double, 2,2> n;
        //a unit square (-1,-1) (1,1) approximating a unit circle
        p(0,0) = 0.0;
        p(1,0) = 0.0;
        p(2,0) = 0.0;
        p(0,1) = 10;
        p(1,1) = 9.0;
        p(2,1) = 0.0;
        n(0,0) = 1/std::sqrt(3);
        n(1,0) = 2/std::sqrt(3);
        n(0,1) = 0;
        n(1,1) = 1;

        double s,t,qX,qY,rX,rY,rZ;int i,j;

        qX = 3.5; qY = 1.2; i=0; j=1;
        REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(qX,qY,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),s,t));
        REQUIRE(adore::mad::fromRelative(s,t,dlib::colm(p,i),dlib::colm(p,j),dlib::colm(n,i),dlib::colm(n,j),rX,rY,rZ));
        REQUIRE(std::abs(qX-rX)<1.0e-10);
        REQUIRE(std::abs(qY-rY)<1.0e-10);
        std::cout<<"s="<<s<<std::endl;
        std::cout<<"t="<<t<<std::endl;
        std::cout<<"qX="<<qX<<std::endl;
        std::cout<<"qY="<<qY<<std::endl;
        std::cout<<"rX="<<rX<<std::endl;
        std::cout<<"rY="<<rY<<std::endl;
    } 
}


TEST_CASE("fromRelative with function","[adoremath]")
{
    adoreMatrix<double,3,5> m;
    m(0,0) = 0.0; m(1,0) = 0.0; m(2,0) = 0.0;
    m(0,1) = 1.0; m(1,1) = 1.0; m(2,1) = 0.0;
    m(0,2) = 2.0; m(1,2) = 2.0; m(2,2) = 0.0;
    m(0,3) = 3.0; m(1,3) = 3.0; m(2,3) = 0.0;
    m(0,4) = 4.0; m(1,4) = 4.0; m(2,4) = 0.0;
    adore::mad::LLinearPiecewiseFunctionM<double,2> centerline(m);

    adoreMatrix<double,3,5> n;
    n(0,0) = 0.0; n(1,0) = 0.0; n(2,0) = 1.0;
    n(0,1) = 1.0; n(1,1) = 0.0; n(2,1) = 1.0;
    n(0,2) = 2.0; n(1,2) = 0.0; n(2,2) = 1.0;
    n(0,3) = 3.0; n(1,3) = 0.0; n(2,3) = 1.0;
    n(0,4) = 4.0; n(1,4) = 0.0; n(2,4) = 1.0;
    adore::mad::LLinearPiecewiseFunctionM<double,2> normals(n);

    double s = 2.5;
    double t = 1.4;
    double X,Y,Z;
    adore::mad::fromRelative(s,t,&centerline,&normals,X,Y,Z);
    REQUIRE(X==s);
    REQUIRE(Y==t);

}



TEST_CASE("toRelative with function","[adoremath]")
{
    adoreMatrix<double,3,5> m;
    m(0,0) = 0.0; m(1,0) = 0.0; m(2,0) = 0.0;
    m(0,1) = 1.0; m(1,1) = 1.0; m(2,1) = 0.0;
    m(0,2) = 2.0; m(1,2) = 2.0; m(2,2) = 0.0;
    m(0,3) = 3.0; m(1,3) = 3.0; m(2,3) = 0.0;
    m(0,4) = 4.0; m(1,4) = 4.0; m(2,4) = 0.0;
    adore::mad::LLinearPiecewiseFunctionM<double,2> centerline(m);

    adoreMatrix<double,3,5> n;
    n(0,0) = 0.0; n(1,0) = 0.0; n(2,0) = 1.0;
    n(0,1) = 1.0; n(1,1) = 0.0; n(2,1) = 1.0;
    n(0,2) = 2.0; n(1,2) = 0.0; n(2,2) = 1.0;
    n(0,3) = 3.0; n(1,3) = 0.0; n(2,3) = 1.0;
    n(0,4) = 4.0; n(1,4) = 0.0; n(2,4) = 1.0;
    adore::mad::LLinearPiecewiseFunctionM<double,2> normals(n);

    double s=0.0,t=0.0;
    double X = 2.5;
    double Y = 1.4;
    REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(X,Y,&centerline,&normals,s,t));
    REQUIRE(X==s);
    REQUIRE(Y==t);

}


TEST_CASE("toRelative with function and noisy normals","[adoremath]")
{
    adoreMatrix<double,3,5> m;
    m(0,0) = 0.0; m(1,0) = 0.0; m(2,0) = 0.0;
    m(0,1) = 1.0; m(1,1) = 1.0; m(2,1) = 0.0;
    m(0,2) = 2.0; m(1,2) = 2.0; m(2,2) = 0.0;
    m(0,3) = 3.0; m(1,3) = 3.0; m(2,3) = 0.0;
    m(0,4) = 4.0; m(1,4) = 4.0; m(2,4) = 0.0;
    adore::mad::LLinearPiecewiseFunctionM<double,2> centerline(m);

    adoreMatrix<double,3,5> n;
    n(0,0) = 0.0; n(1,0) = -0.01; n(2,0) = 1.0;
    n(0,1) = 1.0; n(1,1) = 0.01; n(2,1) = 1.0;
    n(0,2) = 2.0; n(1,2) = 0.02; n(2,2) = 1.0;
    n(0,3) = 3.0; n(1,3) = -0.02; n(2,3) = 1.0;
    n(0,4) = 4.0; n(1,4) = 0.01; n(2,4) = 1.0;
    adore::mad::LLinearPiecewiseFunctionM<double,2> normals(n);

    for(double X = 0.3;X<4.0;X+=0.3)for(double Y = -4;Y<4;Y+=0.3)
    {
        std::cout<<"X="<<X<<"; Y="<<Y<<std::endl;
        double s=0.0,t=0.0;
        REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(X,Y,&centerline,&normals,s,t));
        std::cout<<"s="<<s<<"; t="<<t<<std::endl;
        REQUIRE(std::abs(X-s)<0.1);
        REQUIRE(std::abs(Y-t)<0.1);
    }

}

TEST_CASE("toRelative with function and noisy normals, rotated","[adoremath]")
{
    double psi = -0.85;
    for(double psi = -M_PI;psi<M_PI;psi+=0.3)
    {
        double cpsi = std::cos(psi);
        double spsi = std::sin(psi);
        adoreMatrix<double,3,5> m;
        m(0,0) = 0.0; m(1,0) = 0.0; m(2,0) = 0.0;
        m(0,1) = 1.0; m(1,1) = 1.0; m(2,1) = 0.0;
        m(0,2) = 2.0; m(1,2) = 2.0; m(2,2) = 0.0;
        m(0,3) = 3.0; m(1,3) = 3.0; m(2,3) = 0.0;
        m(0,4) = 4.0; m(1,4) = 4.0; m(2,4) = 0.0;
        for(int i=0;i<5;i++)
        {
            double X = m(1,i), Y = m(2,i);
            m(1,i) = cpsi * X - spsi * Y;
            m(2,i) = spsi * X + cpsi * Y;
        }
        adore::mad::LLinearPiecewiseFunctionM<double,2> centerline(m);


        adoreMatrix<double,3,5> n;
        n(0,0) = 0.0; n(1,0) = -0.01; n(2,0) = 1.0;
        n(0,1) = 1.0; n(1,1) = 0.01; n(2,1) = 1.0;
        n(0,2) = 2.0; n(1,2) = 0.02; n(2,2) = 1.0;
        n(0,3) = 3.0; n(1,3) = -0.02; n(2,3) = 1.0;
        n(0,4) = 4.0; n(1,4) = 0.01; n(2,4) = 1.0;
        for(int i=0;i<5;i++)
        {
            double X = n(1,i), Y = n(2,i);
            n(1,i)   = cpsi * X - spsi * Y;
            n(2,i)   = spsi * X + cpsi * Y;
        }
        adore::mad::LLinearPiecewiseFunctionM<double,2> normals(n);

        for(double X = 0.3;X<4.0;X+=0.3)for(double Y = -1.4;Y<1.4;Y+=0.3)
        {
            double Xt = cpsi * X - spsi * Y;
            double Yt = spsi * X + cpsi * Y;
            std::cout<<"X="<<X<<"; Y="<<Y<<std::endl;
            double s=0.0,t=0.0;
            REQUIRE(adore::mad::toRelativeWithNormalExtrapolation(Xt,Yt,&centerline,&normals,s,t));
            std::cout<<"s="<<s<<"; t="<<t<<std::endl;
            REQUIRE(std::abs(X-s)<0.1);
            REQUIRE(std::abs(Y-t)<0.1);
        }
    }
}

