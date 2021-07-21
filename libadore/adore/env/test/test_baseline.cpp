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
 *   Daniel Heß - unit tests for borderbased/baseline.h
 *********************************************************************************/

#include <adore/env/borderbased/baseline.h>
#include <catch2/catch.hpp>
#include <math.h>

TEST_CASE( "straight line, angle=0", "[adore::env::BorderBased::Baseline::update]" ) 
{
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::Baseline Baseline;
    typedef adore::env::BorderBased::BorderSequence BorderSequence;

    const double psi = 0.0;
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);
    const double ds = 10.0;
    const double soffset = 250.0;
    const int N = 50;

    BorderSequence bs;
    for(int i=0;i<N;i++)
    {
        double di = ds*(double)i;
        double dj = ds*(double)(i+1);
        bs.push_back(new Border(cpsi*di,spsi*di,cpsi*dj,spsi*dj));
    }
    Baseline base;
    base.update(bs,soffset);
    REQUIRE(base.isValid());
    REQUIRE(base.base_L_[0]==0.0);
    REQUIRE(std::abs(base.base_L_[base.getNSamplePoints()-1]-base.getLookAhead()-base.getLookBehind())<1.0e-6);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_kappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_dkappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_nx_[i]-(-spsi))<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_ny_[i]-(cpsi))<1.0e-10);
    

    for(auto b:bs)delete b;
}

TEST_CASE( "straight line, angle=pi/4", "[adore::env::BorderBased::Baseline::update]" ) 
{
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::Baseline Baseline;
    typedef adore::env::BorderBased::BorderSequence BorderSequence;

    const double psi = M_PI*0.25;
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);
    const double ds = 10.0;
    const double soffset = 250.0;
    const int N = 50;

    BorderSequence bs;
    for(int i=0;i<N;i++)
    {
        double di = ds*(double)i;
        double dj = ds*(double)(i+1);
        bs.push_back(new Border(cpsi*di,spsi*di,cpsi*dj,spsi*dj));
    }
    Baseline base;
    base.update(bs,soffset);
    REQUIRE(base.isValid());
    REQUIRE(base.base_L_[0]==0.0);
    REQUIRE(std::abs(base.base_L_[base.getNSamplePoints()-1]-base.getLookAhead()-base.getLookBehind())<1.0e-6);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_kappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_dkappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_nx_[i]-(-spsi))<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_ny_[i]-(cpsi))<1.0e-10);
    

    for(auto b:bs)delete b;
}

TEST_CASE( "two straight lines", "[adore::env::BorderBased::Baseline::defineOffset]" ) 
{
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::Baseline Baseline;
    typedef adore::env::BorderBased::BorderSequence BorderSequence;

    const double psi = 0;
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);
    const double ds = 10.0;
    const double soffset = 250.0;
    const int N = 50;
    const double offset = 5.0;

    BorderSequence bs;
    for(int i=0;i<N;i++)
    {
        double di = ds*(double)i;
        double dj = ds*(double)(i+1);
        bs.push_back(new Border(cpsi*di,spsi*di,cpsi*dj,spsi*dj));
    }
    BorderSequence parallel;
    for(int i=0;i<N;i++)
    {
        double di = ds*(double)i;
        double dj = ds*(double)(i+1);
        parallel.push_back(new Border(cpsi*di,offset+spsi*di,cpsi*dj,offset+spsi*dj));
    }

    Baseline base;
    base.update(bs,soffset);
    REQUIRE(base.isValid());
    adore::mad::function_type_scalar offset_fct;
    base.defineOffset(parallel,&offset_fct);
    // for(int i=0;i<offset_fct.getData().nc();i++)std::cout<<"("<<offset_fct.getData()(0,i)<<","<<offset_fct.getData()(1,i)<<")\n";
    for(double s = 0.0;s<200.0;s+=23.0)REQUIRE(std::abs(offset_fct(s)-offset)<1.0e-10);

    for(auto b:bs)delete b;
    for(auto b:parallel)delete b;
}


TEST_CASE( "straight line, angle=pi/2", "[adore::env::BorderBased::Baseline::update" ) 
{
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::Baseline Baseline;
    typedef adore::env::BorderBased::BorderSequence BorderSequence;

    const double psi = M_PI*0.5;
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);
    const double ds = 10.0;
    const double soffset = 250.0;
    const int N = 50;

    BorderSequence bs;
    for(int i=0;i<N;i++)
    {
        double di = ds*(double)i;
        double dj = ds*(double)(i+1);
        bs.push_back(new Border(cpsi*di,spsi*di,cpsi*dj,spsi*dj));
    }
    Baseline base;
    base.update(bs,soffset);
    REQUIRE(base.isValid());
    REQUIRE(base.base_L_[0]==0.0);
    REQUIRE(std::abs(base.base_L_[base.getNSamplePoints()-1]-base.getLookAhead()-base.getLookBehind())<1.0e-6);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_kappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_dkappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_nx_[i]-(-spsi))<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_ny_[i]-(cpsi))<1.0e-10);
    

    for(auto b:bs)delete b;
}


TEST_CASE( "straight line, angle=-pi*0.25", "[adore::env::BorderBased::Baseline::update]" ) 
{
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::Baseline Baseline;
    typedef adore::env::BorderBased::BorderSequence BorderSequence;

    const double psi = -M_PI*0.25;
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);
    const double ds = 10.0;
    const double soffset = 250.0;
    const int N = 50;

    BorderSequence bs;
    for(int i=0;i<N;i++)
    {
        double di = ds*(double)i;
        double dj = ds*(double)(i+1);
        bs.push_back(new Border(cpsi*di,spsi*di,cpsi*dj,spsi*dj));
    }
    Baseline base;
    base.update(bs,soffset);
    REQUIRE(base.isValid());
    REQUIRE(base.base_L_[0]==0.0);
    REQUIRE(std::abs(base.base_L_[base.getNSamplePoints()-1]-base.getLookAhead()-base.getLookBehind())<1.0e-6);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_kappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_dkappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_nx_[i]-(-spsi))<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_ny_[i]-(cpsi))<1.0e-10);
    

    for(auto b:bs)delete b;
}


TEST_CASE( "straight line, angle=-pi", "[adore::env::BorderBased::Baseline::update]" ) 
{
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::Baseline Baseline;
    typedef adore::env::BorderBased::BorderSequence BorderSequence;

    const double psi = -M_PI;
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);
    const double ds = 10.0;
    const double soffset = 250.0;
    const int N = 50;

    BorderSequence bs;
    for(int i=0;i<N;i++)
    {
        double di = ds*(double)i;
        double dj = ds*(double)(i+1);
        bs.push_back(new Border(cpsi*di,spsi*di,cpsi*dj,spsi*dj));
    }
    Baseline base;
    base.update(bs,soffset);
    REQUIRE(base.isValid());
    REQUIRE(base.base_L_[0]==0.0);
    REQUIRE(std::abs(base.base_L_[base.getNSamplePoints()-1]-base.getLookAhead()-base.getLookBehind())<1.0e-6);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_kappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_dkappa_[i])<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_nx_[i]-(-spsi))<1.0e-10);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_ny_[i]-(cpsi))<1.0e-10);
    

    for(auto b:bs)delete b;
}

TEST_CASE( "circle w/ radius=500m", "[adore::env::BorderBased::Baseline::update]" ) 
{
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::Baseline Baseline;
    typedef adore::env::BorderBased::BorderSequence BorderSequence;

    const double psi0 = 0;
    const double ds = 1.0;
    const double soffset = 250.0;
    const double R = 500.0;
    const int N = 500;

    BorderSequence bs;
    for(int i=0;i<N;i++)
    {
        const double psii = ds*(double)(i)/R + psi0;
        const double psij = ds*(double)(i+1)/R + psi0;
        bs.push_back(new Border(std::cos(psii)*R,std::sin(psii)*R,std::cos(psij)*R,std::sin(psij)*R));
    }
    Baseline base;
    base.update(bs,soffset);
    REQUIRE(base.isValid());
    REQUIRE(base.base_L_[0]==0.0);
    REQUIRE(std::abs(base.base_L_[base.getNSamplePoints()-1]-base.getLookAhead()-base.getLookBehind())<1.0e-2);
    // for(int i=0;i<base.getNSamplePoints();i+=25)std::cout<<"p"<<i<<": ("<<base.samples_x_[i]<<","<<base.samples_y_[i]<<")\n";
    // for(int i=0;i<base.getNSamplePoints();i+=25)std::cout<<"derx"<<i<<": ("<<base.samples_dx_[i]<<","<<base.samples_ddx_[i]<<","<<base.samples_dddx_[i]<<")\n";
    // for(int i=0;i<base.getNSamplePoints();i+=25)std::cout<<"dery"<<i<<": ("<<base.samples_dy_[i]<<","<<base.samples_ddy_[i]<<","<<base.samples_dddy_[i]<<")\n";
    // for(int i=(int)(0.2*(double)base.getNSamplePoints());i<(int)(0.8*(double)base.getNSamplePoints());i+=10)std::cout<<"R"<<i<<": ("<<1.0/base.base_kappa_[i]<<")\n";
    for(int i=(int)(0.2*(double)base.getNSamplePoints());i<(int)(0.8*(double)base.getNSamplePoints());i++)REQUIRE(std::abs(1.0/base.base_kappa_[i]-R)/R<0.01);
    for(int i=0;i<base.getNSamplePoints();i++)REQUIRE(std::abs(base.base_nx_[i]*base.base_nx_[i]+base.base_ny_[i]*base.base_ny_[i]-1.0)<1.0e-10);
    for(int i=0;i<base.getNSamplePoints()-1;i++)
    {
        //test change of curvature against finite difference of curvature
        const double deltakappa = (base.base_kappa_[i+1]-base.base_kappa_[i]) / (base.base_L_[i+1]-base.base_L_[i]);
        const double dkappadL = base.base_dkappa_[i];
        // std::cout<<"dkappadL"<<i<<"="<<dkappadL<<", deltakappa"<<i<<"="<<deltakappa<<"\n";
        REQUIRE(std::abs(deltakappa-dkappadL)<1.0e-3);
    }
    

    for(auto b:bs)delete b;
}
