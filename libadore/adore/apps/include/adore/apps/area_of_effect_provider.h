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
 *   Daniel Heß - initial implementation
 ********************************************************************************/


#pragma once

#include <adore/env/afactory.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <adore/env/situation/areaofeffect.h>


namespace adore
{
namespace apps
{
/**
 * Provides boundary of area, which is currently reachable by vehicle:
 * ThreeLaneView is processed to compute and publish outline.
 */
class AreaOfEffectProvider
{
    private:
    using EnvFactory = adore::env::EnvFactoryInstance;
    adore::env::AFactory::TAreaOfEffectWriter* writer_;/**<write output*/
    adore::env::AFactory::TVehicleMotionStateReader* xreader_;/**<read vehicle state to crop lane view behind behind vehicle*/
    adore::env::ThreeLaneViewDecoupled three_lanes_;/**<lane-based representation of environment*/
    double resolution_;
    double lookbehind_;
    std::vector<double> s_;/**<s values encircling the area of effect in ccw direction*/
    public:
    AreaOfEffectProvider(double resolution = 1.0):
        resolution_(resolution),lookbehind_(10.0)
    {
        for(int i=0;i<=13;i++)s_.push_back(0.0);//initialize with zeros
        writer_ = EnvFactory::get()->getAreaOfEffectWriter();//initialize output writer
        xreader_ = EnvFactory::get()->getVehicleMotionStateReader();//initialize vehicle state reader
    }
    void run()
    {
        if(!xreader_->hasData())return;        
        adore::env::VehicleMotionState9d x;
        xreader_->getData(x);
        three_lanes_.update();
        adore::view::AThreeLaneView* view = &three_lanes_;
        adore::view::ALane* source = view->getCurrentLane();
        adore::view::ALane* leftTarget = nullptr;
        adore::view::ALane* rightTarget = nullptr;
        if( view->getRightLaneChange()!=nullptr 
        &&  view->getRightLaneChange()->getTargetLane()->isValid())
        {
            rightTarget = view->getRightLaneChange()->getTargetLane();
        }
        if( view->getLeftLaneChange()!=nullptr 
        &&  view->getLeftLaneChange()->getTargetLane()->isValid())
        {
            leftTarget = view->getLeftLaneChange()->getTargetLane();
        }
        

        adore::env::AreaOfEffect aoe;
        if(view->getCurrentLane()->isValid())
        {
            computeSValues(&three_lanes_,x.getX(),x.getY());
            sampleLane(source,s_[0],s_[1],aoe);
            if(rightTarget!=nullptr)
            {
                sampleLane(rightTarget,s_[2],s_[3],aoe);
                sampleLane(rightTarget,s_[3],s_[4],aoe);
                sampleLane(rightTarget,s_[4],s_[5],aoe);
            }
            sampleLane(source,s_[6],s_[7],aoe);
            sampleLane(source,s_[7],s_[8],aoe);
            if(leftTarget!=nullptr)
            {
                sampleLane(leftTarget,s_[9],s_[10],aoe);
                sampleLane(leftTarget,s_[10],s_[11],aoe);
                sampleLane(leftTarget,s_[11],s_[12],aoe);
            }
            sampleLane(source,s_[13],s_[0],aoe);
        }
        writer_->write(aoe);
    }

    void computeSValues(adore::view::AThreeLaneView* view,double X0,double Y0)
    {
        double X,Y,Z,s,n;
        auto source = view->getCurrentLane();
        source->toRelativeCoordinates(X0,Y0,s,n);
        double smin = std::max(s-lookbehind_,source->getSMin());
        //ccw list of s values
        s_[0] = smin;
        s_[1] = s_[0];
        s_[2] = 0.0;
        s_[3] = 0.0;
        s_[4] = 0.0;
        s_[5] = 0.0;
        s_[6] = s_[0];
        s_[7] = source->getSMax();
        s_[8] = s_[7];
        s_[9] = 0.0;
        s_[10] = 0.0;
        s_[11] = 0.0;
        s_[12] = 0.0;
        s_[13] = s_[7];
        if( view->getRightLaneChange()!=nullptr 
        &&  view->getRightLaneChange()->getTargetLane()->isValid())
        {
            auto lc = view->getRightLaneChange(); 
            auto target = lc->getTargetLane();

            s_[1] = std::max(smin,lc->getProgressOfGateOpen());
            //s2: translate gate open to target lane progress
            source->toEucledianCoordinates(s_[1],0.0,X,Y,Z);
            target->toRelativeCoordinates(X,Y,s,n);
            s_[2] = s;

            target->toRelativeCoordinates(X0,Y0,s,n);
            s_[3] = std::max(s-lookbehind_,target->getSMin());
            s_[4] = target->getSMax();


            s_[6] = std::max(smin,lc->getProgressOfGateClosed());
            //s5: translate gate closed to target lane progress
            source->toEucledianCoordinates(s_[6],0.0,X,Y,Z);
            target->toRelativeCoordinates(X,Y,s,n);
            s_[5] = s;
        }
        if( view->getLeftLaneChange()!=nullptr 
        &&  view->getLeftLaneChange()->getTargetLane()->isValid())
        {
            auto lc = view->getLeftLaneChange(); 
            auto target = lc->getTargetLane();

            s_[13] = std::max(smin,lc->getProgressOfGateOpen());
            //s12: translate gate open to target lane progress
            source->toEucledianCoordinates(s_[13],0.0,X,Y,Z);
            target->toRelativeCoordinates(X,Y,s,n);
            s_[12] = s;

            target->toRelativeCoordinates(X0,Y0,s,n);
            s_[11] = std::max(s-lookbehind_,target->getSMin());
            s_[10] = target->getSMax();


            s_[8] = std::max(smin,lc->getProgressOfGateClosed());
            //s9: translate gate closed to target lane progress
            source->toEucledianCoordinates(s_[8],0.0,X,Y,Z);
            target->toRelativeCoordinates(X,Y,s,n);
            s_[9] = s;
        }

    }

    void sampleLane(adore::view::ALane* lane,double s0,double s1,adore::env::AreaOfEffect& area)
    {
        if(s0==s1)return;
        bool inverted  = s0>s1;
        double n,X,Y,Z,lasts;
        for(double s=s0;
            (!inverted && s<=s1) || (inverted && s>=s1);
            s+=(inverted?-1.0:1.0)*resolution_)
        {
            lasts = s;
            n = inverted?lane->getOffsetOfLeftBorder(s):lane->getOffsetOfRightBorder(s);
            lane->toEucledianCoordinates(s,n,X,Y,Z);
            area.push_back(std::make_pair(X,Y));
        }
        if(lasts!=s1)
        {
            n = inverted?lane->getOffsetOfLeftBorder(s1):lane->getOffsetOfRightBorder(s1);
            lane->toEucledianCoordinates(s1,n,X,Y,Z);
            area.push_back(std::make_pair(X,Y));
        }
    }
};

}
}