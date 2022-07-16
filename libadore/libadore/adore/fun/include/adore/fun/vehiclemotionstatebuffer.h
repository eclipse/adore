/********************************************************************************
 * Copyright (C) 2017-2021 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#pragma once

#include <adore/fun/afactory.h>

namespace adore
{
namespace fun
{

    /**
     * @brief Buffers and interpolates vehicle positions
     */
    class VehicleMotionStateBuffer
    {
        private:
        adore::fun::AFactory::TMotionStateFeed* feed_;
        std::list<VehicleMotionState9d> buffer_;
        double interp(double t,double ta,double a,double tb,double b)
        {
            return a + (b-a)*(t-ta)/(tb-ta);
        }
        double dt_max_;
        public:
        VehicleMotionStateBuffer(adore::fun::AFactory::TMotionStateFeed* feed,double dt_max=1.0)
            :feed_(feed), dt_max_(dt_max){}
        ~VehicleMotionStateBuffer(){delete feed_;}
        /**
         * @brief call update to check for new data
         */
        void update()
        {
            while(feed_->hasNext())
            {
                VehicleMotionState9d x;
                feed_->getNext(x);
                buffer_.push_back(x);
            }
            while(buffer_.size()>0 && 
                  buffer_.begin()->getTime()+dt_max_<buffer_.rbegin()->getTime())
            {
                buffer_.pop_front();
            }
        }
        double size()
        {
            return buffer_.size();
        }
        double getTmax()
        {
            if(size()==0)return 0.0;
            return buffer_.rbegin()->getTime();
        }
        /**
         * @brief get the latest, if @param t is bigger than latest known time point. Otherwise interpolate.
         * @return true if data was found
         */
        bool interpolate_or_latest(double t, VehicleMotionState9d& result)
        {
            if(buffer_.size()==0)return false;
            if(t<buffer_.begin()->getTime())return false;
            if(t>buffer_.rbegin()->getTime())
            {
                std::cout<<"vehiclemotionstatebuffer: retrieving latest with dt="<<(t-buffer_.rbegin()->getTime())<<std::endl;
                result = *buffer_.rbegin();
                return true;
            }
            return interpolate(t,result);
        }
        /**
         * @brief interpolate a value for time t, if t is in buffered time range
         * @return true if data was found
         */
        bool interpolate(double t, VehicleMotionState9d& result)
        {
            if(buffer_.size()==0)return false;
            if(buffer_.size()==1 && t==buffer_.begin()->getTime())
            {
                result = *buffer_.begin();
                return true;
            }
            if(buffer_.size()<2||t<buffer_.begin()->getTime()||t>buffer_.rbegin()->getTime())return false;
            for(auto a=buffer_.begin();a!=buffer_.end();a++)
            {
                auto b = a;
                b++;
                if(b==buffer_.end())break;
                double ta = a->getTime();
                double tb = b->getTime();
                if(t==ta)
                {
                    result = *a;
                    return true;
                }
                if(t==tb)
                {
                    result = *b;
                    return true;
                }
                if(ta<=t && t<tb)
                {
                    result.setTime(t);
                    result.setX(interp(t,ta,a->getX(),tb,b->getX()));
                    result.setY(interp(t,ta,a->getY(),tb,b->getY()));
                    result.setZ(interp(t,ta,a->getZ(),tb,b->getZ()));
                    const double ca = std::cos(a->getPSI());
                    const double sa = std::sin(a->getPSI());
                    const double cb = std::cos(b->getPSI());
                    const double sb = std::sin(b->getPSI());
                    const double dpsi = std::atan2(-sa*cb+ca*sb,ca*cb+sa*sb);
                    result.setPSI(a->getPSI()+interp(t,ta,0,tb,dpsi));
                    result.setvx(interp(t,ta,a->getvx(),tb,b->getvx()));
                    result.setvy(interp(t,ta,a->getvy(),tb,b->getvy()));
                    result.setOmega(interp(t,ta,a->getOmega(),tb,b->getOmega()));
                    result.setAx(interp(t,ta,a->getAx(),tb,b->getAx()));
                    result.setDelta(interp(t,ta,a->getDelta(),tb,b->getDelta()));
                    return true;
                }
            }
            return false;
        }
    };

}
}