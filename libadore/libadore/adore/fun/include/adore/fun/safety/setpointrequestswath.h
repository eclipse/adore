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
 *   Daniel Heß - simple conversion from SetPointRequest to OccupancyCylinderTree
 ********************************************************************************/
#pragma once
#include <adore/mad/occupancycylinder.h>
#include <adore/fun/setpointrequest.h>
#include <adore/mad/adoremath.h>


namespace adore
{
namespace fun
{

class SetPointRequestSwath
{
private:
    double l_;/**<length of vehicle*/
    double w_;/**<width of vehicle*/
    double p_;/**<distance from rear bumper to reference point of set point request*/
    double lat_precision_;/**< maximum lateral overapproximation of the vehicle body*/
    double lat_error_;/**<initial (and constant) (measurement) error to the side of the vehicle*/
    double lon_error_;/**<initial (measurement) error in movement direction of the vehicle*/
    double duration_;/**<minimum prediction duration for vehicle in standstill*/
    double a_e_;/**<being slower than expected acceleration error*/
public:
    void setL(double value){l_=value;}
    void setW(double value){w_=value;}
    void setP(double value){p_=value;}
    void setLatPrecision(double value){lat_precision_=value;}
    void setLatError(double value){lat_error_=value;}
    void setLonError(double value){lon_error_=value;}
    void setDuration(double value){duration_=value;}
    void setAccelerationErrorSlow(double value){a_e_=value;}
    SetPointRequestSwath(double l,double w,double p,double lat_precision,double lat_error=0.0,double lon_error=0.0)
        :l_(l),w_(w),p_(p),lat_precision_(lat_precision),lat_error_(lat_error),lon_error_(lon_error),duration_(10.0),a_e_(0.0){}

    /**
     * Represents the space occupied by the vehicle during execution of a SetPointRequest as an OccupancyCylinderTree.
     * It is assumed that deviation from the SetPointRequest is constant.
     * The coverage is computed approximated using a simple, sampling-based approach with linear interpolation for the motion of the vehicle body.
     * The curving of non-linear paths is ignored and it is assumed that interpolation of given SetPointRequest is sufficient to achieve desired precision. 
     * The resulting cylinders are appended to level 0 of the OccupancyCylinderTree.
     */
    void append_cylinder_swath_linear(const SetPointRequest& spr, adore::mad::OccupancyCylinderTree& prediction,bool terminal=false)
    {
        const double b = w_*0.5 + lat_error_;
        const double r0 = b+lat_precision_;
        const double ds = 2.0 * std::sqrt(r0*r0-b*b);
        int n = spr.setPoints.size();
        adore::mad::LLinearPiecewiseFunctionM<double,4> path;//s->t,x,y,psi
        adore::mad::LLinearPiecewiseFunctionM<double,1> rear_end;//s->t
        adore::mad::LLinearPiecewiseFunctionM<double,1> front_end;//s->t
        path.getData().set_size( 4 + 1, n + 2 );
        rear_end.getData().set_size( 1 + 1, n );
        front_end.getData().set_size( 1 + 1, n );
        //set initial path point at rear end of vehicle at t0
        double cpsi = std::cos(spr.setPoints[0].x0ref.getPSI());
        double spsi = std::sin(spr.setPoints[0].x0ref.getPSI());
        path.getData()(0,0) = -p_ - lon_error_;//s0
        path.getData()(1,0) = cpsi * (-p_ - lon_error_) + spr.setPoints[0].x0ref.getX();//x0
        path.getData()(2,0) = spsi * (-p_ - lon_error_) + spr.setPoints[0].x0ref.getY();//y0
        path.getData()(3,0) = spr.setPoints[0].tStart;//t0
        //set second path point at reference point of vehicle at t0
        path.getData()(0,1) = 0.0;//s1
        path.getData()(1,1) = spr.setPoints[0].x0ref.getX();//x1
        path.getData()(2,1) = spr.setPoints[0].x0ref.getY();//y1
        path.getData()(3,1) = spr.setPoints[0].tStart;//t1
        for(int i=1;i<n;i++)
        {
            path.getData()(1,i+1) = spr.setPoints[i].x0ref.getX();//xi
            path.getData()(2,i+1) = spr.setPoints[i].x0ref.getY();;//yi
            path.getData()(3,i+1) = spr.setPoints[i].tStart;//ti
            double dx = path.getData()(1,i+1)-path.getData()(1,i);
            double dy = path.getData()(2,i+1)-path.getData()(2,i);
            path.getData()(0,i+1) = std::max(path.getData()(0,i) + std::sqrt(dx*dx+dy*dy),path.getData()(0,i)+0.0000001);
        }
        cpsi = std::cos(spr.setPoints[n-1].x0ref.getPSI());
        spsi = std::sin(spr.setPoints[n-1].x0ref.getPSI());
        //set last path point at at front end of vehicle at tend
        path.getData()(0,n+1) = path.getData()(0,n) + l_ -p_ + lon_error_;//send
        path.getData()(1,n+1) = cpsi * (l_ -p_ + lon_error_) + spr.setPoints[n-1].x0ref.getX();//xend
        path.getData()(2,n+1) = spsi * (l_ -p_ + lon_error_) + spr.setPoints[n-1].x0ref.getY();//yend
        path.getData()(3,n+1) = spr.setPoints[n-1].tStart;//tend
        //compute the time bounds for front end and rear end
        for(int i=0;i<n;i++)
        {
            rear_end.getData()(0,i) = path.getData()(0,i+1) - p_ - lon_error_;
            rear_end.getData()(1,i) = path.getData()(3,i+1);
            front_end.getData()(0,i) = path.getData()(0,i+1) - p_ + l_ + lon_error_;
            front_end.getData()(1,i) = path.getData()(3,i+1);
            if( i > 0 )
            {
                double t = path.getData()(3,i+1)-path.getData()(3,0);
                double s_e = 0.5 * a_e_ * t * t;
                //apply position error due to breaking error
                rear_end.getData()(0,i) = std::max(rear_end.getData()(0,i) + s_e,rear_end.getData()(0,i-1)+0.01);//do not move backwards
            }
        }
        //extend the lower bound in time, if a terminating maneuver
        if(terminal)
        {
            //if the trajectory terminates at endpoint, prolong standstill until maximum duration
            rear_end.getData()(1,n-1) = std::max(rear_end.getData()(1,n-1),rear_end.getData()(1,0)+duration_);
        }

        double s0 = rear_end.limitLo() + ds * 0.5;
        double s1 = front_end.limitHi() - ds * 0.5;
        s1 = s0 + std::ceil((s1-s0)/ds)*ds + ds * 0.00001;
        // std::cout<<"s-interval=["<<s0<<";"<<s1<<"]"<<std::endl;
        for(double s = s0;s<=s1;s += ds)
        {
            double r = r0;//@TODO: recompute r to account for body rotation
            auto pos = path.f(adore::mad::bound(path.limitLo(),s,path.limitHi()));
            double t0i = front_end.f(adore::mad::bound(front_end.limitLo(),s-r,front_end.limitHi()));
            double t1i = rear_end.f(adore::mad::bound(rear_end.limitLo(),s+r,rear_end.limitHi()));
            prediction.insert(adore::mad::OccupancyCylinder(r,pos(0),pos(1),t0i,t1i));            
        }
        
    }

};

}
}