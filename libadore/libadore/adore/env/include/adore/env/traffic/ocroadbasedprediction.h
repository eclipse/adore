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
 *    Daniel Heß - initial implementation and API
 ********************************************************************************/
#pragma once

#include <adore/env/traffic/occupancycylinderprediction.h>
#include <adore/env/traffic/trafficmap.h>
#include <unordered_set>
#include <vector>

namespace adore
{
    namespace env
    {
        /**
         * A prediction along a given road-graph.
         * Challenges:
         * - splits in road graph: continue each arm
         * - merges in road graph: continue target arm
         * - priorities: do not follow arms, which are rated lower priority than arm followed by ego vehicle
         * - narrowing lanes: keep width of object. typically extend object into parallel traffic lane (not head-on)
         * Approach:
         * - one border generates one prediction branch
         * - objects associated with multiple borders (during lane change) fill initial set with all borders
         * - prevent cycles and multi-predictions due to consecutive borders with unordered_set<BorderID> explored
         * - compute boundary functions for fastest and slowest travel s->t, so that smax is known and [t0,t1] can be computed for cylinders
         * - process a border:
         *   - retrieve a border from the open list
         *   - compute centerline between border and left neighbor
         *   - create a new prediction, which references predecessor prediction
         *   - generate cylinders along centerline, add to occupancy of prediction
         *   - add prediction to set
         *   - add all continuous successors of border to open list
         */
        class OCRoadBasedPrediction: public OCPredictionStrategy<traffic::Participant>
        {
            private:
            double t_max_utc_;/**< time at which the prediction ends. (prediciton starts at time stamp of object.)*/
            double lat_precision_;/**< maximum lateral overapproximation of the vehicle body*/
            double lat_error_;/**initial (and constant) (measurement) error to the side of the vehicle*/
            double lon_error_;/**initial (measurement) error in movement direction of the vehicle*/
            double v_max_;/**< maximum speed up until which participant could accelerate. only relevant, if a_max>0*/
            double a_max_;/**< maximum acceleration. set a_max_=a_min_ to get no divergence.*/
            double a_min_;/**< minimum acceleration. set a_max_=a_min_ to get no divergence.*/
            double angle_error_max_;/**< maximum angle error, which is permitted to associate with a border*/
            double time_headway_;/**< arriving earlier than expected*/
            double time_leeway_;/**< leaving later than expected*/
            double delay_;/**<delay after which acceleration is applied*/
            bool lane_width_predictions_;/**<select whether to predict motion of objects up to lane border or only the width of objects relative to lane center*/
            bool lateral_predictions_;/**<select whether to predict radius based on lateral motion up to lane width*/
            double width_ub_;/**< upper bound on prediction width*/
            double width_lb_;/**< lower bound on prediction width*/
            adore::env::traffic::TrafficMap* trafficMap_;/**< a data object, which allows to map between borders and traffic participants*/
            struct SearchState
            {
                public:
                double s0_border_;       /**< where to start on border */
                double s0_participant_;  /**< how many m has the participant progressed, when it starts on border*/
                adore::env::BorderBased::BorderID borderID_; /**< the border to be considered*/
                int predecessorID_;      /**< predecessor branch id*/
                double n0_lb_;              /**< initial tangential distance lower bound*/
                double n0_ub_;              /**< initial tangential distance upper bound*/
                double dn0_lb_;             /**< initial tangential velocity lower bound*/
                double dn0_ub_;             /**< initial tangential velocity upper bound*/
            };

            public:
            void setAngleErrorMax(double value){angle_error_max_=value;}
            void setTMaxUTC(double value){t_max_utc_=value;}
            void setLatPrecision(double value){lat_precision_=value;}
            void setLatError(double value){lat_error_=value;}
            void setLonError(double value){lon_error_=value;}
            void setVMax(double value){v_max_=value;}
            void setAMax(double value){a_max_=value;}
            void setAMin(double value){a_min_=value;}
            void setTimeHeadway(double value){time_headway_=value;}
            void setTimeLeeway(double value){time_leeway_=value;}
            void setDelay(double value){delay_=value;}
            void setLaneWidthPredictions(bool value){lane_width_predictions_=value;}
            void setLateralPredictions(bool value){lateral_predictions_=value;}
            void setWidthUB(double value){width_ub_=value;}
            void setWidthLB(double value){width_lb_=value;}
            OCRoadBasedPrediction(adore::env::traffic::TrafficMap* trafficMap)
                :t_max_utc_(0.0),lat_precision_(0.2),lat_error_(0.0),lon_error_(0.0),
                 v_max_(9999.9),a_max_(0.0),a_min_(0.0),
                 time_headway_(2.0),
                 time_leeway_(1.0),
                 delay_(0.0),
                 width_ub_(10.0),
                 width_lb_(0.5),
                 trafficMap_(trafficMap),
                 lane_width_predictions_(true),
                 lateral_predictions_(true){}
            virtual bool predict(const traffic::Participant& p, OccupancyCylinderPredictionSet& set)const override
            {
                const double width = adore::mad::bound(width_lb_,p.getWidth(),width_ub_);
                const double b = width*0.5 + lat_error_;
                const double r = b+lat_precision_;
                const double ds = 2.0 * std::sqrt(r*r-b*b);
                const double t0 = p.getObservationTime();
                const double t1 = (std::max)(t0,t_max_utc_);
                const double v = std::sqrt(p.getVx()*p.getVx()+p.getVy()*p.getVy());
                const double l = p.getLength() + 2.0*lon_error_;
                const double k = (int)std::ceil(l/ds);
                const double dx = std::cos(p.getYaw());
                const double dy = std::sin(p.getYaw());
                auto pos0 = p.getCenter();
                adore::mad::LLinearPiecewiseFunctionM<double,3> rear_end_s_tva;
                adore::mad::LLinearPiecewiseFunctionM<double,3> front_end_s_tva;
                RampPrediction ramp;
                ramp.predict_s_tva(-l*0.5,v,a_min_,a_min_<0.0?0.0:v_max_,t0,0.1,t1,rear_end_s_tva);//lower bound: back of behicle with amin
                ramp.predict_s_tva(+l*0.5,v,0.0,delay_,a_max_,a_max_<0.0?0.0:v_max_,t0,0.1,t1,front_end_s_tva);//upper bound: front of vehicle with amax
                const double s1 = front_end_s_tva.limitHi() + 0.5*ds;

                //prediction for current shape and position
                {
                    OccupancyCylinderPrediction prediction;
                    int predictionID = set.size();
                    adore::mad::LLinearPiecewiseFunctionM<double,3> center(2,0.0);
                    center.getData()(0,0) = 0.0;
                    center.getData()(1,0) = pos0(0)-l*0.5*dx;
                    center.getData()(2,0) = pos0(1)-l*0.5*dy;
                    center.getData()(3,0) = pos0(2);
                    center.getData()(0,1) = l;
                    center.getData()(1,1) = pos0(0)+l*0.5*dx;
                    center.getData()(2,1) = pos0(1)+l*0.5*dy;
                    center.getData()(3,1) = pos0(2);
                    // if(center.getData().nc()<2)continue;
                    //create a new prediction for the encountered border
                    prediction.trackingID_ = p.getTrackingID();
                    prediction.v2xStationID_ = p.getStationID();
                    prediction.branchID_ = predictionID;
                    prediction.predecessorID_ = -1;
                    prediction.confidence_ = 1.0;
                    for(double s = ds*0.5;s<=l-ds*0.5;s+=ds)
                    {
                        auto pos = center.f(s);
                        double t0i,t1i;
                        //first time entering circle with front of vehicle
                        // t0i = front_end_s_tva.f(adore::mad::bound(front_end_s_tva.limitLo(),s-ds*0.5-0.5*l,front_end_s_tva.limitHi()))(0);
                        // t0i -= time_headway_;
                        t0i = t0;
                        //latest time exiting circle with back of vehicle
                        t1i = rear_end_s_tva.f(adore::mad::bound(rear_end_s_tva.limitLo(),s+ds*0.5-0.5*l,rear_end_s_tva.limitHi()))(0);
                        t1i += time_leeway_;
                        if(!std::isnan(r)&&!std::isnan(pos(0))&&!std::isnan(pos(1))&&!std::isnan(t0)&&!std::isnan(t1))
                        {
                            prediction.occupancy_.insert(adore::mad::OccupancyCylinder(r,pos(0),pos(1),t0i,t1i));
                        }
                    }
                    set.push_back(prediction);
                }


                //predict based on road-network
                std::unordered_set<adore::env::BorderBased::BorderID,adore::env::BorderBased::BorderIDHasher> closed;
                std::vector<SearchState> open;
                //initialize open
                for(auto itpair = trafficMap_->getParticipantToBorder().equal_range(p.getTrackingID());
                    itpair.first!=itpair.second;
                    itpair.first++)
                {
                    auto it = itpair.first;
                    auto test_participantID = it->first;
                    auto borderID = it->second.first;
                    auto positioning = it->second.second;   
                    if(test_participantID==p.getTrackingID() && positioning.anyInside())//start on border, if any point of vehicle is inside
                    {
                        //get the centerline of the lane
                        adore::mad::LLinearPiecewiseFunctionM<double,4> center = trafficMap_->getBorderSet()->getCenterline(borderID);
                        //compute the offset along the centerline
                        double min_tangential_distance, min_normal_distance;
                        double smin = center.getPositionOfPoint(p.center_(0),p.center_(1),1,2,min_tangential_distance,min_normal_distance);
                        if(min_tangential_distance<0.0 || 1.0<min_tangential_distance)continue;

                        SearchState x;
                        // x.s0_border_ = positioning.s;
                        x.s0_border_ = smin;
                        x.s0_participant_ = 0.0;
                        x.predecessorID_ = -1;
                        x.borderID_ = borderID;

                        //angle offset
                        //find two points on right border
                        // auto bright = trafficMap_->getBorderSet()->getBorder(borderID);
                        // if(bright==nullptr)continue;//no border found: prediction impossible
                        //compute orientation information:
                        // const double lah = adore::mad::bound(0.5,l*0.5,1.0);//look ahead/behind
                        const double lah = 1.0;//look ahead/behind
                        // if(positioning.s>=bright->m_path->limitHi()-0.1)continue;
                        const auto p0 = center.f_bounded(x.s0_border_-lah);
                        const auto p1 = center.f_bounded(x.s0_border_+lah);
                        const auto p = center.f_bounded(x.s0_border_);
                        double pdx = p1(0)-p0(0); //path delta x
                        double pdy = p1(1)-p0(1); //path delta y
                        const double lpdxy = std::sqrt(pdx*pdx+pdy*pdy);
                        pdx = pdx / lpdxy;
                        pdy = pdy / lpdxy;
                        if(lpdxy<0.01)continue;//very short border?
                        // const double rpdx =  pdx*dx + pdy*dy;//path delta x rotated in vehicle coordinates
                        // const double rpdy = -pdx*dy + pdy*dx;//path delta y rotated in vehicle coordinates
                        const double rpx = (pos0(0)-p(0))*pdx + (pos0(1)-p(1))*pdy;
                        const double rpy = -(pos0(0)-p(0))*pdy + (pos0(1)-p(1))*pdx;
                        const double rpdx =  dx*pdx + dy*pdy;//path delta x rotated in path coordinates
                        const double rpdy = -dx*pdy + dy*pdx;//path delta y rotated in path coordinates
                        if((std::abs)((std::atan2)(rpdy,rpdx))>angle_error_max_)continue;//orientation is wrong do not start on this border
                        double lateral_width = std::abs(rpdy*l*0.5+rpdx*b);//use orientation and width,length
                        x.n0_lb_ = adore::mad::bound(-p(3)*0.5,rpy - lateral_width,p(3)*0.5);
                        x.n0_ub_ = adore::mad::bound(-p(3)*0.5,rpy + lateral_width,p(3)*0.5);
                        x.dn0_lb_ = rpdy*v;
                        x.dn0_ub_ = rpdy*v;
                        std::cout<<"pos0=["<<pos0(0)-p(0)<<","<<pos0(1)-p(1)<<"]"<<std::endl;
                        std::cout<<"p0=["<<p0(0)-p(0)<<","<<p0(1)-p(1)<<"], p1=["<<p1(0)-p(0)<<","<<p1(1)-p(1)<<"]"<<std::endl;
                        std::cout<<"rp=["<<rpx<<","<<rpy<<"], rpd=["<<rpdx<<","<<rpdy<<"], dxy=["<<dx<<","<<dy<<"], v="<<v<<std::endl;
                        std::cout<<"n0=["<<x.n0_lb_<<","<<x.n0_ub_<<"], dn0=["<<x.dn0_lb_<<","<<x.dn0_ub_<<"]"<<std::endl;
                        std::cout<<"---"<<std::endl;

                        open.push_back(x);
                    }
                }
                int branch_count = 0;
                //process search states in open (depth first)
                while(open.size()>0)
                {
                    int predictionID = set.size();
                    SearchState x = open.back();
                    open.pop_back();
                    if(closed.find(x.borderID_)!=closed.end())continue;//do not explore twice
                    closed.insert(x.borderID_);
                    //find the center
                    adore::mad::LLinearPiecewiseFunctionM<double,4> center = trafficMap_->getBorderSet()->getCenterline(x.borderID_);
                    //auto lane_width = trafficMap_->getBorderSet()->getLaneWidth(x.borderID_);
                    // if(center.getData().nc()<2)continue;
                    //create a new prediction for the encountered border
                    OccupancyCylinderPrediction prediction;
                    prediction.trackingID_ = p.getTrackingID();
                    prediction.v2xStationID_ = p.getStationID();
                    prediction.branchID_ = predictionID;
                    prediction.predecessorID_ = x.predecessorID_;
                    prediction.confidence_ = 1.0;
                    const double sx0 = x.s0_participant_;
                    const double dsmax = center.limitHi()-x.s0_border_;
                    const double sx1 = std::min(s1,sx0+dsmax);
                    const double s_max_border = center.limitHi();
                    if(center.getData().nc()>=2)for(double s = sx0;s<=sx1;s+=ds)
                    {
                        double t0i,t1i;
                        //first time entering circle with front of vehicle
                        t0i = front_end_s_tva.f(adore::mad::bound(front_end_s_tva.limitLo(),s-ds*0.5,front_end_s_tva.limitHi()))(0);
                        t0i = std::max(t0,t0i);
                        //latest time exiting circle with back of vehicle
                        t1i = rear_end_s_tva.f(adore::mad::bound(rear_end_s_tva.limitLo(),s+ds*0.5,rear_end_s_tva.limitHi()))(0);
                        const double si = s+x.s0_border_-x.s0_participant_;
                        const auto pos0 = center.f(si);
                        double dxi = center.dfidx(si,0);
                        double dyi = center.dfidx(si,1);
                        double scaledxyi = 1.0/std::sqrt(dxi*dxi+dyi*dyi);
                        dxi = dxi * scaledxyi;
                        dyi = dyi * scaledxyi;
                        const double widthi = pos0(3);
                        // maximum between the r_dynamic and r
                        double r_final = lane_width_predictions_?std::max(0.5*widthi,r):r;
                        double c_final = 0.0;
                        if(lateral_predictions_)
                        {
                            double dt = t1i-t0;//latest time arriving at that position
                            double alatmax = 2;
                            double n_ub = adore::mad::bound(-0.5*widthi,x.n0_ub_ + 0.5*alatmax*dt*dt + x.dn0_ub_*dt,0.5*widthi);
                            double n_lb = adore::mad::bound(-0.5*widthi,x.n0_lb_ - 0.5*alatmax*dt*dt + x.dn0_lb_*dt,0.5*widthi);
                            std::cout<<"n("<<dt<<")=["<<n_lb<<","<<n_ub<<"], di=["<<dxi<<","<<dyi<<"]"<<std::endl;
                            c_final = (n_ub+n_lb)*0.5;
                            r_final = std::abs(n_ub-n_lb)*0.5;
                        }
                        if(!std::isnan(r_final)&&!std::isnan(pos0(0))&&!std::isnan(pos0(1))&&!std::isnan(t0)&&!std::isnan(t1))
                        {
                            prediction.occupancy_.insert(adore::mad::OccupancyCylinder(r_final,
                                pos0(0) - dyi * c_final,
                                pos0(1) + dxi * c_final,
                                t0i-time_headway_,
                                t1i+time_leeway_));
                        }
                    }
                    set.push_back(prediction);

                    if(sx1<s1)
                    {
                        //find all successors of the border and add them to the open list 
                        auto border = trafficMap_->getBorderSet()->getBorder(x.borderID_);
                        for(auto it = trafficMap_->getBorderSet()->getSuccessors(border);
                            it.current()!=it.end();
                            it.current()++)
                        {
                            auto nextBorder = it.current()->second;
                            if(border->isContinuousPredecessorOf(nextBorder))
                            {
                                SearchState xnext;
                                xnext.s0_border_ = 0.0;
                                xnext.s0_participant_ = sx1;
                                xnext.predecessorID_ = predictionID;
                                xnext.borderID_ = nextBorder->m_id;
                                xnext.n0_lb_ = x.n0_lb_;
                                xnext.n0_ub_ = x.n0_ub_;
                                xnext.dn0_lb_ = x.dn0_lb_;
                                xnext.dn0_ub_ = x.dn0_ub_;
                                open.push_back(xnext);
                            }

                        }
                    }
                    predictionID++;
                    branch_count++;
                }

                return branch_count>0;
            }


        };
    }
}