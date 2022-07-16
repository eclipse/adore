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
            adore::env::traffic::TrafficMap* trafficMap_;/**< a data object, which allows to map between borders and traffic participants*/
            struct SearchState
            {
                public:
                double s0_border_;       /**< where to start on border */
                double s0_participant_;  /**< how many m has the participant progressed, when it starts on border*/
                adore::env::BorderBased::BorderID borderID_; /**< the border to be considered*/
                int predecessorID_;      /**< predecessor branch id*/
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
            OCRoadBasedPrediction(adore::env::traffic::TrafficMap* trafficMap)
                :t_max_utc_(0.0),lat_precision_(0.2),lat_error_(0.0),lon_error_(0.0),
                 v_max_(9999.9),a_max_(0.0),a_min_(0.0),
                 time_headway_(2.0),
                 time_leeway_(1.0),
                 trafficMap_(trafficMap){}
            virtual bool predict(const traffic::Participant& p, OccupancyCylinderPredictionSet& set)const override
            {
                const double b = p.getWidth()*0.5 + lat_error_;
                const double r = b+lat_precision_;
                const double ds = 2.0 * std::sqrt(r*r-b*b);
                const double t0 = p.getObservationTime();
                const double t1 = (std::max)(t0,t_max_utc_);
                const double dt = t1-t0;
                const double v = std::sqrt(p.getVx()*p.getVx()+p.getVy()*p.getVy());
                const double l = p.getLength() + 2.0*lon_error_;
                const double k = (int)std::ceil(l/ds);
                const double s0 = -k*ds*0.5 + 0.5*ds;
                const double s1 = +k*ds*0.5 + v*dt + 0.5*a_max_*dt*dt - 0.5*ds;
                const double dx = std::cos(p.getYaw());
                const double dy = std::sin(p.getYaw());
                auto pos0 = p.getCenter();
                adore::mad::LLinearPiecewiseFunctionM<double,3> rear_end_s_tva;
                adore::mad::LLinearPiecewiseFunctionM<double,3> front_end_s_tva;
                RampPrediction ramp;
                ramp.predict_s_tva(-l*0.5,v,a_min_,a_min_<0.0?0.0:v_max_,t0,0.1,t1,rear_end_s_tva);//lower bound: back of behicle with amin
                ramp.predict_s_tva(+l*0.5,v,a_max_,a_max_<0.0?0.0:v_max_,t0,0.1,t1,front_end_s_tva);//upper bound: front of vehicle with amax

                //prediction for current shape and position
                {
                    OccupancyCylinderPrediction prediction;
                    int predictionID = set.size();
                    adore::mad::LLinearPiecewiseFunctionM<double,3> center(2,0.0);
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
                    prediction.branchID_ = predictionID;
                    prediction.predecessorID_ = -1;
                    prediction.confidence_ = 1.0;
                    for(double s = ds*0.5;s<=l-ds*0.5;s+=ds)
                    {
                        auto pos = center.f(s);
                        double t0i,t1i;
                        //first time entering circle with front of vehicle
                        t0i = front_end_s_tva.f(adore::mad::bound(front_end_s_tva.limitLo(),s-ds*0.5,front_end_s_tva.limitHi()))(0);
                        t0i -= time_headway_;
                        //latest time exiting circle with back of vehicle
                        t1i = rear_end_s_tva.f(adore::mad::bound(rear_end_s_tva.limitLo(),s+ds*0.5,rear_end_s_tva.limitHi()))(0);
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
                        SearchState x;
                        x.s0_border_ = positioning.s;
                        x.s0_participant_ = 0.0;
                        x.predecessorID_ = -1;
                        x.borderID_ = borderID;
                        //angle offset
                        //find two points on right border
                        auto bright = trafficMap_->getBorderSet()->getBorder(borderID);
                        if(bright==nullptr)continue;//no border found: prediction impossible
                        //compute orientation information:
                        const double lah = (std::max)(1.0,l*0.5);//look ahead/behind
                        if(positioning.s>=bright->m_path->limitHi()-0.1)continue;
                        const auto p0 = bright->m_path->f_bounded(positioning.s-lah);
                        const auto p1 = bright->m_path->f_bounded(positioning.s+lah);
                        const double pdx = p1(0)-p0(0); //path delta x
                        const double pdy = p1(1)-p0(1); //path delta y
                        if(pdx*pdx+pdy*pdy<0.01)continue;//very short border?
                        const double rpdx =  pdx*dx + pdy*dy;//path delta x rotated in vehicle coordinates
                        const double rpdy = -pdx*dy + pdy*dx;//path delta y rotated in vehicle coordinates
                        if((std::abs)((std::atan2)(rpdy,rpdx))>angle_error_max_)continue;//orientation is wrong do not start on this border
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
                    adore::mad::LLinearPiecewiseFunctionM<double,3> center = trafficMap_->getBorderSet()->getCenterline(x.borderID_);
                    auto right_border = trafficMap_->getBorderSet()->getBorder(x.borderID_);
                    adore::mad::LLinearPiecewiseFunctionM<double,3>* right = right_border->m_path;
                    // if(center.getData().nc()<2)continue;
                    //create a new prediction for the encountered border
                    OccupancyCylinderPrediction prediction;
                    prediction.trackingID_ = p.getTrackingID();
                    prediction.branchID_ = predictionID;
                    prediction.predecessorID_ = x.predecessorID_;
                    prediction.confidence_ = 1.0;
                    const double sx0 = x.s0_participant_;
                    const double dsmax = center.limitHi()-x.s0_border_;
                    const double sx1 = std::min(s1,sx0+dsmax);
                    const double s_max_border = center.limitHi();
                    if(center.getData().nc()>=2)for(double s = sx0;s<=sx1;s+=ds)
                    {
                        auto pos = center.f(s+x.s0_border_-x.s0_participant_);
                        auto pos_right = right->f(s+x.s0_border_-x.s0_participant_);
                        double r_dynamic = sqrt((pos(0)-pos_right(0))*(pos(0)-pos_right(0)) + (pos(1)-pos_right(1))*(pos(1)-pos_right(1)));
                        
                        // maximum between the r_dynamic and r
                        
                        double t0i,t1i;
                        //first time entering circle with front of vehicle
                        t0i = front_end_s_tva.f(adore::mad::bound(front_end_s_tva.limitLo(),s-ds*0.5,front_end_s_tva.limitHi()))(0);
                        t0i -= time_headway_;
                        //latest time exiting circle with back of vehicle
                        t1i = rear_end_s_tva.f(adore::mad::bound(rear_end_s_tva.limitLo(),s+ds*0.5,rear_end_s_tva.limitHi()))(0);
                        t1i += time_leeway_;
                        if(!std::isnan(r_dynamic)&&!std::isnan(pos(0))&&!std::isnan(pos(1))&&!std::isnan(t0)&&!std::isnan(t1))
                        {
                            prediction.occupancy_.insert(adore::mad::OccupancyCylinder(r_dynamic,pos(0),pos(1),t0i,t1i));
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