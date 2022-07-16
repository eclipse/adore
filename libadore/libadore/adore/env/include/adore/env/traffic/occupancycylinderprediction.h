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
#include <adore/mad/occupancycylinder.h>
#include "participant.h"
#include "velocityprediction.h"

namespace adore
{
    namespace env
    {
        /**
         * Data object for a behavior prediction
         */
        struct OccupancyCylinderPrediction
        {
        public:
            int v2xStationID_;/**< corresponding to TrafficParticipantDetection::v2xStationID*/
            int trackingID_;/**<internal tracking id corresponding to TrafficParticipantDetection::trackingID*/
            int branchID_;/**<different optional behaviors may be predicted - this is an id for the branch, which has to be unique for a single trackingid*/
            int predecessorID_;/**<predecessorID_ references the branchID_ of a predecessor prediction. -1 for no predecessor*/
            adore::mad::OccupancyCylinderTree occupancy_; /**<prediction of occupied space at a given time*/
            float confidence_;/**<the confidence that an agent with trackingID will be enclosed in the given occupancy at future points of time*/
        };

        /**
         * Data object for a set of behavior predictions
         */
        typedef std::vector<OccupancyCylinderPrediction> OccupancyCylinderPredictionSet;

        /**
         * Abstract strategy for prediction of traffic participants, based on "circular" approach
         */
        template <typename T>
        class OCPredictionStrategy
        {
            public:
            /**
             * formulate one or more predictions for Participant p as an OccupancyCylinderTree (a set of cylinders in xyz and t)
             * @param p the participant state information
             * @param set the resulting occupancy prediction is placed in set
             * @return true if prediction is successful
             */
            virtual bool predict(const T& p, OccupancyCylinderPredictionSet& set)const =0;
        };
        /**
         * A simple straight line prediction with given heading and speed.
         * max_time, lat-Precision, lat-error, lon-error, top-speed, maximum acceleration and minimum acceleration can be provided to parametrize prediciton.
         */
        class OCStraightLinePrediction:public OCPredictionStrategy<traffic::Participant>
        {
            private:
                double t_max_utc_;/**< time at which the prediction ends. (prediciton starts at time stamp of object.)*/
                double lat_precision_;/**< maximum lateral overapproximation of the vehicle body*/
                double lat_error_;/**initial (and constant) (measurement) error to the side of the vehicle*/
                double lon_error_;/**initial (measurement) error in movement direction of the vehicle*/
                double v_max_;/**< maximum speed up until which participant could accelerate. only relevant, if a_max>0*/
                double a_max_;/**< maximum acceleration. set a_max_=a_min_ to get no divergence.*/
                double a_min_;/**< minimum acceleration. set a_max_=a_min_ to get no divergence.*/
                double time_headway_;/**< arriving earlier than expected*/
                double time_leeway_;/**< leaving later than expected*/
                double width_ub_;/**< upper bound on prediction width*/
                double width_lb_;/**< lower bound on prediction width*/

            public:
            void setTMaxUTC(double value){t_max_utc_=value;}
            void setLatPrecision(double value){lat_precision_=value;}
            void setLatError(double value){lat_error_=value;}
            void setLonError(double value){lon_error_=value;}
            void setVMax(double value){v_max_=value;}
            void setAMax(double value){a_max_=value;}
            void setAMin(double value){a_min_=value;}
            void setTimeHeadway(double value){time_headway_=value;}
            void setTimeLeeway(double value){time_leeway_=value;}
            void setWidthUB(double value){width_ub_=value;}
            void setWidthLB(double value){width_lb_=value;}
            OCStraightLinePrediction():t_max_utc_(0.0),lat_precision_(0.1),lat_error_(0.0),lon_error_(0.0),v_max_(9999.9),
                                       a_max_(0.0),a_min_(0.0),time_headway_(2.0),time_leeway_(1.0),width_ub_(10.0),width_lb_(0.5){}
            virtual bool predict(const traffic::Participant& p, OccupancyCylinderPredictionSet& set)const override
            {
                double width;
                double length;
                double psi;
                double dx;
                double dy;
                double v = std::sqrt(p.getVx()*p.getVx()+p.getVy()*p.getVy());
                const double form_factor =  std::min(p.getWidth(),p.getLength())/std::max(p.getWidth(),p.getLength());
                const double area = p.getWidth()*p.getLength();
                auto pos0 = p.getCenter();
                const double t0 = p.getObservationTime();
                const double t1 = (std::max)(t0,t_max_utc_);
                //huge objects: usually not on the road
                if(area>100.0 && form_factor>0.8)
                {
                    OccupancyCylinderPrediction prediction;
                    prediction.trackingID_ = p.getTrackingID();
                    prediction.v2xStationID_ = p.getStationID();
                    prediction.branchID_ = set.size();
                    prediction.predecessorID_ = -1;
                    prediction.confidence_ = 0.5;
                    double r = std::sqrt(p.getWidth()*p.getWidth()+p.getLength()*p.getLength())*0.5;
                    prediction.occupancy_.insert(adore::mad::OccupancyCylinder(r,pos0(0),pos0(1),t0,t1));
                    set.push_back(prediction);
                    return true;
                }
                //object orientation correction
                if(p.getWidth()>p.getLength() && v<0.25)
                {
                    width = p.getLength();
                    length = p.getWidth();
                    psi = p.getYaw() + M_PI*0.5;
                    dx = std::cos(psi);
                    dy = std::sin(psi);
                    v = 0.0;
                }
                else
                {
                    length = p.getLength();
                    width = p.getWidth();
                    psi = p.getYaw();
                    const double cospsi = std::cos(psi);
                    const double sinpsi = std::sin(psi);
                    if(v<0.1)
                    {
                        dx = cospsi;
                        dy = sinpsi;
                    }
                    else
                    {
                        dx = cospsi * p.getVx()/v - sinpsi * p.getVy()/v;
                        dy = sinpsi * p.getVx()/v + cospsi * p.getVy()/v;
                    }
                }
                width = adore::mad::bound(width_lb_,width,width_ub_);
                const double b = width*0.5 + lat_error_;
                const double r = b+lat_precision_;
                const double ds = 2.0 * std::sqrt(r*r-b*b);
                const double dt = t1-t0;
                const double l = length + 2.0*lon_error_;
                const double k = (int)std::ceil(l/ds);
                const double s0 = -k*ds*0.5 + 0.5*ds;
                const double t_acc = a_max_>=0.0?dt:std::min((-v/a_max_),dt);
                const double s1 = +k*ds*0.5 + v*dt + 0.5*a_max_*t_acc*t_acc - 0.5*ds;
                adore::mad::LLinearPiecewiseFunctionM<double,3> rear_end_s_tva;
                adore::mad::LLinearPiecewiseFunctionM<double,3> front_end_s_tva;
                RampPrediction ramp;
                ramp.predict_s_tva(-l*0.5,v,a_min_,a_min_<0.0?0.0:v_max_,t0,0.1,t1,rear_end_s_tva);//lower bound: back of behicle with amin
                ramp.predict_s_tva(+l*0.5,v,a_max_,a_max_<0.0?0.0:v_max_,t0,0.1,t1,front_end_s_tva);//upper bound: front of vehicle with amax

                OccupancyCylinderPrediction prediction;
                prediction.trackingID_ = p.getTrackingID();
                prediction.v2xStationID_ = p.getStationID();
                prediction.branchID_ = set.size();
                prediction.predecessorID_ = -1;
                prediction.confidence_ = 1.0;
                for(double s = s0;s<=s1;s+=ds)
                {
                    const double x = pos0(0) + dx * s;
                    const double y = pos0(1) + dy * s;
                    double t0i,t1i;
                    //first time entering circle with front of vehicle
                    t0i = front_end_s_tva.f(adore::mad::bound(front_end_s_tva.limitLo(),s-ds*0.5,front_end_s_tva.limitHi()))(0);
                    t0i -= time_headway_;
                    //latest time exiting circle with back of vehicle
                    t1i = rear_end_s_tva.f(adore::mad::bound(rear_end_s_tva.limitLo(),s+ds*0.5,rear_end_s_tva.limitHi()))(0);
                    t1i += time_leeway_;
                    prediction.occupancy_.insert(adore::mad::OccupancyCylinder(r,x,y,t0i,t1i));
                }
                set.push_back(prediction);
                return true;
            }

        };
    }
}