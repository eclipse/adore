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
 *   Daniel Heß - collision detection, navigation cost, acceleration costs
 ********************************************************************************/

#pragma once
#include "setpointrequestevaluation.h"
#include <adore/view/athreelaneview.h>
#include <adore/view/atrafficpredictionview.h>
#include <adore/params/ap_tactical_planner.h>
#include <adore/fun/safety/setpointrequestswath.h>
#include <adore/params/afactory.h>


namespace adore
{
    namespace fun
    {
        /**
         * Apply SPRInvariantCollisionFreedom to combined maneuver to test whether future collisions with known obstacles can be prevented
         */
        class SPRInvariantCollisionFreedom: public ASPRConstraint
        {
            private: 
            adore::view::ATrafficPredictionView* predictionView_;
            adore::params::APVehicle* pvehicle_;
            adore::params::APTacticalPlanner* ptac_;
            adore::params::APPrediction* ppred_;
            public:
            SPRInvariantCollisionFreedom(adore::view::ATrafficPredictionView* predictionView, 
                                         adore::params::APVehicle* pvehicle = adore::params::ParamsFactoryInstance::get()->getVehicle(),
                                         adore::params::APTacticalPlanner* ptac = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner(),
                                         adore::params::APPrediction* ppred = adore::params::ParamsFactoryInstance::get()->getPrediction() 
                                         )
                :predictionView_(predictionView),pvehicle_(pvehicle),ptac_(ptac),ppred_(ppred)
                {}
            virtual bool isValid(const adore::fun::SetPointRequest& spr)const override
            {
                double front_buffer_space = ptac_->getCollisionDetectionFrontBufferSpace();
                double lateral_precision = ptac_->getCollisionDetectionLateralPrecision();
                SetPointRequestSwath spr_swath(
                    pvehicle_->get_a()+pvehicle_->get_b()+pvehicle_->get_c()+pvehicle_->get_d()+front_buffer_space,
                    pvehicle_->get_bodyWidth(),
                    pvehicle_->get_d(),//spr reference point at rear axle
                    lateral_precision);
                spr_swath.setLonError(ptac_->getCollisionDetectionLongitudinalError());
                spr_swath.setLatError(ptac_->getCollisionDetectionLateralError());
                spr_swath.setDuration(ppred_->get_roadbased_prediction_duration());
                adore::mad::OccupancyCylinderTree space;
                spr_swath.append_cylinder_swath_linear(spr,space,true);
                return !predictionView_->overlapsEmergencyBehavior(space);
            }
        };

        /**
         * Apply SPRNonCoercive to a setpointrequest to test whether overlap with expected behavior of other traffic participants can be prevented
         */
        class SPRNonCoercive: public ASPRConstraint
        {
            private: 
            adore::view::ATrafficPredictionView* predictionView_;
            adore::params::APVehicle* pvehicle_;
            adore::params::APTacticalPlanner* ptac_;
            adore::params::APPrediction* ppred_;
            public:
            SPRNonCoercive(adore::view::ATrafficPredictionView* predictionView, 
                                         adore::params::APVehicle* pvehicle = adore::params::ParamsFactoryInstance::get()->getVehicle(),
                                         adore::params::APTacticalPlanner* ptac = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner(),
                                         adore::params::APPrediction* ppred = adore::params::ParamsFactoryInstance::get()->getPrediction()
                                         )
                :predictionView_(predictionView),pvehicle_(pvehicle),ptac_(ptac),ppred_(ppred)
                {}
            virtual bool isValid(const adore::fun::SetPointRequest& spr)const override
            {
                double front_buffer_space = ptac_->getCollisionDetectionFrontBufferSpace();
                double lateral_precision = ptac_->getCollisionDetectionLateralPrecision();
                SetPointRequestSwath spr_swath(
                    pvehicle_->get_a()+pvehicle_->get_b()+pvehicle_->get_c()+pvehicle_->get_d()+front_buffer_space,
                    pvehicle_->get_bodyWidth(),
                    pvehicle_->get_d(),//spr reference point at rear axle
                    lateral_precision);
                spr_swath.setLonError(ptac_->getCollisionDetectionLongitudinalError());
                spr_swath.setLatError(ptac_->getCollisionDetectionLateralError());
                spr_swath.setAccelerationErrorSlow(ptac_->getNominalSwathAccelerationError());
                spr_swath.setDuration(ppred_->get_roadbased_prediction_duration());
                adore::mad::OccupancyCylinderTree space;
                spr_swath.append_cylinder_swath_linear(spr,space,true);
                return !predictionView_->overlapsExpectedBehavior(space);
            }
            std::string getName()const{return "coercion";}
        };

        /**
         * Apply SPRTTCNominal to nominal maneuver to test for expected time to collison of nominal maneuver with other objects.
         * The cost value returned is ((t_max-ttc)/t_max)^2 \in [0,1]
         */
        class SPRTTCNominal: public ASPRCost
        {
            private: 
            adore::view::ATrafficPredictionView* predictionView_;
            adore::params::APVehicle* pvehicle_;
            public:
            SPRTTCNominal(adore::view::ATrafficPredictionView* predictionView, 
                            adore::params::APVehicle* pvehicle = adore::params::ParamsFactoryInstance::get()->getVehicle())
                :predictionView_(predictionView),pvehicle_(pvehicle)
                {}
            virtual std::string getName()const override{return "TTC_nom";}
            virtual double getCost(const adore::fun::SetPointRequest& spr)const override
            {
                if(spr.setPoints.size()==0)return 0.0;
                double t0 = spr.setPoints.begin()->tStart;
                double ttc_max = 10.0;
                double tmax = t0 + ttc_max;
                double front_buffer_space = 4.0;
                double lateral_precision = 0.05;
                SetPointRequestSwath spr_swath(
                    pvehicle_->get_a()+pvehicle_->get_b()+pvehicle_->get_c()+pvehicle_->get_d()+front_buffer_space,
                    pvehicle_->get_bodyWidth(),
                    pvehicle_->get_d(),//spr reference point at rear axle
                    lateral_precision);
                spr_swath.setLonError(0.0);
                spr_swath.setLatError(0.0);
                adore::mad::OccupancyCylinderTree space;
                spr_swath.append_cylinder_swath_linear(spr,space);
                double tmin = predictionView_->getExpectedCollisionTime(space,tmax);
                double ttc = std::max(0.0,tmin-t0);
                double cost = (ttc_max-ttc)*(ttc_max-ttc)/ttc_max/ttc_max;
                return cost;
                // return ttc;
            }
            
        };


        /**
         * computes average of lost progress for a plan: lost progress wrt moving at global_speed_limit
         */
        class SPRAverageProgressLoss: public ASPRCost
        {
            private:
            adore::params::APTacticalPlanner* params_;

            public:
            SPRAverageProgressLoss(adore::params::APTacticalPlanner* params):params_(params){}
            virtual std::string getName()const override{return "AverageProgressLoss";}
            virtual double getCost(const adore::fun::SetPointRequest& spr)const override
            {
                const double tmax = 10.0;
                double vmax = std::max(1.0,params_->getGlobalSpeedLimit());
                double distance = 0.0;
                double time = 0.0;
                for(auto& sp:spr.setPoints)
                {
                    double dt = sp.tEnd-sp.tStart;
                    double v = sp.x0ref.getvx();
                    distance += v*dt;
                    time += dt;
                    if(time>tmax)break;
                }
                return std::max(0.0,1.0 - distance/(vmax*tmax));
            }

        };

        /**
         * Apply SPRNavigationCostOnLane to nominal manuever to investigate how good a lane is at navigating towards the goal.
         */
        class SPRNavigationCostOnLane : public ASPRCost
        {
          private:
            adore::view::AThreeLaneView* threeLaneView_;
            adore::params::APTacticalPlanner* params_;
            double max_overshoot_;/**<evaluate navigation cost correctly, even if vehicle has slightly surpassed navigation goal*/
            int laneID_;

          public:
            SPRNavigationCostOnLane(adore::view::AThreeLaneView* threeLaneView,adore::params::APTacticalPlanner* params, int laneID)
                :threeLaneView_(threeLaneView),params_(params),max_overshoot_(50.0),laneID_(laneID)
            {}
            virtual std::string getName()const override{return "MinimumNavigationCostOnLane";}
            virtual double getCost(const adore::fun::SetPointRequest& spr)const override
            {
                try{
                    return nav_cost_of_lane(spr,laneID_);
                }
                catch(const std::exception& e)
                {
                    std::cout<<e.what();
                    // throw(e);
                    return 1.0;
                }
            }
            /**
             * @brief minimum cost of given lane, independent of trajectory
             * @param lane index of lane: 0 current, -1 right, 1 left
             */
            double nav_cost_of_lane(const adore::fun::SetPointRequest& spr,int laneid)const 
            {
                adore::view::ALane* lane=nullptr;
                switch(laneid)
                {
                    case 0:
                        lane = threeLaneView_->getCurrentLane();
                    break;
                    case 1:
                        if(threeLaneView_->getLeftLaneChange()!=nullptr 
                        && threeLaneView_->getLeftLaneChange()->getTargetLane()!=nullptr
                        && threeLaneView_->getLeftLaneChange()->getTargetLane()->isValid())
                        {
                            lane = threeLaneView_->getLeftLaneChange()->getTargetLane();
                        }
                    break;
                    case -1:
                        if(threeLaneView_->getRightLaneChange()!=nullptr 
                        && threeLaneView_->getRightLaneChange()->getTargetLane()!=nullptr
                        && threeLaneView_->getRightLaneChange()->getTargetLane()->isValid())
                        {
                            lane = threeLaneView_->getRightLaneChange()->getTargetLane();
                        }
                    break;
                }
                if(lane==nullptr)return 1.0e99;
                double X0 = spr.setPoints.begin()->x0ref.getX();
                double Y0 = spr.setPoints.begin()->x0ref.getY();
                double s0,n0,p0;
                lane->toRelativeCoordinates(X0,Y0,s0,n0);

                try
                {
                    double interval_start = adore::mad::bound(lane->getSMin(),s0-max_overshoot_,lane->getSMax());
                    double interval_end = lane->getSMax();
                    double tmp;lane->boundNavigationCost(interval_start,interval_end,p0,tmp);
                    return p0;
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
                
                return 1e99;
            }
        };
        /**
         * Apply SPRNormalizedNavigationCost to nominal manuever to investigate how good it is at navigating towards the goal.
         */
        class SPRNormalizedNavigationCost : public ASPRCost
        {
          private:
            adore::view::AThreeLaneView* threeLaneView_;
            adore::params::APTacticalPlanner* params_;
            double max_overshoot_;/**<evaluate navigation cost correctly, even if vehicle has slightly surpassed navigation goal*/
            int laneID_;

          public:
            SPRNormalizedNavigationCost(adore::view::AThreeLaneView* threeLaneView,adore::params::APTacticalPlanner* params,int laneID)
                :threeLaneView_(threeLaneView),params_(params),max_overshoot_(50.0),laneID_(laneID)
            {}
            virtual std::string getName()const override{return "NormalizedNavigationCost";}
            virtual double getCost(const adore::fun::SetPointRequest& spr)const override
            {
                try{
                    return normalized_nav_cost_simple(spr,laneID_);
                }
                catch(const std::exception& e)
                {
                    std::cout<<e.what();
                    // throw(e);
                    return 1.0;
                }
            }

            /**
             * @brief simplified version of normalized_nav_cost: lane is given
             * @param lane index of lane: 0 current, -1 right, 1 left
             */
            double normalized_nav_cost_simple(const adore::fun::SetPointRequest& spr,int laneid)const 
            {
                double fmax = 1.0;
                if(spr.setPoints.size()==0)return fmax;
                double vmax = params_->getGlobalSpeedLimit();
                double amax = params_->getAccLonUB();
                double v0 = std::abs(spr.setPoints.begin()->x0ref.getvx());
                double t0 = spr.setPoints.begin()->tStart;
                double t1 = spr.setPoints.rbegin()->tEnd;
                double dt = t1-t0;
                double ta = std::min(dt,std::abs(vmax-v0)/amax);
                double optimal_progress = v0*ta + 0.5*amax*ta*ta + (dt-ta)*vmax;

                adore::view::ALane* lane=nullptr;
                switch(laneid)
                {
                    case 0:
                        lane = threeLaneView_->getCurrentLane();
                    break;
                    case 1:
                        if(threeLaneView_->getLeftLaneChange()!=nullptr 
                        && threeLaneView_->getLeftLaneChange()->getTargetLane()!=nullptr
                        && threeLaneView_->getLeftLaneChange()->getTargetLane()->isValid())
                        {
                            lane = threeLaneView_->getLeftLaneChange()->getTargetLane();
                        }
                    break;
                    case -1:
                        if(threeLaneView_->getRightLaneChange()!=nullptr 
                        && threeLaneView_->getRightLaneChange()->getTargetLane()!=nullptr
                        && threeLaneView_->getRightLaneChange()->getTargetLane()->isValid())
                        {
                            lane = threeLaneView_->getRightLaneChange()->getTargetLane();
                        }
                    break;
                }
                if(lane==nullptr)return 1.0;
                double X0 = spr.setPoints.begin()->x0ref.getX();
                double Y0 = spr.setPoints.begin()->x0ref.getY();
                double X1 = spr.setPoints.rbegin()->x0ref.getX();
                double Y1 = spr.setPoints.rbegin()->x0ref.getY();
                double s0,n0,s1,n1,p0,p1;
                try
                {
                    lane->toRelativeCoordinates(X0,Y0,s0,n0);
                    lane->toRelativeCoordinates(X1,Y1,s1,n1);
                }
                catch(const std::exception& e)
                {
                    std::cerr << "error in coordinate transformation\n";
                    std::cerr << e.what() << '\n';
                }
                
                try
                {
                    
                    double interval_start = adore::mad::bound(0.0,s0-max_overshoot_,lane->getSMax());
                    double interval_end = adore::mad::bound(0.0,s0,lane->getSMax());
                    double tmp;lane->boundNavigationCost(interval_start,interval_end,p0,tmp);

                    interval_start = adore::mad::bound(0.0,s0-max_overshoot_,lane->getSMax());
                    interval_end = adore::mad::bound(0.0,s1,lane->getSMax());//here: evaluate until s1
                    tmp;lane->boundNavigationCost(interval_start,interval_end,p1,tmp);

                    double actual_progress = std::max(0.0,p0-p1);
                    return adore::mad::bound(0.0,(optimal_progress-actual_progress)/optimal_progress,1.0);
                }
                catch(const std::exception& e)
                {
                    std::cerr << "error in cost bounding\n";
                    std::cerr << e.what() << '\n';
                }
                
                return 1.0;
            }

        };

        class SPRLongitudinalAcceleration2Cost : public ASPRCost
        {
            public:
            virtual std::string getName()const override{return "LongitudinalAccelerationCost";}
            virtual double getCost(const adore::fun::SetPointRequest& spr)const override
            {      
                double value = 0.0;
                for(const auto& setPoint : spr.setPoints)
                {
                    value += (setPoint.tEnd-setPoint.tStart) * setPoint.x0ref.getAx()* setPoint.x0ref.getAx();
                }
                return value; 
            }
        };
        class SPRLongitudinalJerk2Cost : public ASPRCost
        {
            public:
            virtual std::string getName()const override{return "LongitudinalJerkCost";}
            virtual double getCost(const adore::fun::SetPointRequest& spr)const override
            {      
                double value = 0.0;
                for(const auto& setPoint : spr.setPoints)
                {
                    value += (setPoint.tEnd-setPoint.tStart) * setPoint.x0ref.getDAx()* setPoint.x0ref.getDAx();
                }
                return value; 
            }
        };

        class SPRLinearlyWeightedCost : public ASPRCost
        {
          private:
            std::vector<adore::fun::ASPRCost*> c_;
            std::vector<double> w_;

          public:
            void add(double wi,ASPRCost* ci)
            {
                w_.push_back(wi);
                c_.push_back(ci);
            } 
            /**
             * getCost - return w_[0]*c_[0](spr) + ... +w_[end]*c_[end](spr) 
             */
            virtual double getCost(const adore::fun::SetPointRequest& spr)const override
            {       
                double value = 0.0;
                for(unsigned int i=0;i<std::min(c_.size(),w_.size());i++)
                {
                    value += w_[i] * c_[i]->getCost(spr);
                }
                return value;
            } 
            virtual std::string getName()const override{return "weightedSum";}
        };

        /**
         * Prevent restarting, if gained distance is too small
         */
        class RestartEffort: public ASPRConstraint
        {
            private: 
                double big_distance_;
                double small_velocity_;
            public:
            RestartEffort()
            {
                big_distance_ = 5.0;
                small_velocity_ = 1;
            }
            virtual bool isValid(const adore::fun::SetPointRequest& spr)const override
            {
                if(spr.setPoints.size()==0)return false;
                if(spr.setPoints[0].x0ref.getvx()>small_velocity_)return true;
                double ddmax = 0.0;
                double x0 = spr.setPoints[0].x0ref.getX();
                double y0 = spr.setPoints[0].x0ref.getY();
                for(int i=1;i<spr.setPoints.size();i++)
                {
                    double x = spr.setPoints[i].x0ref.getX();
                    double y = spr.setPoints[i].x0ref.getY();
                    ddmax = (std::max)(ddmax,(x-x0)*(x-x0)+(y-y0)*(y-y0));
                }
                return ddmax>big_distance_*big_distance_;
            }
        };

   }
}