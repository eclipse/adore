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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/env/afactory.h>
#include <adore/view/athreelaneview.h>
#include <adore/env/traffic/participant.h>
#include <adore/env/traffic/trafficqueueonalane.h>
#include <adore/env/borderbased/lanegeometrydataproxy.h>

namespace adore
{
    namespace env
    {
        /**
         * ThreeLaneViewDecoupled - Realization of AThreeLaneView interface, which makes use of decoupled lane geometry computation.
         * The lane geometry for lane following and lane change is asynchronously computed in a different instance, (possibly at lower rate).
         * Traffic data is received and matched to the geometries by this class.
         */
        class ThreeLaneViewDecoupled: public adore::view::AThreeLaneView
        {
            public:
                class LaneProxy: public adore::view::ALane
                {
                    public:
                    adore::env::BorderBased::LaneGeometryDataProxy data_;
                    adore::env::traffic::TrafficQueueOnALane queueOnThisLane_;
                    double s_offset_;
                    LaneProxy():s_offset_(0.0){}
                    double limitS(double s)
                    {
                        s-=s_offset_;
                        return adore::mad::bound(
                            data_.centerSmoothed_fct.limitLo(),
                            s+data_.centerSmoothed_fct.limitLo(),
                            data_.centerSmoothed_fct.limitHi());
                    }
                    /**
                     * isValid - return true if representation of lane is valid
                     */
                    virtual bool isValid() const override
                    {
                        return data_.isValid;
                    }
                    /**
                     * @return the maximum s parameter of the relative coordinate system
                     */
                    virtual double getSMax()const override
                    {
                        return data_.centerSmoothed_fct.limitHi() - data_.centerSmoothed_fct.limitLo() + s_offset_;
                    }
                    /**
                     * @return the minimum s parameter of the relative coordinate system
                     */
                    virtual double getSMin()const override
                    {
                        return +s_offset_;
                    }
                    /**
                     *  adapts s_offset_ of this object in such a way that an s value refers to adjacent points on this and master
                     */
                    bool syncS(ALane* master)
                    {
                        s_offset_ = 0.0;
                        if(!isValid()||!master->isValid())return false;
                        double X,Y,Z,sout,tmp;
                        for(double s = master->getSMin();s<master->getSMax();s+=1.0)
                        {
                            master->toEucledianCoordinates(s,0.0,X,Y,Z);
                            this->toRelativeCoordinates(X,Y,sout,tmp);
                            if(sout>=getSMax())return false;
                            if(sout>getSMin())
                            {
                                s_offset_ = s-sout;
                                return true;
                            }
                        }
                        return false;
                    }
                    /**
                     * getProgressOfWidthOpen - returns s-coordinate of the position where the lane starts to have at least the required
                     * width
                     */
                    double getProgressOfWidthOpen() const
                    {
                        return data_.centerSmoothed_fct.limitLo();
                    }
                    /**
                     * getProgressOfWidthClosed - returns s-coordinate of the position where the lane ends to have at least the required
                     * width
                     */
                    double getProgressOfWidthClosed() const
                    {
                        return data_.centerSmoothed_fct.limitHi();
                    }
                    /**
                     * getOnLaneTraffic - return queue of traffic objects moving on lane, ordered by progress of objects on lane
                     */
                    virtual const adore::view::TrafficQueue& getOnLaneTraffic()const  override
                    {
                        return queueOnThisLane_.getQueue();
                    }
                    /**
                     * getConflictSet - return set of conflict zones, ordered by occurance along lane
                     */
                    virtual const adore::view::ConflictSet& getConflictSet()const  override
                    {
                        throw std::logic_error("Not implemented");
                    }
                    /**
                     * getSpeedLimit - return the speed limit at a certain distance s along the lane
                     */
                    virtual double getSpeedLimit(double s) override
                    {
                        return data_.speedLimit_fct(limitS(s));
                    }
                    /**
                     * * getLeftIndicatorHint - return left indicator light hint at s along the lane
                     */
                    double getLeftIndicatorHint(double s) override
                    {
                        return data_.left_indicator_hint_fct(limitS(s));
                    }
                    /**
                     * * getRightIndicatorHint - return right indicator light hint at s along the lane
                     */
                    double getRightIndicatorHint(double s) override
                    {
                        return data_.right_indicator_hint_fct(limitS(s));
                    }
                    /**
                     * hasSpeedRecommendation - return true, if a speed recommendation is available (GLOSA or other infrastructure advice) at a certain distance s along the lane
                     */
                    virtual bool hasSpeedRecommendation(double s)const  override
                    {
                        return false;
                    }
                    /**
                     * getSpeedRecommendation - return a speed recommendation at a certain distance s along the lane
                     */
                    virtual double getSpeedRecommendation(double s)const override
                    {
                        throw std::logic_error("Not implemented");
                        return 0.0;
                    }
                    virtual double getNavigationCost(double s) override
                    {
                        if(data_.navigationCost_fct.getData().nc()<2)return 1.0e10;  
                        return data_.navigationCost_fct.f_bounded(limitS(s));
                    }
                    /**
                     * boundNavigationCost - return bounds for navigation-cost on a distance s interval along the lane
                     * @param cmin the minimum cost on the interval
                     * @param cmax the maximum cost on the interval
                     */
                    virtual void boundNavigationCost(double s0,double s1,double& cmin,double& cmax) override
                    {
                        if(data_.navigationCost_fct.getData().nc()<2)
                        {
                            cmin = 1.0e10;
                            cmax = 1.0e10;
                            return;
                        }   
                        adoreMatrix<double,1,1> y0,y1;
                        s0 = limitS(s0);
                        s1 = limitS(s1);
                        s0 = std::max(data_.navigationCost_fct.limitLo(),s0);
                        s1=  std::min(data_.navigationCost_fct.limitHi(),s1);
                        data_.navigationCost_fct.bound(s0,s1,y0,y1);
                        cmin = y0;cmax = y1;
                    }

                    /**
                     *  getHeading - return the heading of the lane at a distance s along the lane
                     */
                    virtual double getHeading(double s) override
                    {
                        // return data_.centerHeading_fct(limitS(s));
                        auto n = data_.centerNormal_fct(limitS(s));
                        // return std::atan2(n(1),n(0))-M_PI*0.5;
                        return std::atan2(-n(0),n(1));
                    }
                    /**
                     * getCurvature - return the lane coordinate system's curvature kappa=1/R and its derivatives 1,2,... at a progress s
                     * the derivative is given as 1: d kappa / ds, 2: d^2 kappa / ds^2, ...
                     * if a derivative is unavailable, 0 will be returned
                     */
                    virtual double getCurvature(double s, int derivative)   override
                    {
                        if (derivative <= 0)
                        {
                            return data_.centerSmoothedCurvature_fct(limitS(s));
                        }
                        else if (derivative <= 1)
                        {
                            return data_.centerSmoothedCurvatureDerivative_fct(limitS(s));
                        }
                        else
                        {
                            return 0.0;
                        }
                    }
                    /**
                     * getOffsetOfLeftBorder - return the lateral offset of the left border at a progress s
                     */
                    virtual double getOffsetOfLeftBorder(double s)   override
                    {
                        return data_.leftDistance_fct(limitS(s));
                    }
                    /**
                     * getOffsetOfRightBorder - return the lateral offset of the right border at a progress s
                     */
                    virtual double getOffsetOfRightBorder(double s)   override
                    {
                        return data_.rightDistance_fct(limitS(s));
                    }
                    /**
                     * coordinate transformation from euclidean (xe,ye) to road relative coordinates (s,n)
                     */
                    virtual void toRelativeCoordinates(double xe,double ye,double& s,double& n)   override
                    {
                        // if(!adore::mad::toRelativeWithNormalExtrapolation(xe,ye,&data_.centerSmoothed_fct,&data_.centerNormal_fct,s,n))
                        {
                            s = data_.centerSmoothed_fct.getClosestParameter(xe, ye, 1, 2, n);
                        }
                        s+=-data_.centerSmoothed_fct.limitLo()+s_offset_;
                    }
                    /**
                     * coordinate transformation from road relative coordinates (s,n) to euclidean (xe,ye,ze) 
                     */
                    virtual void toEucledianCoordinates(double s,double n,double& xe,double& ye,double& ze)   override
                    {
                        s = limitS(s);
                        adore::mad::fromRelative(s,n,&data_.centerSmoothed_fct,&data_.centerNormal_fct,xe,ye,ze);
                        ze=0.0;
                    }
                    
                };

                class LaneChangeViewProxy:public adore::view::ALaneChangeView
                {
                    public:
                    LaneProxy* source_;
                    LaneProxy* target_;
                    adore::env::BorderBased::LaneChangeDataProxy data_;
                    double limitS(double s)
                    {
                        return target_->limitS(s);
                    }
                    LaneChangeViewProxy():source_(nullptr),target_(nullptr){}
                    void setLanes(LaneProxy* source,LaneProxy* target)
                    {
                        source_=source;
                        target_=target;
                    }
                    virtual adore::view::ALane* getSourceLane() override
                    {
                        return source_;
                    }
                    /**
                     * getSourceLane - return ALane pointer for the target lane of the lane change
                     */
                    virtual adore::view::ALane* getTargetLane() override
                    {
                        return target_;
                    }
                    /**
                     * getLCDirection - return the direction of a lane change leading to target lane
                     */
                    virtual adore::view::ALaneChangeView::direction getLCDirection() const override
                    {
                        return data_.direction;
                    }
                    /**
                     * getProgressOfGateOpen - return progress s of the next opening of a gate (distance to end of solid
                     * line or otherwise impassable lane border)
                     */
                    virtual double getProgressOfGateOpen() const override
                    {
                        return data_.gate_s0 + target_->getSMin();
                    }
                    /**
                     * getProgressOfGateClosed - return progress s of the closure of the next gate (distance to beginngin of
                     * solid line or otherwise impassable lane border after gate)
                     */
                    virtual double getProgressOfGateClosed() const override
                    {
                        return data_.gate_s1 + target_->getSMin();
                    }
                    /**
                     * getOffsetOfStartOuterBorder - return lateral offset n of the outer border of the AV's current lane
                     */
                    virtual double getOffsetOfStartOuterBorder(double s)  override
                    {
 
                        return data_.sourceOuterBorderDistance_fct.f(limitS(s));
                    }
                    /**
                     * getOffsetOfStartInnerBorder - return lateral offset n of the inner border of the AV's current lane
                     */
                    virtual double getOffsetOfSeparatingBorder(double s) override
                    {
                        return data_.separatingBorderDistance_fct.f(limitS(s));
                    }
                    /**
                     * getOffsetOfDestinationOuterBorder - return lateral offset n of the outer border of target lane
                     */
                    virtual double getOffsetOfDestinationOuterBorder(double s) override
                    {
                        return data_.targetOuterBorderDistance_fct.f(limitS(s));
                    }

                    /**
                     * @return the difference in navigation cost between two lanes: g_target-g_source; if the return value is below 0, the target lane has lower navigation cost 
                     */
                    virtual double getNavigationCostDifference() override
                    {
                        double smin_target,smax_target,smin_source,smax_source;
                        target_->boundNavigationCost(target_->getSMin(),target_->getSMax(),smin_target,smax_target);
                        source_->boundNavigationCost(source_->getSMin(),source_->getSMax(),smin_source,smax_source);
                        return smin_target-smin_source;
                        // double s = (getProgressOfGateOpen() + getProgressOfGateClosed())*0.5;
                        // return target_->getNavigationCost(s) - source_->getNavigationCost(s);
                    }


                };

            private:
                adore::env::AFactory::TLaneGeometryFeed* laneFeed_;/**< data source for lane geometry */
                adore::env::BorderBased::BorderSet* borderSet_; /**< set of borders */
                adore::env::AFactory::TParticipantSetReader* tpsetReader_; /**< data source for traffic participants */
                LaneProxy left_;/**< proxy representing ALane interface for left lane*/
                LaneProxy center_;/**< proxy representing ALane interface for center lane*/
                LaneProxy right_;/**< proxy representing ALane interface for right lane*/
                LaneChangeViewProxy to_left_;/**< proxy representing ALaneChangeView interface for left lane*/
                LaneChangeViewProxy to_right_;/**< proxy representing ALaneChangeView interface for right lane*/
                adore::env::traffic::TParticipantSet participantSet_; /**< set of participants: local buffer */
                bool monitor_traffic_;/**< indicates whether traffic is processed*/

            public:
            ThreeLaneViewDecoupled(bool monitor_traffic=true):monitor_traffic_(monitor_traffic)
            {
                laneFeed_ = adore::env::EnvFactoryInstance::get()->getLaneGeometryFeed();
                if(monitor_traffic_)
                {
                    tpsetReader_ = adore::env::EnvFactoryInstance::get()->getTrafficParticipantSetReader();
                }
                else
                {
                    tpsetReader_ = nullptr;
                }
                
                to_left_.setLanes(&center_,&left_);
                to_right_.setLanes(&center_,&right_);
            }
            virtual ~ThreeLaneViewDecoupled()
            {
                delete laneFeed_;
                if(tpsetReader_!=nullptr)delete tpsetReader_;
            }
            void update()
            {
                bool recompute_traffic_queues = false;
                if (laneFeed_->hasNext())
                {
                    adore::env::BorderBased::CombinedLaneGeometry combinedGeometry;
                    laneFeed_->getLatest(combinedGeometry);
                    center_.data_ = *combinedGeometry.center;
                    left_.data_ = *combinedGeometry.left;
                    right_.data_ = *combinedGeometry.right;
                    to_left_.data_ = *combinedGeometry.leftChange;
                    to_right_.data_ = *combinedGeometry.rightChange;
                    recompute_traffic_queues = true;
                    left_.syncS(&center_);
                    right_.syncS(&center_);
                }
                if( monitor_traffic_ && tpsetReader_->hasData() )
                {
                    tpsetReader_->getData(participantSet_);
                    recompute_traffic_queues = true;
                }
                if(recompute_traffic_queues)
                {
                    left_.queueOnThisLane_.mapVehicles(&left_,&participantSet_);
                    center_.queueOnThisLane_.mapVehicles(&center_,&participantSet_);
                    right_.queueOnThisLane_.mapVehicles(&right_,&participantSet_);
                }
            }

            /**
             * getCurrentLane - return ALane pointer the vehicle's current lane
             */
            virtual adore::view::ALane* getCurrentLane()
            {
                return &center_;
            }
            /**
             * getLeftLaneChange - returns ALaneChangeView pointer to the lane left of the vehicle's current lane.
             */
            virtual adore::view::ALaneChangeView* getLeftLaneChange()
            {
                return &to_left_;
            }
            /**
             * getRightLaneChange - returns ALaneChangeView pointer to the lane right of the vehicle's current lane.
             */
            virtual adore::view::ALaneChangeView* getRightLaneChange()
            {
                return &to_right_;
            }
        };
    }
}
