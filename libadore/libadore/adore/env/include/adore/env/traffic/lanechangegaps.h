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
#include <adore/env/traffic/gapdata.h>
#include <adore/view/alanechangeview.h>
#include <adore/view/trafficobject.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
namespace adore
{
namespace env
{
    /**
     * Interface for gap rating
     */
    class AGapRating
    {
        public:
        virtual void rate(GapData& gap,double s_ego,double ds_ego)=0;
    };
    /**
     * Rates gap according to ego distance to gap
     */
    class GapRating_NearEgo:public AGapRating
    {
        public:
        virtual void rate(GapData& gap,double s_ego,double ds_ego)override
        {
            double ds = adore::mad::bound(gap.s_chase+gap.s_anchor,s_ego,gap.s_lead+gap.s_anchor)-s_ego;
            gap.rating = ds*ds;
            gap.feasible = std::abs(ds)<100.0 && gap.s_chase<gap.s_gate_closure;
        }
    };

    /**
     * LaneChangeGaps computes gaps that are available for a lane change
     */
    class LaneChangeGaps
    {
        private:
        GapQueue data_;
        adore::view::ALaneChangeView* lcv_;
        double anchor_offset_;
        AGapRating* gaprating_;
        public:
        LaneChangeGaps(adore::view::ALaneChangeView* lcv):lcv_(lcv)
        {
            anchor_offset_ = 10.0;
            gaprating_ = new GapRating_NearEgo();
        }
        ~LaneChangeGaps()
        {
            setGapRating(nullptr);
        }
        GapQueue& getData()
        {
            return data_;
        }
        void setGapRating(AGapRating* gaprating)
        {
            if(gaprating_!=nullptr)delete gaprating_;
            gaprating_=gaprating;
        }
        void computeGaps(const adore::view::TrafficQueue& queue,const adore::env::VehicleMotionState9d& ego)
        {
            data_.clear();
            if(lcv_->getSourceLane()==nullptr||!lcv_->getSourceLane()->isValid())return;
            if(lcv_->getTargetLane()==nullptr||!lcv_->getTargetLane()->isValid())return;
            double s_ego,n_ego;
            lcv_->getTargetLane()->toRelativeCoordinates(ego.getX(),ego.getY(),s_ego,n_ego);
            for(int i=0;i<=queue.size();i++)
            {
                const adore::view::TrafficObject* lead = i<queue.size()?&queue[i]:nullptr;
                const adore::view::TrafficObject* chase = i>0?&queue[i-1]:nullptr;
                GapData gap;
                gap.lead_exists = lead!=nullptr;
                gap.chase_exists = chase!=nullptr;

                if(!gap.lead_exists && !gap.chase_exists)//anchor according to ego 
                {
                    gap.s_anchor = adore::mad::bound(lcv_->getProgressOfGateOpen(),s_ego,lcv_->getProgressOfGateClosed());
                    if(gap.s_anchor==s_ego)//if ego is in gate region, a velocity can be assigned
                    {
                        gap.anchor_vt = ego.getvx();
                    }
                    gap.t_obs = ego.getTime();
                    gap.s_lead = 1.0e4;
                    gap.v_lead = 1.0e3;
                    gap.s_chase = -1.0e4;
                    gap.v_chase = 0.0;
                }
                else if(!gap.lead_exists)//anchor to front of chase vehicle
                {
                    gap.s_anchor =  chase->getCurrentProgress() + chase->getLength()*0.5 + anchor_offset_;
                    gap.s_lead = 1.0e4;
                    gap.v_lead = 1.0e3;
                    gap.s_chase =  -anchor_offset_;
                    gap.v_chase = chase->getCurrentSpeed();
                    gap.anchor_vt = chase->getCurrentSpeed();
                    gap.t_obs = chase->getObservationTime();
                }
                else if(!gap.chase_exists)
                {
                    gap.s_anchor =  lead->getCurrentProgress() - lead->getLength()*0.5 - anchor_offset_;
                    gap.s_lead = anchor_offset_;
                    gap.v_lead = lead->getCurrentSpeed();
                    gap.s_chase = -1.0e4;
                    gap.v_chase = 0.0;
                    gap.anchor_vt = lead->getCurrentSpeed();
                    gap.t_obs = lead->getObservationTime();
                }
                else
                {
                    gap.s_anchor = lead->getCurrentProgress()-lead->getLength()*0.5 - anchor_offset_;
                    if(chase->getCurrentProgress()+chase->getLength()*0.5>gap.s_anchor)
                    {
                        gap.s_anchor =  (lead->getCurrentProgress()-lead->getLength()*0.5 
                                       + chase->getCurrentProgress()+chase->getLength()*0.5)*0.5;
                    }
                    gap.s_lead = lead->getCurrentProgress()-lead->getLength()*0.5-gap.s_anchor;
                    gap.s_chase = chase->getCurrentProgress()+chase->getLength()*0.5-gap.s_anchor;
                    gap.v_lead = lead->getCurrentSpeed();
                    gap.v_chase = chase->getCurrentSpeed();
                    gap.anchor_vt = lead->getCurrentSpeed();
                    gap.t_obs = lead->getObservationTime();
                }

                double n = 0.5*(lcv_->getOffsetOfSeparatingBorder(gap.s_anchor)+lcv_->getOffsetOfDestinationOuterBorder(gap.s_anchor));
                lcv_->getTargetLane()->toEucledianCoordinates(gap.s_anchor,n,gap.anchor_X,gap.anchor_Y,gap.anchor_Z);
                double sin = std::sin(lcv_->getTargetLane()->getHeading(gap.s_anchor));
                double cos = std::cos(lcv_->getTargetLane()->getHeading(gap.s_anchor));
                gap.anchor_dX = cos*gap.anchor_vt;
                gap.anchor_dY = sin*gap.anchor_vt;
                gap.s_gate_opening = lcv_->getProgressOfGateOpen()-gap.s_anchor;
                gap.s_gate_closure = lcv_->getProgressOfGateClosed()-gap.s_anchor;
                gaprating_->rate(gap,s_ego,ego.getvx());
                if(gap.feasible)data_.push_back(gap);
            }
        }
    };
}
}