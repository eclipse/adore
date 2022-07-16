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
 *   Thomas Lobig
 ********************************************************************************/
#pragma once
#include <adore/env/borderbased/lanechangeborders.h>
#include <adore/env/borderbased/bordersequence.h>
#include <adore/env/borderbased/baseline.h>

namespace adore
{
namespace env
{
namespace BorderBased
{
/**
 * @brief A class for computation of the geometric information required to perform lane changes.
 * The class name contains the term "independent", as it computes its own smooth baseline function instead of using the smooth baseline function of the lane following geometry.
 * Downside of the independence is an increase computational effort. Upside is an improved projection in cases where target lane steeply separates after the gate.
 * In contrast to the "normal" lc-geometry, which uses a baseline in the center of the source lane, the independent smooth baseline is situated between source and target lane.
 * There is still a minor dependence to the lane following geometry: To compute the lane change geometry, the borders collected by the lane following geometry have to be provided.
 */
class IndependentLaneChangeGeometry
{
    private:
    BorderSet* borderSet_;
    BorderCostMap* borderCostMap_;
    bool valid_;
    double progressGateOpen_;
    double progressGateClosed_;
    public:
    BorderSequence separatingBorders_;  /**<lcl: source-left, gate-end,  target-right */
    BorderSequence sourceOuterBorders_; /**<lcl: source-right,gate-end,  target-right */
    BorderSequence targetOuterBorders_; /**<lcl: source-left, gate-start,target-left  */
    BorderSequence navigationCostBorders_; /**<lcl: source-right, gate-start,target-right  */
    LaneChangeBorders lcb_;
    Baseline baseline_;
    using function_type_scalar = adore::mad::function_type_scalar;
    function_type_scalar offsetSeparatingBorders_;
    function_type_scalar offsetSourceOuterBorders_;
    function_type_scalar offsetTargetOuterBorders_;
    function_type_scalar navigationCost_fct_;
    function_type_scalar speedLimit_fct_;
    bool navigationCost_fcn_available_;
    double maximum_navcost_increase_;
    
    public:
    void setSmoothness(double value)
    {
        baseline_.setSmoothness(value);
    }
    void setMaximumNavCostIncrease(double value)
    {
        maximum_navcost_increase_ = value;
    }
    void setLookAhead(double value)
    {
        baseline_.setLookAhead(value);
        lcb_.setLookAhead(value+100.0);
    }
    void setLookBehind(double value)
    {
        baseline_.setLookBehind(value);
        lcb_.setLookBehind(value+100.0);

    }
    bool isValid()const{return valid_;}
    double getProgressGateOpen()const{return progressGateOpen_;}
    double getProgressGateClosed()const{return progressGateClosed_;}
    bool isNavigationCostFcnAvailable()const{return navigationCost_fcn_available_;}
    function_type_scalar& getLeftOffsetFct()
    {
        return lcb_.isLCDirectionLeft()?offsetTargetOuterBorders_:offsetSourceOuterBorders_;
    }
    function_type_scalar& getRightOffsetFct()
    {
        return lcb_.isLCDirectionLeft()?offsetSourceOuterBorders_:offsetTargetOuterBorders_;
    }
    function_type_scalar& getOffsetSourceOuterBordersFct()
    {
        return offsetSourceOuterBorders_;
    }
    function_type_scalar& getOffsetTargetOuterBordersFct()
    {
        return offsetTargetOuterBorders_;
    }
    IndependentLaneChangeGeometry(bool lc_direction_left,BorderSet* borderSet,BorderCostMap* borderCostMap)
            :lcb_(lc_direction_left,borderSet,borderCostMap),borderSet_(borderSet),borderCostMap_(borderCostMap),valid_(false)
    {
        maximum_navcost_increase_ = 1000.0;
    }
    /**
     * @brief constructs a lane change geometry if a gate is available
     * @param Iterator iterator for std::vector<Border*>, std::list<Border*>, etc.
     * @param position_on_current location of vehicle on current border, used for lookahead and lookbehind measurement
     * @param current_lf iterator pointing to current border in lane following borders
     * @param first_lf start of the lane following borders
     * @param last_lf end of the lane following borders (last valid entry)
     */
    template<typename Iterator>
    void update(double position_on_current,Iterator current_lf, Iterator first_lf, Iterator last_lf)
    {
        valid_ = false;
        separatingBorders_.clear();
        sourceOuterBorders_.clear();
        targetOuterBorders_.clear();
        navigationCostBorders_.clear();
        //compute set of borders relevant for lane change
        lcb_.setConinueOnIncreasingCost(maximum_navcost_increase_>0.0);
        //lcb_.setConinueOnIncreasingCost(false);
        lcb_.update(position_on_current,current_lf,first_lf,last_lf);
        if(!lcb_.isValid())return;
        //assemple border sequences from relevant borders
        if(!collectSeparatingBorders())return;
        if(!collectSourceOuterBorders())return;
        if(!collectTargetOuterBorders())return;
        if(!collectNavigationCostBorders())return;
        //fit baseline
        baseline_.update(separatingBorders_,lcb_.getDistanceToCurrent());
        if(!baseline_.isValid())return;
        baseline_.defineOffset(separatingBorders_,&offsetSeparatingBorders_);
        baseline_.defineOffset(sourceOuterBorders_,&offsetSourceOuterBorders_);
        baseline_.defineOffset(targetOuterBorders_,&offsetTargetOuterBorders_);
        navigationCost_fcn_available_ = computeNavigationCostFunction();
        //compute progress for gate interval
        if(!computeGateInterval_v2())return;
        //some checks
        if(!sanityTest())return;
        if(lcb_.getNavCostIncrease()>maximum_navcost_increase_)return;
        valid_ = true;
    }

    /**
     * @brief a set of tests making sure that lane change geometry is drivable in real conditions
     * @return true if sane
     */
    bool sanityTest()
    {
        if(getMaximumTargetLaneWidth()<1.5)return false;
        return true;
    }

    /**
     * @brief returns the maximum width of the lane change
     */
    double getMaximumTargetLaneWidth()
    {
        double width = 0.0;
        const double sign = lcb_.isLCDirectionLeft()?1.0:-1.0;
        for(double s=baseline_.position_fct_.limitLo();s<baseline_.position_fct_.limitHi();s+=1.0)
        {
            const double outside = offsetTargetOuterBorders_(s);
            const double inside = offsetSeparatingBorders_(s);
            width = std::max(width,sign*(outside-inside));
        }
        return width;
    }

    /**
     * @brief compute gate open and close in the domain of the baseline function
     * @return true if successful
     * @todo This method of computation encounters a problem, when lcbs look further behind than baseline and track circles around. First border of lcb and thus gate open may match to the end of baseline.
     */
    bool computeGateInterval()
    {
        if(lcb_.gate_source_borders_.size()==0)return false;
        if(lcb_.gate_target_borders_.size()==0)return false;
        Coordinate start_coord = lcb_.isLCDirectionLeft()
                                ?lcb_.gate_target_borders_[0]->m_id.m_first
                                :lcb_.gate_source_borders_[0]->m_id.m_first;
        Coordinate end_coord = lcb_.isLCDirectionLeft()
                                ?lcb_.gate_target_borders_.back()->m_id.m_last
                                :lcb_.gate_source_borders_.back()->m_id.m_last;
        double n=0;//not required
        progressGateOpen_ = baseline_.position_fct_.getClosestParameter(start_coord.m_X,start_coord.m_Y,1,2,n);
        progressGateClosed_ = baseline_.position_fct_.getClosestParameter(end_coord.m_X,end_coord.m_Y,1,2,n);
        return true;
    }

    /**
     * @brief compute gate open and close in the domain of the baseline function. this version uses offset functions instead of lcbs.
     * @return true if successful
     */
    bool computeGateInterval_v2()
    {
        if(lcb_.gate_source_borders_.size()==0)return false;
        if(lcb_.gate_target_borders_.size()==0)return false;
        double min_separation = 0.5;
        bool gate_open_found = false;
        for(double s = offsetSeparatingBorders_.limitLo();s<offsetSeparatingBorders_.limitHi();s+=1.0)
        {
            if((std::abs)(offsetSeparatingBorders_(s)-offsetTargetOuterBorders_(s))>min_separation)
            {
                progressGateOpen_ = s;
                gate_open_found = true;
                break;
            }
        }
        Coordinate end_coord = lcb_.isLCDirectionLeft()
                                ?lcb_.gate_target_borders_.back()->m_id.m_last
                                :lcb_.gate_source_borders_.back()->m_id.m_last;
        double n=0;//not required
        progressGateClosed_ = baseline_.position_fct_.getClosestParameter(end_coord.m_X,end_coord.m_Y,1,2,n);
        return gate_open_found;
    }



    /**
     * @brief projects navigation cost of borders in navigationCostBorders_ unto baseline
     */
    bool computeNavigationCostFunction()
    {

        //collect known navigation cost points
        std::vector<adoreMatrix<double,3,1>> navcost_vector;
        for(Border* rb:navigationCostBorders_)
        {
            auto c = borderCostMap_->find(rb->m_id);
            if(c==borderCostMap_->end())return false;
            adoreMatrix<double,3,1> value;
            value(0) = c->second.getDistanceToGoal();
            value(1) = rb->m_id.m_first.m_X;
            value(2) = rb->m_id.m_first.m_Y;
            navcost_vector.push_back(value);
        }
        {
            Border* rb = *navigationCostBorders_.rbegin();
            auto c = borderCostMap_->find(rb->m_id);
            if(c==borderCostMap_->end())return false;
            adoreMatrix<double,3,1> value;
            value(0) = std::max(0.0,c->second.getDistanceToGoal()-rb->getLength());
            value(1) = rb->m_id.m_last.m_X;
            value(2) = rb->m_id.m_last.m_Y;
            navcost_vector.push_back(value);
        }

        //project navigation cost points to baseline
        std::vector<double> svalues;
        std::vector<double> cvalues;
        auto xstart = baseline_.position_fct_(baseline_.position_fct_.limitLo());
        auto xend = baseline_.position_fct_(baseline_.position_fct_.limitHi());
        for(auto& value:navcost_vector)
        {
            double tmp=0;//not required
            double s = baseline_.position_fct_.getClosestParameter(value(1),value(2),1,2,tmp);
            double c;//cost
            if(s==baseline_.position_fct_.limitLo())//potentially located before interval start
            {
                double dx = value(1)-xstart(0);
                double dy = value(2)-xstart(1);
                double d = std::sqrt(dx*dx+dy*dy);
                c = std::max(0.0,value(0) - d);
            }
            else if(s==baseline_.position_fct_.limitHi())//potentially located after interval end
            {
                double dx = value(1)-xend(0);
                double dy = value(2)-xend(1);
                double d = std::sqrt(dx*dx+dy*dy);
                c = std::max(0.0,value(0) + d);
            }
            else
            {  
                c = value(0); 
            }
            if( svalues.size()>0 && s<svalues[svalues.size()-1])continue;
            else if( svalues.size()>0 && s==svalues[svalues.size()-1])
            {
                if(c<cvalues[cvalues.size()-1])
                {
                svalues[svalues.size()-1] = s;
                cvalues[svalues.size()-1] = c;
                }
            }
            else
            {
                svalues.push_back(s);
                cvalues.push_back(c);
            }        
        }
        if(svalues.size()>0)
        {
            navigationCost_fct_.setData(dlib::zeros_matrix<double>(2, svalues.size()));
            for(int i=0;i<svalues.size();i++)
            {
                navigationCost_fct_.getData()(0,i) = svalues[i];
                navigationCost_fct_.getData()(1,i) = cvalues[i];
            }            
            return true;
        }
        else
        {
            navigationCost_fct_.setData(dlib::zeros_matrix<double>(2, 2));
            navigationCost_fct_.getData()(0,0) = baseline_.position_fct_.limitLo();
            navigationCost_fct_.getData()(0,1) = baseline_.position_fct_.limitHi();
            navigationCost_fct_.getData()(1,0) = 1e10;
            navigationCost_fct_.getData()(1,1) = 1e10;
            return true;
        }
        
    }

    /**
     * @brief collects points describing the inner, separating border
     * The separating borders are defined as a sequence of Border objects: If the lane change direction is left,
     * it starts with the lane-following-view left Border objects, otherwise with the right Border Objects.
     * At the end of the gate region, it switches to the target lanes right Border objects for a lane change left
     * and to the left Border objects for a lane change right.
     * @return true if successful
     */
    bool collectSeparatingBorders()
    {
        if(lcb_.isLCDirectionLeft())
        {
            for(Border* rb:lcb_.upstream_borders_)
            {
                Border* lb = borderSet_->getLeftNeighbor(rb);
                if(!lb)return false;
                separatingBorders_.push_back(lb);
            }
            for(Border* rb:lcb_.gate_target_borders_)
            {
                separatingBorders_.push_back(rb);
            }
            for(Border* rb:lcb_.downstream_borders_)
            {
                separatingBorders_.push_back(rb);
            }
        }
        else
        {
            for(Border* rb:lcb_.upstream_borders_)
            {
                separatingBorders_.push_back(rb);
            }
            for(Border* rb:lcb_.gate_source_borders_)
            {
                separatingBorders_.push_back(rb);
            }
            for(Border* rb:lcb_.downstream_borders_)
            {
                Border* lb = borderSet_->getLeftNeighbor(rb);
                if(!lb)return false;
                separatingBorders_.push_back(lb);
            }
        }        
        return true;
    }
    /**
     * @brief collects points describing the target-side outer border
     * @return true if successful
     */
    bool collectTargetOuterBorders()
    {
        if(lcb_.isLCDirectionLeft())
        {
            for(Border* rb:lcb_.upstream_borders_)
            {
                Border* lb = borderSet_->getLeftNeighbor(rb);
                if(!lb)return false;
                targetOuterBorders_.push_back(lb);
            }
            for(Border* rb:lcb_.gate_target_borders_)
            {
                Border* lb = borderSet_->getLeftNeighbor(rb);
                if(!lb)return false;
                targetOuterBorders_.push_back(lb);
            }
            for(Border* rb:lcb_.downstream_borders_)
            {
                Border* lb = borderSet_->getLeftNeighbor(rb);
                if(!lb)return false;
                targetOuterBorders_.push_back(lb);
            }
        }
        else
        {
            for(Border* rb:lcb_.upstream_borders_)
            {
                targetOuterBorders_.push_back(rb);
            }
            for(Border* rb:lcb_.gate_target_borders_)
            {
                targetOuterBorders_.push_back(rb);
            }
            for(Border* rb:lcb_.downstream_borders_)
            {
                targetOuterBorders_.push_back(rb);
            }
        }
        return true;
    }
    /**
     * @brief collects points describing the source-side outer border
     * @return true if successful
     */
    bool collectSourceOuterBorders()
    {
        if(lcb_.isLCDirectionLeft())
        {
            for(Border* rb:lcb_.upstream_borders_)
            {
                sourceOuterBorders_.push_back(rb);
            }
            for(Border* rb:lcb_.gate_source_borders_)
            {
                sourceOuterBorders_.push_back(rb);
            }
            for(Border* rb:lcb_.downstream_borders_)
            {
                sourceOuterBorders_.push_back(rb);
            }
        }
        else
        {
            for(Border* rb:lcb_.upstream_borders_)
            {
                Border* lb = borderSet_->getLeftNeighbor(rb);
                if(!lb)return false;
                sourceOuterBorders_.push_back(lb);
            }
            for(Border* rb:lcb_.gate_source_borders_)
            {
                Border* lb = borderSet_->getLeftNeighbor(rb);
                if(!lb)return false;
                sourceOuterBorders_.push_back(lb);
            }
            for(Border* rb:lcb_.downstream_borders_)
            {
                Border* lb = borderSet_->getLeftNeighbor(rb);
                if(!lb)return false;
                sourceOuterBorders_.push_back(lb);
            }
        }
        return true;
    }
    /**
     * @brief collects points describing the navigation cost reference borders
     * @return true if successful
     */
    bool collectNavigationCostBorders()
    {
        if(lcb_.isLCDirectionLeft())
        {
            for(Border* rb:lcb_.gate_target_borders_)
            {
                navigationCostBorders_.push_back(rb);
            }
            for(Border* rb:lcb_.downstream_borders_)
            {
                navigationCostBorders_.push_back(rb);
            }
        }
        else
        {
            for(Border* rb:lcb_.gate_target_borders_)
            {
                navigationCostBorders_.push_back(rb);
            }
            for(Border* rb:lcb_.downstream_borders_)
            {
                navigationCostBorders_.push_back(rb);
            }
        }
        return true;
    }
};
}
}
}