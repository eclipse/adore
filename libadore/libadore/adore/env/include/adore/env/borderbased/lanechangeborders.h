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
#include <adore/env/borderbased/borderaccumulator.h>
#include <adore/env/borderbased/bordercostmap.h>
#include <adore/env/borderbased/borderset.h>
#include <vector>

namespace adore
{
namespace env
{
namespace BorderBased
{
/**
 * @brief Selects Borders from BorderSet required for LaneChangeView construction
 */
class LaneChangeBorders
{
    private:
    BorderSet* borderSet_;
    BorderCostMap* borderCostMap_;
    bool lc_direction_left_;                   /**<lane change direction: true if going left*/
    double lookahead_;
    double lookbehind_;
    double progress_in_gate_;
    double remaining_in_gate_;
    double downstream_length_;
    double distance_to_current_;
    bool valid_;
    bool continueOnIncreasingCost_;
    public:
    std::vector<Border*> gate_target_borders_; /**< borders on target lane in region of gate */
    std::vector<Border*> gate_source_borders_; /**< borders on source lane in region of gate */
    std::vector<Border*> downstream_borders_;  /**< borders on target lane after gate region */
    std::vector<Border*> upstream_borders_;    /**< borders on source lane before gate region */
    public:
    LaneChangeBorders(bool lc_direction_left,BorderSet* borderSet,BorderCostMap* borderCostMap)
        :lc_direction_left_(lc_direction_left),borderSet_(borderSet),borderCostMap_(borderCostMap),
        valid_(false)
    {
        lookahead_ = 100.0;
        lookbehind_ = 100.0;
        continueOnIncreasingCost_ = false;
    }
    bool isLCDirectionLeft()
    {
        return lc_direction_left_;
    }
    bool isValid()
    {
        return valid_;
    }
    double getProgressInGate()
    {
        return progress_in_gate_;
    }
    double getRemainingInGate()
    {
        return remaining_in_gate_;
    }
    double getDownstreamLength()
    {
        return downstream_length_;
    }
    double getDistanceToCurrent()
    {
        return distance_to_current_;
    }
    double getLookAhead()
    {
        return lookahead_;
    }
    void setLookAhead(double value)
    {
        lookahead_ = value;
    }
    double getLookBehind()
    {
        return lookbehind_;
    }
    void setLookBehind(double value)
    {
        lookbehind_ = value;
    }
    void setConinueOnIncreasingCost(bool value)
    {
        continueOnIncreasingCost_ = value;
    }
    double getNavCostIncrease()
    {
        //query navigation cost at end of gate
        auto q0 = borderCostMap_->find((*gate_source_borders_.rbegin())->m_id);
        auto q1 = borderCostMap_->find((*gate_target_borders_.rbegin())->m_id);
        adore::env::NavigationCost cost0(adore::env::NavigationCost::maximum_cost());
        if(!(q0==borderCostMap_->end()))
        {
            cost0 = q0->second;
        }        
        adore::env::NavigationCost cost1(adore::env::NavigationCost::maximum_cost());
        if(!(q1==borderCostMap_->end()))
        {
            cost1 = q1->second;
        }        
        return cost1.getCombinedCost()-cost0.getCombinedCost(); 
    }
    /**
     * @brief collects all borders relevant for lane change view in object-variable vectors
     * @param Iterator iterator for std::vector<Border*>, std::list<Border*>, etc.
     * @param position_on_current location of vehicle on current border, used for lookahead and lookbehind measurement
     * @param current_lf iterator pointing to current border in lane following borders
     * @param first_lf start of the lane following borders
     * @param last_lf end of the lane following borders (last valid entry)
     */
    template<typename Iterator>
    void update(double position_on_current,Iterator current_lf, Iterator first_lf, Iterator last_lf)
    {
        gate_target_borders_.clear();
        gate_source_borders_.clear();
        downstream_borders_.clear();
        upstream_borders_.clear();
        progress_in_gate_ = 0.0;
        remaining_in_gate_ = 0.0;
        downstream_length_ = 0.0;
        distance_to_current_ = 0.0;
        valid_ = true;

        // std::cout<<"lane change view "<<(lc_direction_left_?"left":"right")<<":\n";

        auto gate_region = getGateRegion(current_lf, first_lf, last_lf);
        if(gate_region.first==std::next(last_lf)||gate_region.second==std::next(last_lf))return;//no region has been found: done
        
        //measure distances
        double d = 0.0;
        for(Iterator it = gate_region.first;it!=std::next(gate_region.second);it++)
        {
            if(it==current_lf)
            {
                progress_in_gate_ = position_on_current + d;
            }
            d += (*it)->getLength();
            if(it==gate_region.second)
            {
                remaining_in_gate_ = d - progress_in_gate_;
            }
        }
        d = 0.0;
        for(Iterator it = first_lf;it!=std::next(last_lf);it++)
        {
            if(it==current_lf)break;
            d += (*it)->getLength();
        }
        distance_to_current_ = d + position_on_current;
        // std::cout<<"position_on_current="<<position_on_current<<std::endl;
        // std::cout<<"progress_in_gate_="<<progress_in_gate_<<std::endl;
        // std::cout<<"remaining_in_gate_="<<remaining_in_gate_<<std::endl;


        int upstream_count = 0;
        int gate_source_count = 0;
        int gate_target_count = 0;
        int downstream_count = 0;
        //collect borders: upstream/source
        for(Iterator it = first_lf;it!=gate_region.first;it++)
        {
            upstream_borders_.push_back(*it);
            upstream_count ++;
        }
        //collect borders: gate/source+target
        for(Iterator it = gate_region.first;it!=std::next(gate_region.second);it++)
        {
            gate_source_borders_.push_back(*it);
            gate_source_count ++;
            auto adjacent_lane = borderSet_->getLaneChangeTarget(*it,lc_direction_left_);
            if(adjacent_lane.first==nullptr||adjacent_lane.second==nullptr)return;
            gate_target_borders_.push_back(adjacent_lane.second);
            gate_target_count ++;
        }
        //collect borders: downstream/target
        double max_distance = (std::max)(0.0,lookahead_ - remaining_in_gate_ + gate_target_borders_.back()->getLength());
        BASFollowNavigation bas(gate_target_borders_.back(), borderSet_, borderCostMap_, max_distance );
        bas.setContinueOnIncreasingCost(continueOnIncreasingCost_);
        Border* b;
        bool inverted = false;
        bas.getNextBorder(b,inverted);
        for(bas.getNextBorder(b,inverted);b!=nullptr;bas.getNextBorder(b,inverted))
        {
            downstream_borders_.push_back(b);
            downstream_length_ += b->getLength();
            downstream_count ++;
        }
        // std::cout<<"upstream_count="<<upstream_count<<std::endl;
        // std::cout<<"gate_source_count="<<gate_source_count<<std::endl;
        // std::cout<<"gate_target_count="<<gate_target_count<<std::endl;
        // std::cout<<"downstream_count="<<downstream_count<<std::endl;
        valid_ = true;
    }

    /**
     * @brief computes the best gate entry point for a lane change
     * Supply iterators for lists or vectors of Border*, which describe the current lane.
     * The area given by the iterators is searched for an entry point.
     * The function iterates from first_lf to last_lf through the borders of the current lane and tries to find an
     * adjacent border, which has minimum (navigation) cost.
     * @param Iterator iterator for std::vector<Border*>, std::list<Border*>, etc.
     * @param first_lf start of the lane following borders
     * @param last_lf end of the lane following borders (last valid entry)
     * @todo implement optional cost metrics for gate selection (strategy pattern)
     * @return A pair of Iterator,Border*, which describes right of current lane=first and right of target lane=second. If no gate can be found, (last_lf,nullptr) is returned
     */
    template <typename Iterator>
    std::pair<Iterator,Border*> findBestGateEntryPoint(Iterator first_lf, Iterator last_lf)
    {
        std::pair<Iterator,Border*> result = std::make_pair(last_lf,nullptr);
        adore::env::NavigationCost best_cost(adore::env::NavigationCost::maximum_cost());
        for(Iterator current = first_lf;current!=std::next(last_lf,1);current++)
        {
            Border* currentBorder = *current;
            auto adjacent_lane = borderSet_->getLaneChangeTarget(currentBorder,lc_direction_left_);
            if(adjacent_lane.first==nullptr||adjacent_lane.second==nullptr)
            {
                continue;
            }
            if(!borderSet_->borderTypeValid(adjacent_lane.second))
            {
                continue;
            }
            
            auto cost_query = borderCostMap_->find(adjacent_lane.second->m_id);
            adore::env::NavigationCost cost(adore::env::NavigationCost::maximum_cost());
            if(!(cost_query==borderCostMap_->end()))
            {
                cost = cost_query->second;
            }
            //for available navigation cost, take the first gate for tie-braking
            //for unavailable navigation cost, replace initializer and use last gate (avoid lane changes)
            if(cost<best_cost || (cost==best_cost && cost.getCombinedCost()==adore::env::NavigationCost::maximum_cost()))
            {
                result = std::make_pair(current,adjacent_lane.second);
                best_cost = cost;
            }
        }
        return result;
    }

    /**
     * @brief computes the gate region for a lane change
     * Supply iterators for a list or vector of Border*, which describe the current lane.
     * The area given by the iterators is searched for a gate.
     * Gates fully located behind the vehicle are ignored as they are unattainable: The current_lf parameter allows to start the search for the gate at the vehicle location.
     * @return a std::pair of iterators pointing to gate start and gate end. If no gate is found, (last_lf+1,last_lf+1) is returned
     * @param Iterator iterator for std::vector<Border*>, std::list<Border*>, etc.
     * @param current_lf iterator pointing to current border in lane following borders
     * @param first_lf start of the lane following borders
     * @param last_lf end of the lane following borders (last valid entry)
     */
    template<typename Iterator>
    std::pair<Iterator,Iterator> getGateRegion(Iterator current_lf, Iterator first_lf, Iterator last_lf)
    {
        auto best_gate = findBestGateEntryPoint<Iterator>(current_lf,last_lf);//best_gate: (source lane iterator, target lane Border*)
        if(best_gate.second==nullptr)return std::make_pair(std::next(last_lf,1),std::next(last_lf,1));//no gate available
        auto gate_start = best_gate;
        for(Iterator candidate = std::prev(best_gate.first);candidate!=std::prev(first_lf);candidate--)//search upstream of best_gate entry point
        {
            auto lct = borderSet_->getLaneChangeTarget(*candidate,lc_direction_left_);
            if(lct.first==nullptr||lct.second==nullptr)break;//no lane change target -> beyond start of gate
            if(!borderSet_->borderTypeValid(lct.second))break;// lane change target invalid -> beyond start of gate
            if(lct.second->isContinuousPredecessorOf(gate_start.second))
            {
                gate_start.first = candidate;
                gate_start.second = lct.second;
            }
            else
            {
                break;
            }
        }
        auto gate_end = best_gate;
        for(Iterator candidate = std::next(best_gate.first);candidate!=std::next(last_lf);candidate++)//search downstream of best_gate entry point
        {
            auto lct = borderSet_->getLaneChangeTarget(*candidate,lc_direction_left_);
            if(lct.first==nullptr||lct.second==nullptr)break;//no lane change target -> beyond end of gate
            if(!borderSet_->borderTypeValid(lct.second))break;// lane change target invalid -> beyond start of gate
            if(lct.second->isContinuousSuccessorOf(gate_end.second))
            {
                gate_end.first = candidate;
                gate_end.second = lct.second;
            }
            else
            {
                break;
            }
        }
        return std::make_pair(gate_start.first,gate_end.first);
    }

};
}
}
}
