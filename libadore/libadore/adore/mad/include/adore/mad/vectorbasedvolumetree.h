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
#include <vector>
#include <queue>

namespace adore
{
    namespace mad
    {
        /**
         * A class, which implements a hierarchical volume representation based on st::vector.
         * Template parameter VolumeType: Type of collected data
         * Template parameter BoundingFunctor: Computes bound for a range in a vector of VolumeType: 
         * Has to implement "IndexedVolumeType operator()(const VolumeVector& vector,int i0,int i1)const"
         */
        template<typename VolumeType, typename BoundingFunctor>
        class VectorBasedVolumeTree
        {
            public:
            typedef typename std::pair<std::pair<int,int>,VolumeType> IndexedVolumeType;
            typedef typename std::vector<IndexedVolumeType> VolumeVector;

            private:
            BoundingFunctor bound_;/**<helps to bound a range*/
            std::vector<VolumeVector> levels_;/**<bounding and occupancy volumes organized as a tree: levels_[0] contains bounding volumes. if levels_[i] contains only a single bounding volume, then index range must include complete base_range and then i is the highest level.*/
            int branching_factor_;/**<determines how many bounding/occupancy volumes in levels_[i] are bounded by a single bounding volume in levels_[i+1]*/
            bool levels_ok_;/**<determines whether levels above 0 have to be recomputed*/

            struct SearchPosition
            {
                int lvl_,i0_,i1_;/**<level and index range*/
                SearchPosition(int lvl,int i0,int i1):lvl_(lvl),i0_(i0),i1_(i1){} 
            };
            struct SearchState
            {
                SearchPosition first_,second_;
                double value_;
                SearchState(double value,SearchPosition first,SearchPosition second)
                :first_(first),second_(second),value_(value){}
            };
            
            public:
            VectorBasedVolumeTree():branching_factor_(3),levels_ok_(true)
            {
                levels_.push_back(VolumeVector());
            }

            VolumeVector& getLevel(int i)
            {
                return levels_[i];
            }

            const VolumeVector& getLevel(int i)const 
            {
                return levels_[i];
            }
            /**
             * returns size of levels[0]
             */
            int getOccupancyCount()
            {
                return levels_[0].size();
            }
            const int getOccupancyCount()const
            {
                return levels_[0].size();
            }
            /**
             * change branching factor
             */
            void setPreferredBranchingFactor(int f)
            {
                branching_factor_ = (std::max)(1,f);
            }
            /**
             * inserts an occupancy volume in levels_[0]
             */
            void insert(const VolumeType& volume)
            {
                IndexedVolumeType ivolume;
                ivolume.second = volume;
                ivolume.first.first = levels_[0].size();
                ivolume.first.second = levels_[0].size();
                levels_[0].push_back(ivolume);
                if(levels_.size()>1)levels_ok_=false;
            }
            /**
             * @return number of levels after base
             */
            int getLevelCount()const
            {
                return levels_.size();
            }
            /**
             * removes bounding volume levels
             */
            void remove_all_levels()
            {
                while(levels_.size()>1)levels_.pop_back();
                levels_ok_ = true;
            }
            /**
             * computes a maximum number of levels (at most 1000)
             */
            void compute_all_levels()
            {
                setLevelCount(1000);
            }
            /**
             * recomputes all levels, if levels are not ok
             */
            void recompute_levels()
            {
                if(!levels_ok_)
                {
                    remove_all_levels();
                    compute_all_levels();
                }
            }

            /**
             * if the current level-count is higher then count, levels are removed until count is achieved
             * if the current level-count is smaller, additional levels are added, until count is achieved or until highest level contains only one element
             */
            void setLevelCount(int count)
            {
                if(!levels_ok_)remove_all_levels();
                while(levels_.size()>(std::max)(1,count))levels_.pop_back();
                while(levels_.size()<count && levels_.back().size()>1)
                {
                    levels_.push_back(VolumeVector());
                    const VolumeVector& lower = levels_[levels_.size()-2];
                    VolumeVector& higher = levels_[levels_.size()-1];
                    for(int i0=0;i0<lower.size();i0+=branching_factor_)
                    {
                        const int i1 = (std::min)((int)lower.size()-1,i0+branching_factor_-1);
                        higher.push_back(bound_(lower,i0,i1));
                    }
                }
            }
           /**
             * computes the minimum value of a distance metric specified by MetricFunctor:
             * double operator()(const VolumeType& a,const OtherFolumeType& b)const
             * @param cutoff metric values above cutoff are ignored for min computation
             * @param result_value is set, if returns true, to the minimum value
             * @return true, if any value below cutoff appears
             */
            template<typename MetricFunctor,typename OtherVolumeType,typename OtherFunctorType>
            bool compute_min(const VectorBasedVolumeTree<OtherVolumeType,OtherFunctorType>& other,double cutoff,double& result_value,const MetricFunctor& f)const
            {
                const double guard = -1.e99;
                //heap
                auto cmp = [](const SearchState& left, const SearchState& right) { return left.value_ > right.value_; };
                std::priority_queue<SearchState, std::vector<SearchState>, decltype(cmp)> open(cmp);
                //initial state of search
                SearchState s0(guard,//lower bound on metric value (not computed yet)
                    SearchPosition(levels_.size()-1,0,levels_.back().size()-1),//complete this range
                    SearchPosition(other.levels_.size()-1,0,other.levels_.back().size()-1)
                    );
                open.push(s0);
                while(!open.empty())
                {
                    SearchState s = open.top();
                    open.pop();
                    //goal condition: 1-to-1 relation on lvl 0 with minimum value
                    if( s.first_.lvl_==0 && s.first_.i0_==s.first_.i1_
                    &&  s.second_.lvl_==0 && s.second_.i0_==s.second_.i1_ )
                    {
                        result_value = s.value_;
                        return true;
                    }
                    //partition the higher level
                    if( s.first_.lvl_ > s.second_.lvl_ )
                    {
                        if( s.first_.i0_ == s.first_.i1_ )
                        {
                            //unfold range
                            auto& data = levels_[s.first_.lvl_][s.first_.i0_];
                            s.first_.lvl_ = s.first_.lvl_-1;
                            s.first_.i0_ = data.first.first;
                            s.first_.i1_ = data.first.second;
                        }
                    }
                    else
                    {
                        if( s.second_.lvl_>0 && s.second_.i0_ == s.second_.i1_ )
                        {
                            //unfold range
                            auto& data = other.levels_[s.second_.lvl_][s.second_.i0_];
                            s.second_.lvl_ = s.second_.lvl_-1;
                            s.second_.i0_ = data.first.first;
                            s.second_.i1_ = data.first.second;
                        }
                    }
                    for(int i=s.first_.i0_;i<=s.first_.i1_;i++)
                    {
                        for(int j=s.second_.i0_;j<=s.second_.i1_;j++)
                        {
                            SearchState child(
                                //compute metric between i and j
                                f(levels_[s.first_.lvl_][i].second,other.levels_[s.second_.lvl_][j].second),
                                SearchPosition(s.first_.lvl_,i,i),
                                SearchPosition(s.second_.lvl_,j,j)
                            );
                            if(child.value_<cutoff)
                            {
                                open.push(child);
                            }
                        }
                    }


                }                
                return false;
            }

        };

    }
}