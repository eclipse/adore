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
 *   Robert Markowski - initial API and implementation
 ********************************************************************************/
#include <adore/if_r2s/r2s2borderbased.h>
#include <adore/if_r2s/r2sfilereader.h>
#include <adore/mad/csvlog.h>

#include <algorithm>
#include <fstream>
#include <sstream>


namespace adore
{
    namespace if_r2s
    {
        R2S2BorderBasedConverter::R2S2BorderBasedConverter()
        {
            extractOnlyReferenceLines_ = false;
        }
        void R2S2BorderBasedConverter::setExtractOnlyReferenceLines(bool b)
        {
            extractOnlyReferenceLines_ = b;
        }
        env::BorderBased::BorderType::TYPE R2S2BorderBasedConverter::convertLaneBorderType(LaneBorder::TYPE type)
        {
            switch(type)
            {
                case LaneBorder::DRIVING:
                {
                    return env::BorderBased::BorderType::DRIVING;
                }
                default:
                {
                    return env::BorderBased::BorderType::OTHER;
                }
            }
        
        }
        void R2S2BorderBasedConverter::cleanup(TSectionMap& sectionmap)
        {
            for(auto it=sectionmap.begin(); it!=sectionmap.end(); it++)
            {
                (*it).second.cleanup();
            }   
        }
        void R2S2BorderBasedConverter::do_convert(std::string referenceLineFile, std::string laneBorderFile, env::BorderBased::BorderSet& targetset, TSectionMap& sectionmap)
        {
            TReferenceLineVector reflines;
            TLaneBorderVector laneborders;
            R2SFileReader::readReferenceLines(referenceLineFile,reflines);
            R2SFileReader::readLaneBorders(laneBorderFile,laneborders);
            int version = 1;
            switch(version)
            {
                case 0:
                {
                    LOG_I("Finished reading files.");
                    group(reflines,laneborders,sectionmap);
                    LOG_I("Finished grouping.");
                    convertToBorder(sectionmap,targetset);
                    LOG_I("Finished converting.");
                    break;
                }
                case 1:
                {
                    toBorders(reflines,laneborders,targetset);
                    break;
                }
            }
 
        }
        void R2S2BorderBasedConverter::toBorders(const TReferenceLineVector& reflines,const TLaneBorderVector& laneborders,env::BorderBased::BorderSet& targetset)
        {
            for(auto& rl:reflines)if(rl.isSane())
            {
                typedef std::pair<int,double> Tidx;
                std::vector<Tidx> left;
                std::vector<Tidx> right;
                
                std::cout<<rl.id_<<"(";
                int i=0;
                for(auto it=laneborders.begin();it!=laneborders.end();it++)
                {
                    auto &b = *it;
                    if(b.parent_id_==rl.id_ && b.isSane())
                    {
                        //right hand side as seen from original direction of reference line
                        if(b.isOnRightHandSide(rl))
                        {
                            right.push_back(std::make_pair(i,b.distance(rl)));
                        }
                        else
                        {
                            left.push_back(std::make_pair(i,b.distance(rl)));
                        }
                    }
                    i++;
                }
                if(left.size()>0)
                {
                    auto parent_border = rl.getInverseBorder();
                    if(right.size()>0)parent_border->m_left = new adore::env::BorderBased::BorderID(rl.getBorderID());
                    std::sort(left.begin(),left.end(),
                                [](const Tidx& a,const Tidx& b){return a.second<b.second;});
                    auto left_neighbor_id = parent_border->m_id;
                    for(auto idx:left)
                    {
                        std::cout<<laneborders[idx.first].id_<<",";
                        auto border = laneborders[idx.first].getBorder(left_neighbor_id);
                        left_neighbor_id = border->m_id;
                        if(targetset.getBorder(border->m_id)==nullptr)
                        {
                            targetset.insert_border(border);
                        }
                        else
                        {
                            std::cout<<"[x"<<laneborders[idx.first].id_<<"]";
                            delete border;
                        }                        
                    }
                    if(targetset.getBorder(parent_border->m_id)==nullptr)
                    {
                        targetset.insert_border(parent_border);
                    }
                    else
                    {
                        std::cout<<"[x"<<rl.id_<<"]";
                        delete parent_border;
                    }
                }
                std::cout<<"|";
                if(right.size()>0)
                {
                    auto parent_border = rl.getBorder();
                    if(left.size()>0)parent_border->m_left = new adore::env::BorderBased::BorderID(rl.getInverseBorderID());
                    std::sort(right.begin(),right.end(),
                                [](const Tidx& a,const Tidx& b){return a.second<b.second;});
                    auto left_neighbor_id = parent_border->m_id;
                    for(auto idx:right)
                    {
                        std::cout<<laneborders[idx.first].id_<<",";
                        auto border = laneborders[idx.first].getBorder(left_neighbor_id);
                        left_neighbor_id = border->m_id;
                        if(targetset.getBorder(border->m_id)==nullptr)
                        {
                            targetset.insert_border(border);
                        }
                        else
                        {
                            std::cout<<"[x"<<laneborders[idx.first].id_<<"]";
                            delete border;
                        }                        
                    }
                    if(targetset.getBorder(parent_border->m_id)==nullptr)
                    {
                        targetset.insert_border(parent_border);
                    }
                    else
                    {
                        std::cout<<"[x"<<rl.id_<<"]";
                        delete parent_border;
                    }
                }
                std::cout<<")\n";
            }            
        }
        void R2S2BorderBasedConverter::convert(std::string referenceLineFile, std::string laneBorderFile, env::BorderBased::BorderSet& targetset, TSectionMap& sectionmap)
        {
            do_convert(referenceLineFile,laneBorderFile,targetset,sectionmap);
        }
        void R2S2BorderBasedConverter::convert(std::string referenceLineFile, std::string laneBorderFile, env::BorderBased::BorderSet& targetset)
        {
            TSectionMap sectionmap;
            do_convert(referenceLineFile,laneBorderFile,targetset,sectionmap);
            cleanup(sectionmap);
        }
        void R2S2BorderBasedConverter::group(TReferenceLineVector rlv, TLaneBorderVector lbv, TSectionMap& sectionmap)
        {
            // create a section for each reference line
            for(auto it=rlv.begin(); it!=rlv.end(); it++)
            {
                auto refLine = (*it);
                Section s;
                s.setReferenceLine(refLine);
                sectionmap.insert(std::make_pair(refLine.id_,s));
            }
            // add laneborders to respective sections
            for(auto it=lbv.begin(); it!=lbv.end(); it++)
            {
                auto laneborder = *it;
                sectionmap.at(laneborder.parent_id_).addLaneBorder(laneborder);
            }
            // try to repair functions
            for(auto it=sectionmap.begin(); it!=sectionmap.end(); it++)
            {
                (*it).second.mend();
            }
        }
        void R2S2BorderBasedConverter::sortFunctionsByDistance(std::vector<TFunctionTypePair>& functions, TDist2Function& dist2function, double s0, double s1)
        {
            static const double DIST_ACCURACY = 0.25;

            // save start and end base point for distance calculation
            auto p_ref0 = dist2function.at(0).first->f(s0);
            auto p_ref1 = dist2function.at(0).first->f(s1);
            // sort functions by their distance to base points
            for(auto funIt=functions.begin(); funIt!=functions.end(); funIt++)
            {
                auto fun = (*funIt).first;
                auto p_cur0 = fun->f(s0);
                auto p_cur1 = fun->f(s1);
                double dx, dy;
                dx = p_ref0(0) - p_cur0(0);
                dy = p_ref0(1) - p_cur0(1);
                int distance = static_cast<int>(std::sqrt((dx*dx + dy*dy))/DIST_ACCURACY);                
                if(dist2function.find(distance)==dist2function.end())
                {
                    dist2function[distance] = *funIt;
                }
                // if first point is equal, sort by last point instead
                else
                {
                    dx = p_ref1(0) - p_cur1(0);
                    dy = p_ref1(1) - p_cur1(1);
                    int distance_back_cur = static_cast<int>(std::sqrt(dx*dx + dy*dy)/DIST_ACCURACY);
                    fun = dist2function[distance].first;
                    p_cur1 = fun->f(s1);
                    dx = p_ref1(0) - p_cur1(0);
                    dy = p_ref1(1) - p_cur1(1);
                    int distance_back_other = static_cast<int>(std::sqrt(dx*dx + dy*dy)/DIST_ACCURACY);
                    if(distance_back_cur < distance_back_other)
                    {
                        dist2function[distance-1] = *funIt;
                    }
                    else
                    {
                        dist2function[distance+1] = *funIt;
                    }
                }
            }
        }
        void R2S2BorderBasedConverter::createBorders(TDist2Function& dist2function, double s0, double s1, int max_points, adore::env::BorderBased::BorderSet& targetSet, bool inverted=false)
        {
            /* create point buffer */
            adoreMatrix<double,0,0> m;
            m = dlib::zeros_matrix<double>(4,max_points);

            /* convert center line */
            auto it = dist2function.begin();
            auto fun = (*it).second.first;
            auto type = convertLaneBorderType((*it).second.second);
            int points =fun->export_points(m,s0,s1,1e-10);
            adoreMatrix<double,4,0> data = colm(m,dlib::range(0,points-1));
            auto centerFun = new adore::mad::LLinearPiecewiseFunctionM<double,3>(data);
            if(inverted)
            {
                centerFun->invertDomain();
            }
            adore::env::BorderBased::Border* center = new adore::env::BorderBased::Border(centerFun,type);
            targetSet.insert_border(center);
            adore::env::BorderBased::Border* previous = center;

            // iterator is already definied, but increase by 1 to get next element
            for(it++;it!=dist2function.end();it++)
            {
                auto borderFun = (*it).second.first;
                type = convertLaneBorderType((*it).second.second);
                points = borderFun->export_points(m,s0,s1,1e-10);
                auto function = new adore::mad::LLinearPiecewiseFunctionM<double,3>(dlib::colm(m,dlib::range(0,points-1)));
                function->startDomainAtZero();
                if(inverted)
                {
                    function->invertDomain();
                }
                adore::env::BorderBased::Border* current = new adore::env::BorderBased::Border(function,type);
                current->m_left = new adore::env::BorderBased::BorderID(previous->m_id);
                previous = current;
                targetSet.insert_border(current);
            }
        }
        void R2S2BorderBasedConverter::convertToBorder(TSectionMap sectionmap, adore::env::BorderBased::BorderSet& targetSet)
        {
            for(auto sectionIt=sectionmap.begin(); sectionIt!=sectionmap.end(); sectionIt++)
            {
                auto section = (*sectionIt).second;
                auto s_values = section.getIntervals();
                static const double S_MIN_DIFFERENCE = 0.05;
                static const double S_OFFSET_START = S_MIN_DIFFERENCE/10;                
                static const double E_MAX = S_MIN_DIFFERENCE;
                static const int POINTSPERBORDER = 128;
                auto refline = section.getReferenceLineFunction();
                for(auto sIt=s_values.begin(); sIt!=s_values.end(); sIt++)
                {
                    TDist2Function dist2functionRight;
                    TDist2Function dist2functionLeft;
                    double s = *sIt;
                    auto s_nextIt = sIt;
                    s_nextIt++;
                    // if interval is too short or last interval, continue
                    if(s_nextIt==s_values.end() || *s_nextIt-s<S_MIN_DIFFERENCE)
                    {
                        continue;
                    }
                    double s_next = *s_nextIt;
                    // hitting exactly the limit also gives functions that are ending at that point, thus apply offset
                    auto functionsRight = section.getRightFunctionsAtParameter(s+S_OFFSET_START);
                    auto functionsLeft = section.getLeftFunctionsAtParameter(s+S_OFFSET_START);
                    
                    // add reference line to relevant functions at distance 0
                    dist2functionRight[0] = std::make_pair(refline,LaneBorder::NONE);
                    dist2functionLeft[0] = std::make_pair((TGeometryFunction*) refline->clone(),LaneBorder::NONE);

                    if(!extractOnlyReferenceLines_)
                    {
                        sortFunctionsByDistance(functionsRight,dist2functionRight,s,s_next);
                        sortFunctionsByDistance(functionsLeft ,dist2functionLeft ,s,s_next);
                    }
                    
                    double s_cut = 0;
                    while(true)
                    {
                        // find cutting distance depending on function complexity, upper limit is next s value
                        s_cut = s_next;
                        for(auto it_border = dist2functionRight.begin(); it_border!= dist2functionRight.end(); it_border++)
                        {
                            TGeometryFunction* border = it_border->second.first;
                            s_cut = std::min(s_cut,border->getXAfterNPoints(s,POINTSPERBORDER-2));
                        }
                        for(auto it_border = dist2functionLeft.begin();  it_border!= dist2functionLeft.end();  it_border++)
                        {
                            TGeometryFunction* border = it_border->second.first;
                            s_cut = std::min(s_cut,border->getXAfterNPoints(s,POINTSPERBORDER-2));
                        }

                        createBorders(dist2functionRight, s, s_cut, POINTSPERBORDER, targetSet);
                        createBorders(dist2functionLeft,  s, s_cut, POINTSPERBORDER, targetSet,!section.isOneway());
                        
                        s = s_cut;                    
                        if(s_next-s_cut < E_MAX) break;
                    }
                }
            }
        }
    }
}