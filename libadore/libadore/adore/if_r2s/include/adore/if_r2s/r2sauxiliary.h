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

#pragma once
#include <vector>
#include <tuple>
#include <algorithm>
#include <adore/env/borderbased/coordinate.h>
#include <adore/env/borderbased/border.h>
#include <adore/mad/csvlog.h>
#include <adore/mad/llinearpiecewisefunction.h>

namespace adore
{
    namespace if_r2s
    {
        typedef std::vector<adore::env::BorderBased::Coordinate> TR2SGeometry;

        /**
         * @brief basic storage struct for ReferenceLine from file
         * 
         */
        struct ReferenceLine
        {
            enum LINETYPE
            {
                StandardLine,
                ConnectionLine
            };
            int id_;
            TR2SGeometry geometry_;
            bool oneway_;
            LINETYPE linetype_;

            ReferenceLine()
            {
                id_=-1;
                oneway_=false;
                linetype_ = LINETYPE::StandardLine;
            }
            bool isSane() const
            {
                return geometry_.size()>1 && getBorderID().distance()>0.1;
            }

            adore::env::BorderBased::BorderID getBorderID() const
            {
                return adore::env::BorderBased::BorderID(*geometry_.begin(),*geometry_.rbegin());
            }
            adore::env::BorderBased::BorderID getInverseBorderID() const
            {
                return adore::env::BorderBased::BorderID(*geometry_.rbegin(),*geometry_.begin());
            }
            adore::mad::function_type_xyz* getBorderPath()const 
            {
                //filter distinct points
                std::vector<int> distinct;
                double tol = 0.05;
                distinct.push_back(0);
                for(int i=1;i<geometry_.size();i++)
                {
                    if(geometry_[i].distance(geometry_[*distinct.rbegin()])>tol)distinct.push_back(i);
                }

                //collect distinct points in path
                adore::mad::function_type_xyz* path = new adore::mad::function_type_xyz(distinct.size(),0.0);
                double s=0.0;
                auto ci = geometry_[distinct[0]];
                for(int i = 0;i<distinct.size();i++)
                {
                    auto cj = geometry_[distinct[i]];
                    s+= ci.distance(cj);
                    path->getData()(0,i) = s;
                    path->getData()(1,i) = cj.m_X;
                    path->getData()(2,i) = cj.m_Y;
                    path->getData()(3,i) = cj.m_Z;
                    ci = cj;
                }
                return path;
            }
            adore::mad::function_type_xyz* getInverseBorderPath()const
            {
                //filter distinct points
                std::vector<int> distinct;
                double tol = 0.05;
                distinct.push_back(geometry_.size()-1);
                for(int i=geometry_.size()-2;i>=0;i--)
                {
                    if(geometry_[i].distance(geometry_[*distinct.rbegin()])>tol)distinct.push_back(i);
                }

                //collect distinct points in path
                adore::mad::function_type_xyz* path = new adore::mad::function_type_xyz(distinct.size(),0.0);
                double s=0.0;
                auto ci = geometry_[distinct[0]];
                for(int i = 0;i<distinct.size();i++)
                {
                    auto cj = geometry_[distinct[i]];
                    s+= ci.distance(cj);
                    path->getData()(0,i) = s;
                    path->getData()(1,i) = cj.m_X;
                    path->getData()(2,i) = cj.m_Y;
                    path->getData()(3,i) = cj.m_Z;
                    ci = cj;
                }
                return path;
            }
            adore::env::BorderBased::Border* getBorder()const
            {
                auto border = new adore::env::BorderBased::Border(
                    getBorderID(),
                    getBorderPath()
                );
                border->m_type = adore::env::BorderBased::BorderType::DRIVING;
                return border;
            }
            adore::env::BorderBased::Border* getInverseBorder()const
            {
                auto border = new adore::env::BorderBased::Border(
                    getInverseBorderID(),
                    getInverseBorderPath()
                );
                border->m_type = adore::env::BorderBased::BorderType::DRIVING;
                return border;
            }
        };
        /**
         * @brief basic storage struct for lane borders from file
         * 
         */
        struct LaneBorder
        {
            enum TYPE
            {
                NONE,
                DRIVING,
                BIKE,
                PARKING,
                SHOULDER,
                RESTRICTED,
                TOWN,
                OTHER
            };
            int id_;
            int parent_id_;
            TR2SGeometry geometry_;
            TYPE type_;
            LaneBorder()
            {
                id_=-1;
                parent_id_=-1;
                type_=TYPE::NONE;
            }
            bool isSane() const
            {
                return geometry_.size()>1 && getBorderID().distance()>0.1;
            }
            adore::env::BorderBased::BorderID getBorderID() const
            {
                return adore::env::BorderBased::BorderID(*geometry_.begin(),*geometry_.rbegin());
            }
            bool isOnRightHandSide(const ReferenceLine& rl)const 
            {
                adore::env::BorderBased::BorderID myID = getBorderID();
                return myID.distance(rl.getBorderID())<=myID.distance(rl.getInverseBorderID());
            }
            double sortingDistance(const ReferenceLine& rl) const
            {
                adore::env::BorderBased::BorderID myID = getBorderID();
                if(isOnRightHandSide(rl))
                {
                    return -myID.distance(rl.getBorderID());
                }
                else
                {
                    return myID.distance(rl.getInverseBorderID());
                }
            }
            double distance(const ReferenceLine& rl) const
            {
                adore::env::BorderBased::BorderID myID = getBorderID();
                return std::min(myID.distance(rl.getBorderID()),myID.distance(rl.getInverseBorderID()));
            }
            adore::mad::function_type_xyz* getBorderPath()const 
            {
                //filter distinct points
                std::vector<int> distinct;
                double tol = 0.05;
                distinct.push_back(0);
                for(int i=1;i<geometry_.size();i++)
                {
                    if(geometry_[i].distance(geometry_[*distinct.rbegin()])>tol)distinct.push_back(i);
                }

                //collect distinct points in path
                adore::mad::function_type_xyz* path = new adore::mad::function_type_xyz(distinct.size(),0.0);
                double s=0.0;
                auto ci = geometry_[distinct[0]];
                for(int i = 0;i<distinct.size();i++)
                {
                    auto cj = geometry_[distinct[i]];
                    s+= ci.distance(cj);
                    path->getData()(0,i) = s;
                    path->getData()(1,i) = cj.m_X;
                    path->getData()(2,i) = cj.m_Y;
                    path->getData()(3,i) = cj.m_Z;
                    ci = cj;
                }
                return path;
            }
            adore::env::BorderBased::Border* getBorder(const adore::env::BorderBased::BorderID& leftID)const
            {
                adore::env::BorderBased::Border* border = new adore::env::BorderBased::Border(
                    getBorderID(),
                    getBorderPath()
                );
                if(type_==DRIVING)border->m_type = adore::env::BorderBased::BorderType::DRIVING;
                else border->m_type = adore::env::BorderBased::BorderType::OTHER;
                border->m_left = new adore::env::BorderBased::BorderID(leftID);
                return border;
            }
        };
        
        typedef std::vector<ReferenceLine> TReferenceLineVector;
        typedef std::vector<LaneBorder> TLaneBorderVector;
        typedef adore::mad::LLinearPiecewiseFunctionM<double,2> TGeometryFunction;
        typedef std::pair<TGeometryFunction*,LaneBorder::TYPE> TFunctionTypePair;
        /**
         * @brief directed borders, ordered from center line to outer border
         * 
         */
        class Section
        {
            /**
             * @brief storage class to access different functions by their domain intervals
             * 
             */
            class FunctionMap
            {
                private:
                std::vector<TFunctionTypePair> functions_;

                std::vector<TFunctionTypePair> getFunctionsStartingAtPoint(double x, double y)
                {
                    std::vector<TFunctionTypePair> value;
                    for(auto it = functions_.begin(); it!=functions_.end(); it++)
                    {
                        auto fun = (*it).first;
                        if(fun->f(fun->limitLo())(0)==x && fun->f(fun->limitLo())(1)==y)
                        {
                            value.push_back(*it);
                        }
                    }
                    return value;
                }
                std::vector<TFunctionTypePair> getFunctionsEndingAtPoint(double x, double y)
                {
                    std::vector<TFunctionTypePair> value;
                    for(auto it = functions_.begin(); it!=functions_.end(); it++)
                    {
                        auto fun = (*it).first;
                        if(fun->f(fun->limitHi())(0)==x && fun->f(fun->limitHi())(1)==y)
                        {
                            value.push_back(*it);
                        }
                    }
                    return value;
                }
                void resetSParameter(TGeometryFunction* geometry)
                {
                    auto c1 = env::BorderBased::Coordinate(geometry->getData()(1,0),geometry->getData()(2,0),0);
                    geometry->getData()(0,0) = 0.0;
                    for(int i = 1; i<geometry->getData().nc(); i++)
                    {
                        auto c2 = env::BorderBased::Coordinate(geometry->getData()(1,i),geometry->getData()(2,i),0);
                        geometry->getData()(0,i) = geometry->getData()(0,i-1) + c1.distance(c2);
                        c1 = c2;
                    }
                }
                public:
                /**
                 * @brief add function to container
                 * 
                 * @param function 
                 * @param type 
                 */
                void addFunction(TGeometryFunction* function, LaneBorder::TYPE type)
                {
                    functions_.push_back(std::make_pair(function,type));
                }
                /**
                 * @brief returns functions that are valid at the given parameter
                 * 
                 * @param x 
                 * @return std::vector<TFunctionTypePair> 
                 */
                std::vector<TFunctionTypePair> getFunctionsAtParameter(double x)
                {
                    std::vector<TFunctionTypePair> value;
                    for(auto it = functions_.begin(); it!=functions_.end(); it++)
                    {
                        auto fun = (*it).first;
                        if(fun->limitLo() <= x && fun->limitHi() >= x)
                        {
                            value.push_back(*it);
                        }
                    }
                    return value;
                }
                /**
                 * @brief getter method
                 * 
                 * @return std::vector<TFunctionTypePair> 
                 */
                std::vector<TFunctionTypePair> getFunctions()
                {
                    return functions_;
                }
                /**
                 * @brief  try to repair some mistakes that might occur due to matching to closest point on refline
                 * 
                 * @param refLine 
                 * @param intervals 
                 */
                void mend(TGeometryFunction* refLine, std::set<double>& intervals)
                {
                    double x0 = refLine->limitLo();
                    double x1 = refLine->limitHi();
                    // if there's only one function, it's likely it goes over the whole length of the reference line
                    if(functions_.size()==1)
                    {
                        auto fun = functions_.at(0).first;
                        if(x0!=fun->limitLo() || x1!=fun->limitHi())
                        {
                            fun->stretchDomain(x0,x1);
                            intervals.insert(x0);
                            intervals.insert(x1);
                        }
                        return;
                    }
                    if(intervals.size()>=2)
                    {
                        auto s_start0 = *intervals.begin();
                        auto s_end1 = *intervals.rbegin();
                        // the first functions are likely to start with the reference line
                        // if no functions exist at x0, stretch all functions at fist interall point to x0
                        auto functions = getFunctionsAtParameter(x0);
                        if(functions.size()<1)
                        {
                            functions = getFunctionsAtParameter(s_start0);
                            for(auto f = functions.begin(); f!=functions.end(); f++)
                            {
                                auto fun = f->first;
                                fun->stretchDomain(x0,fun->limitHi());                                
                            }
                            intervals.insert(x0);
                        }
                        // the last functions are likely to end with the reference line
                        // if no functions exist at x1, stretch all functions at last interall point to x0
                        functions = getFunctionsAtParameter(x1);
                        if(functions.size()<1)
                        {
                            functions = getFunctionsAtParameter(s_end1);
                            for(auto f = functions.begin(); f!=functions.end(); f++)
                            {
                                auto fun = f->first;
                                fun->stretchDomain(fun->limitLo(),x1);                                
                            }
                            intervals.insert(x1);
                        }
                    }
                    return;

                    // further ideas: multilane treatment
                    // combine close interval values at start and end
                }
                /**
                 * @brief delete function objects
                 * 
                 */
                void deleteFunctions()
                {
                    for(auto it = functions_.begin(); it!=functions_.end(); it++)
                    {
                        if(!((*it).first==nullptr))
                        {
                            delete (*it).first;
                        }
                    }
                }
            };

            /**
             * @brief storage for functions in the same directions as the reference line
             * 
             */
            FunctionMap functionMapRight_;
            /**
             * @brief storage for functions originally in the opposite direction as the reference line, stored in functions in the same direction as reference line
             * 
             */
            FunctionMap functionMapLeft_;
            bool oneway_;
            TGeometryFunction* referenceLine_;
            std::set<double> intervals_;

            /**
             * @brief easy conversion from a coordinate vector to a LLinearPiecewiseFunction
             * 
             * @param geometry 
             * @return TGeometryFunction* 
             */
            TGeometryFunction* geometry2function(TR2SGeometry geometry)
            {
                adoreMatrix<double,0,0> m;
                m.set_size(3,geometry.size());
                m(0,0) = 0.0;
                m(1,0) = geometry.at(0).m_X;
                m(2,0) = geometry.at(0).m_Y;
                for(int i = 1; i<geometry.size(); i++)
                {
                    m(0,i) = m(0,i-1)+geometry.at(i).distance(geometry.at(i-1));
                    m(1,i) = geometry.at(i).m_X;
                    m(2,i) = geometry.at(i).m_Y;
                }
                TGeometryFunction* fun = new TGeometryFunction(m);
                LOG_T("limitLo: %.2f\tlimithi: %.2f", fun->limitLo(), fun->limitHi());
                return fun;
            }

            public:
            Section()
            {
                referenceLine_ = nullptr;
                oneway_ = false;
            }
            /**
             * @brief setter
             * 
             * @param rl 
             */
            void setReferenceLine(ReferenceLine rl)
            {
                referenceLine_ = geometry2function(rl.geometry_);
                oneway_ = rl.oneway_;
            }
            /**
             * @brief add lane border to section, determine its valid range
             * 
             * @param lb 
             */
            void addLaneBorder(LaneBorder lb)
            {
                if(referenceLine_ == nullptr) throw("Section::referenceLine_ is undefined.");
                LOG_T("LaneBorder ID: %i", lb.id_);
                TGeometryFunction* fun = geometry2function(lb.geometry_);
                // determine whether lane is left or right from center
                double d = 0;
                for(auto point = lb.geometry_.begin(); point!=lb.geometry_.end(); point++)
                {
                    referenceLine_->getClosestParameter(point->m_X,point->m_Y,1,2,d);
                    if(abs(d) > 0.05) break;
                }
                auto p_start = fun->f(fun->limitLo());
                auto p_end   = fun->f(fun->limitHi());
                double d_start, d_end;
                double s_start = referenceLine_->getClosestParameter(p_start(0),p_start(1),1,2,d_start);
                double s_end   = referenceLine_->getClosestParameter(p_end(0),  p_end(1),  1,2,d_end);
                // invert if necessary
                if(s_start < s_end)
                {
                    fun->stretchDomain(s_start, s_end);
                }
                else
                {
                    fun->invertDomain();
                    fun->stretchDomain(s_end,s_start);
                }
                if(d <= 0)
                {
                    functionMapRight_.addFunction(fun,lb.type_);
                }
                else
                {
                    functionMapLeft_.addFunction(fun,lb.type_);
                }
                intervals_.insert(s_start);
                intervals_.insert(s_end);
                LOG_T("p_start: (%.2f|%.2f)",p_start(0),p_start(1));
                LOG_T("p_end  : (%.2f|%.2f)",p_end(0),p_end(1));
                LOG_T("s_start: %.2f",s_start);
                LOG_T("s_end  : %.2f",s_end);
            }
            /**
             * @brief getter
             * 
             * @return std::set<double> 
             */
            std::set<double> getIntervals()
            {
                return intervals_;
            }
            /**
             * @brief get functions that are valid at given parameter
             * 
             * @param x 
             * @return std::vector<TFunctionTypePair> 
             */
            std::vector<TFunctionTypePair> getRightFunctionsAtParameter(double x)
            {
                return functionMapRight_.getFunctionsAtParameter(x);
            }
            /**
             * @brief get functions that are valid at given parameter
             * 
             * @param x 
             * @return std::vector<TFunctionTypePair> 
             */
            std::vector<TFunctionTypePair> getLeftFunctionsAtParameter(double x)
            {
                return functionMapLeft_.getFunctionsAtParameter(x);
            }
            /**
             * @brief getter
             * 
             * @return TGeometryFunction* 
             */
            TGeometryFunction* getReferenceLineFunction()
            {
                return referenceLine_;
            }
            /**
             * @brief delete objects in pointers
             * 
             */
            void cleanup()
            {
                delete referenceLine_;
                functionMapRight_.deleteFunctions();
                functionMapLeft_.deleteFunctions();
            }
            /**
             * @brief try to repair mistakes that occured during function generation
             * 
             */
            void mend()
            {
                functionMapLeft_.mend(referenceLine_,intervals_);
                functionMapRight_.mend(referenceLine_,intervals_);
            }
            /**
             * @brief getter
             * 
             * @return FunctionMap 
             */
            FunctionMap getFunctionMapRight()
            {
                return functionMapRight_;
            }
            /**
             * @brief getter
             * 
             * @return FunctionMap 
             */
            FunctionMap getFunctionMapLeft()
            {
                return functionMapLeft_;
            }
            bool isOneway()
            {
                return oneway_;
            }
            ~Section()
            {
            }
        };
    }
}