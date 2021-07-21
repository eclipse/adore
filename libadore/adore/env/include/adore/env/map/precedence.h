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
#include <adore/mad/adoremath.h>
#include <adore/mad/com_patterns.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <string>
#include <iostream>
#include <fstream>

namespace adore
{
    namespace env
    {
        /**
         * @brief PriorityRoute implicitly references a route between two coordinates.
         * The coordinates should be chosen such that the route is unique.
         */
        struct PriorityRoute
        {
            public:
            typedef boost::geometry::model::point<double,3,boost::geometry::cs::cartesian> boost_point;
            typedef boost::geometry::model::box<boost_point> boost_box;
            adoreMatrix<double,3,1> from_;/**< coordinate at which priority route starts*/
            adoreMatrix<double,3,1> to_;/**< coordinate at which priority route ends*/
            bool coordinates_in_UTM_;/**< if true, coordinates are in UTM, otherwise in WGS84*/
            /**
             * @brief empty constructor sets coordinates to 0.0^3
             */
            PriorityRoute()
            {
                set(0.0,0.0,0.0,0.0,0.0,0.0);
                coordinates_in_UTM_ = true;
            }
            /**
             * @brief set the two coordinates
             */
            void set(double f1,double f2,double f3,double t1,double t2,double t3)
            {
                from_(0)=f1;
                from_(1)=f2;
                from_(2)=f3;
                to_(0)=t1;
                to_(1)=t2;
                to_(2)=t3;
            }
            /**
             * @brief reads PriorityRoute's two coordinates from a string
             * exemplary valid string "0.0,3.5,1.7;1.2,6.6,1.7"
             */
            void set(const std::string& s, bool coordinates_in_UTM=true)
            {
                std::stringstream is;
                double f1,f2,f3,t1,t2,t3;
                char sep;
                is<<s;
                is>> f1 >> sep >> f2 >> sep >> f3 >> sep >> t1 >> sep >> t2 >> sep >> t3;
                set(f1,f2,f3,t1,t2,t3);
                coordinates_in_UTM_ = coordinates_in_UTM;
                std::cout<<s<<std::endl;
            }
            /**
             * @brief returns a box in boost format, which encompasses start/end points
             */
            boost_box getBoostBox()
            {
                const double min0 = (std::min)(from_(0),to_(0));
                const double min1 = (std::min)(from_(1),to_(1));
                const double min2 = (std::min)(from_(2),to_(2));
                const double max0 = (std::max)(from_(0),to_(0));
                const double max1 = (std::max)(from_(1),to_(1));
                const double max2 = (std::max)(from_(2),to_(2));
                return boost_box(boost_point(min0,min1,min2),boost_point(max0,max1,max2));
            }
            /**
             * @brief returns true if all values are equal with other PriorityRoute
             */
            bool equals(const PriorityRoute& other)const
            {
                return  this->from_(0)==other.from_(0)
                &&      this->from_(1)==other.from_(1)
                &&      this->from_(2)==other.from_(2)
                &&      this->to_(0)==other.to_(0)
                &&      this->to_(1)==other.to_(1)
                &&      this->to_(2)==other.to_(2);                
            }
        };
        /**
         * @brief The PrecedenceRule defines a precedence relationship between two routes.
         * Vehicles on the low_ priority route have to yield, while vehicles on the high_ priority route may proceed. 
         */
        struct PrecedenceRule
        {
            public:
            PriorityRoute low_;/**< route with low priority, has to yield to high_*/
            PriorityRoute high_;/**< route with high priority, does *not* have to yield to low_*/
            bool unary_;/**< route with high priority has precedence over any other route, if unary_ is true. In that case, low_ is not used.*/

            PrecedenceRule():unary_(true){}

            /**
             * @brief reads PrecedenceRule's two PriorityRoutes from the input string
             * exemplary valid string "0.0,3.5,1.7;1.2,6.6,1.7 > 0.45,1.1,1.3;17,18,19": first=high_
             * exemplary valid string "0.0,3.5,1.7;1.2,6.6,1.7 < 0.45,1.1,1.3;17,18,19": first=low_
             * exemplary valid string "0.0,3.5,1.7;1.2,6.6,1.7": unary, higher then all crossing
             */
            void set(const std::string& s,bool coordinates_in_UTM=true)
            {
                std::size_t pos = s.find(">");
                if(pos!=std::string::npos)
                {
                    high_.set(s.substr(0,pos),coordinates_in_UTM);
                    low_.set(s.substr(pos+1,s.length()-pos-1),coordinates_in_UTM);
                    unary_=false;
                }
                else
                {
                    pos = s.find("<");
                    if(pos!=std::string::npos)
                    {
                        low_.set(s.substr(0,pos),coordinates_in_UTM);
                        high_.set(s.substr(pos+1,s.length()-pos-1),coordinates_in_UTM);
                        unary_=false;
                    }
                    else
                    {
                        high_.set(s,coordinates_in_UTM);
                        unary_=true;
                    }
                    
                }
            }
            bool equals(const PrecedenceRule& other)const
            {
                if(this->unary_!=other.unary_)return false;
                if(!this->high_.equals(other.high_))return false;
                if(this->unary_ && !this->low_.equals(other.low_))return false;
                return true;
            }

            PriorityRoute::boost_box getBoostBox()
            {
                if(unary_)return high_.getBoostBox();
                auto l = low_.getBoostBox();
                auto h = high_.getBoostBox();
                l.min_corner().get<0>();
                const double min0 = (std::min)(l.min_corner().get<0>(),h.min_corner().get<0>());
                const double min1 = (std::min)(l.min_corner().get<1>(),h.min_corner().get<1>());
                const double min2 = (std::min)(l.min_corner().get<2>(),h.min_corner().get<2>());
                const double max0 = (std::max)(l.max_corner().get<0>(),h.max_corner().get<0>());
                const double max1 = (std::max)(l.max_corner().get<1>(),h.max_corner().get<1>());
                const double max2 = (std::max)(l.max_corner().get<2>(),h.max_corner().get<2>());
                return PriorityRoute::boost_box(
                            PriorityRoute::boost_point(min0,min1,min2),
                            PriorityRoute::boost_point(max0,max1,max2)
                            );
            }
        };

         /**
         * @brief PrecedenceSet contains PrecedenceRules, indexed by the area they affect.
         */
        class PrecedenceSet
        {
            private:
            adore::mad::AFeed<PrecedenceRule>* ruleFeed_;

            public:
            PrecedenceSet(){}
            PrecedenceSet(adore::mad::AFeed<PrecedenceRule>* ruleFeed)
            {
               ruleFeed_ = ruleFeed;
            }
            void update(double radius, double x, double y)
            {
                while(ruleFeed_->hasNext())
                {
                    PrecedenceRule r;
                    ruleFeed_->getNext(r);
                    insertRule(r);
                }
                refocus(x-radius,y-radius,x+radius,y+radius); 
            }
			/**
			 * @brief custom equal test for iterators
			 * 
			 * @tparam value_type 
			 * @tparam Tfirst 
			 */
			template<typename value_type,typename Tfirst>
			struct my_equal 
			{ 
				typedef bool result_type; 
				result_type operator() (value_type const& v1, value_type const& v2) const 
				{ 
					return boost::geometry::equals<Tfirst,Tfirst>(v1.first, v2.first) && v1.second == v2.second;
				} 
			}; 
            typedef std::pair<PriorityRoute::boost_box,PrecedenceRule*> idxRegion2Precedence;
            typedef boost::geometry::index::rtree<	idxRegion2Precedence,
                                                    boost::geometry::index::quadratic<16>,
                                                    boost::geometry::index::indexable<idxRegion2Precedence>,
                                                    my_equal<idxRegion2Precedence,PriorityRoute::boost_box>
                                                    > Region2PrecedenceRT;
            template<typename T1, typename T2>
            struct itpair
            {
                T1 first;
                T2 second;
                itpair(T1 first,T2 second):first(first),second(second){}
                T1& current(){return first;}
                T2& end(){return second;}
            };
            typedef itpair<Region2PrecedenceRT::const_query_iterator,Region2PrecedenceRT::const_query_iterator> itRegion2PrecedenceRT;

            private:
            Region2PrecedenceRT precedenceRT_;
            public:

            /**
             * @brief reads a set of precedence rules from a file
             */
            bool readFile(const std::string& filename)
            {
                std::string line;
                std::ifstream file(filename);
                if(!file.is_open())
                {
                    std::cout<<"failed to open file\n";
                    return false;
                }
                while(std::getline(file,line))
                {
                    if (line.empty() || line.at(0) == '#')
                    {
                        continue;
                    }
                    auto rule = parseRule(line);//@TODO: parse coordinate format from file
                    std::cout<<precedenceRT_.size()<<": ";
                    std::cout<<
                        rule->high_.from_(0)<<","<<
                        rule->high_.from_(1)<<","<<
                        rule->high_.from_(2)<<";"<<
                        rule->high_.to_(0)<<","<<
                        rule->high_.to_(1)<<","<<
                        rule->high_.to_(2);
                    if(!rule->unary_)
                    {
                    std::cout<<
                        " > "<<
                        rule->low_.from_(0)<<","<<
                        rule->low_.from_(1)<<","<<
                        rule->low_.from_(2)<<";"<<
                        rule->low_.to_(0)<<","<<
                        rule->low_.to_(1)<<","<<
                        rule->low_.to_(2);
                    }
                    std::cout<<"\n";
                }
                return true;
            }

            /**
             * @brief creates a precedence rule from string and adds it to container.
             */
            PrecedenceRule* parseRule(const std::string& s,bool use_UTM_coordinates = true)
            {
                PrecedenceRule* myrule = new PrecedenceRule();
                myrule->set(s,use_UTM_coordinates);
                precedenceRT_.insert(std::make_pair(myrule->getBoostBox(),myrule));
                return myrule;
            }
            /**
             * @brief inserts a copy of the given rule into container
             */
            void insertRule(const PrecedenceRule& rule)
            {
                PrecedenceRule* myrule = new PrecedenceRule();
                *myrule=rule;
                precedenceRT_.insert(std::make_pair(myrule->getBoostBox(),myrule));
            }
            /**
             * @brief initialize by copying entries
             */
            void init(PrecedenceSet* other)
            {
                std::cout<<"copying...";
                for(auto const& pair : other->precedenceRT_)
                {
                    std::cout<<"+";
                    auto rule = pair.second;
                    insertRule(*rule);
                }
            }
            /**
             * @brief check whether a rule is contained
             */
            bool contains(const PrecedenceRule& rule)
            {
                for(auto it = precedenceRT_.qbegin(boost::geometry::index::intersects(
                                                PriorityRoute::boost_box(
                                                PriorityRoute::boost_point(
                                                    rule.high_.from_(0),
                                                    rule.high_.from_(1),
                                                    rule.high_.from_(2)
                                                ),
                                                PriorityRoute::boost_point(
                                                    rule.high_.from_(0),
                                                    rule.high_.from_(1),
                                                    rule.high_.from_(2)
                                                ))));
                    it!=precedenceRT_.qend();
                    it++)
                    {
                        if(it->second->equals(rule))return true;
                    }
                return false;                
            }
            /**
             * @brief removes a rule from the container
             */
            void eraseRule(PrecedenceRule* rule)
            {
                precedenceRT_.remove(std::make_pair(rule->getBoostBox(),rule));
                delete rule;
            }
            /**
             * @brief returns a subset of rules in a region
             */
            itRegion2PrecedenceRT getRulesInRegion(double x0,double y0,double x1,double y1) const
            {
                static const double guard = 1.0e99;
                auto it = precedenceRT_.qbegin(boost::geometry::index::intersects(
                                            PriorityRoute::boost_box(	
                                                PriorityRoute::boost_point(x0,y0,-guard),
                                                PriorityRoute::boost_point(x1,y1,+guard) 
                                            )
                                        ));
                return itRegion2PrecedenceRT(it,precedenceRT_.qend());
            }
            /**
             * @brief returns all rules
             */
            itRegion2PrecedenceRT getAllRulesIt() const
            {
                static const double guard = 1.0e99;
                auto it = precedenceRT_.qbegin(boost::geometry::index::intersects(
                                            PriorityRoute::boost_box(	
                                                PriorityRoute::boost_point(-guard,-guard,-guard),
                                                PriorityRoute::boost_point(+guard,+guard,+guard) 
                                            )
                                        ));
                return itRegion2PrecedenceRT(it,precedenceRT_.qend());
            }
            
             /**
             * @brief returns all rules
             */
            Region2PrecedenceRT* getAllRules()
            {
                return &precedenceRT_;
            }
            /**
             * @brief returns a subset of rules outside of a region
             */
            itRegion2PrecedenceRT getRulesOutsideRegion(double x0,double y0,double x1,double y1)
            {
                static const double guard = 1.0e99;
                auto it = precedenceRT_.qbegin(boost::geometry::index::disjoint(
                                            PriorityRoute::boost_box(	
                                                PriorityRoute::boost_point(x0,y0,-guard),
                                                PriorityRoute::boost_point(x1,y1,+guard) 
                                            )
                                        ));
                return itRegion2PrecedenceRT(it,precedenceRT_.qend());
            }
            /**
             * @brief removes all rules outside of a region
             */
            void refocus(double x0,double y0,double x1,double y1)
            {
                std::vector<PrecedenceRule*> removeset;
                for(auto it = getRulesOutsideRegion(x0,y0,x1,y1);it.current()!=it.end();it.current()++)
                {
                    removeset.push_back(it.current()->second);
                }
                for(auto it: removeset)eraseRule(it);
            }

            
        };
    }
}
