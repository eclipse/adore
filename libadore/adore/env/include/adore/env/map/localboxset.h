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
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace adore
{
    namespace env
    {
        /**
         * LocalBoxSet aggregates a set of objects representable by bounding boxes.
         * TObject: type of collected objects
         * TBoxGen: generates box-shaped impact region for objects
         * TComparator: compares objects for equality
         * On insertion, the existance of a similar object is checked: This container allows no duplicates according to TComparator.
         * If objects are identified with a certain tolerance, the TBoxGen class has to use the same tolerance in box generation, in order to find duplicates in insert or find or contains.
         */
        template<typename TObject,typename TBoxGen,typename TComparator>
        class LocalBoxSet
        {
            public:
                typedef boost::geometry::model::point<double,3,boost::geometry::cs::cartesian> Tboost_point;
                typedef boost::geometry::model::box<Tboost_point> Tboost_box;
                typedef std::pair<Tboost_box,TObject*> TBoxPointerPair;
                struct EqualFunctor
                { 
                    TComparator compare_;
                    bool operator() (TBoxPointerPair const& v1, TBoxPointerPair const& v2) const 
                    { 
                        return boost::geometry::equals<Tboost_box,Tboost_box>(v1.first, v2.first)&&(v1.second == v2.second);
                    }
                };
                typedef boost::geometry::index::rtree<	TBoxPointerPair,
                                                        boost::geometry::index::quadratic<16>,
                                                        boost::geometry::index::indexable<TBoxPointerPair>,
                                                        EqualFunctor > Trtree;
                typedef typename Trtree::const_query_iterator Trtreeit;
                struct itpair
                {
                    Trtreeit first,second;
                    itpair(Trtreeit first,Trtreeit second):first(first),second(second){}
                    Trtreeit& current(){return first;}
                    Trtreeit& end(){return second;}
                };
            private:
                Trtree data_;/** < the container */
                TBoxGen boxgen_;/** < object responsible for bounding box computation */
                TComparator compare_;/** < object responsible for comparison */
            public:
                /**
                 * Inserts a TObject for a region specified by TBoxGen.
                 * If the object already exists (equality defined by TComparator), the old object is replaced.
                 */
                void insert(const TObject& object)
                {
                    TObject* old = find(object);
                    if(old !=0)
                    {
                        remove(old);
                    }
                    TObject* copy = new TObject(object);
                    Tboost_box box  = boxgen_(copy);
                    TBoxPointerPair pair = std::make_pair(box,copy);
                    data_.insert(pair);
                }
                /**
                 * Searches for an object pointer, which is equal to object according to TComparator.
                 * If found, returns valid pointer, otherwise 0. 
                 */
                TObject* find(const TObject& object)
                {
                    Tboost_box box = boxgen_(&object);
                    for(auto it = data_.qbegin(boost::geometry::index::intersects(box));
                             it!= data_.qend();
                             it++)
                    {
                        if(compare_(it->second,&object))return it->second;
                    }
                    return 0;
                }
                /**
                 * Returns true, if overall container has object, which is equal to function parameter, according to TComparator
                 */
                bool contains(const TObject& object)
                {
                    return find(object)!=0;                
                }
                /**
                 * Remove an object from the overall container
                 */
                void remove(TObject* object)
                {
                    data_.remove(std::make_pair(boxgen_(object),object));
                    delete object;
                }
                /**
                 * get the number of contained objects 
                 */
                int size()const{return data_.size();}

                /**
                 * Identifies all objects overlapping or included in a box-shaped region with llc (x0,y0) and urc (x1,y).
                 * Returns an iterator pair. The first entry of the pair is the current position and the second entry the end position.
                 */
                itpair getObjectsInRegion(double x0,double y0,double x1,double y1)
                {
                    static const double guard = 1.0e99;
                    auto it = data_.qbegin(boost::geometry::index::intersects(
                                                Tboost_box(	
                                                    Tboost_point(x0,y0,-guard),
                                                    Tboost_point(x1,y1,+guard) 
                                                )
                                            ));
                    return itpair(it,data_.qend());
                }
                /**
                 * Identifies all objects not overlapping or included in a box-shaped region with llc (x0,y0) and urc (x1,y).
                 * Returns an iterator pair. The first entry of the pair is the current position and the second entry the end position.
                 */
                itpair getObjectsOutsideRegion(double x0,double y0,double x1,double y1)
                {
                    static const double guard = 1.0e99;
                    auto it = data_.qbegin(boost::geometry::index::disjoint(
                                                Tboost_box(	
                                                    Tboost_point(x0,y0,-guard),
                                                    Tboost_point(x1,y1,+guard) 
                                                )
                                            ));
                    return itpair(it,data_.qend());
                }
                /**
                 * Deletes and removes all objects from container, which are not included in or overlapping given box  with llc (x0,y0) and urc (x1,y).
                 */
                void refocus(double x0,double y0,double x1,double y1)
                {
                    std::vector<TObject*> removeset;
                    for(auto itp = getObjectsOutsideRegion(x0,y0,x1,y1);
                        itp.current()!=itp.end();
                        itp.current()++)
                    {
                        removeset.push_back(itp.current()->second);
                    }
                    for(auto obj: removeset)remove(obj);
                }
        };
    }
}