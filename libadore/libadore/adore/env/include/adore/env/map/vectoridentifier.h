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
         * VectorIdentifier identifies an object by a vector from a certain location to a certain location, with a given precision of the start and end locations.
         */
        struct VectorIdentifier
        {
            public:
                typedef boost::geometry::model::point<double,6,boost::geometry::cs::cartesian> Tboost_vectors;
                typedef boost::geometry::model::point<double,3,boost::geometry::cs::cartesian> Tboost_point;
            private:
                Tboost_vectors id_;
                Tboost_vectors precision_;
            public:
                VectorIdentifier(double x0,double y0,double z0,double x1,double y1,double z1)
                {
                    id_.set<0>(x0);
                    id_.set<1>(y0);
                    id_.set<2>(z0);
                    id_.set<3>(x1);
                    id_.set<4>(y1);
                    id_.set<5>(z1);
                    precision_.set<0>(1.0);
                    precision_.set<1>(1.0);
                    precision_.set<2>(3.0);
                    precision_.set<3>(1.0);
                    precision_.set<4>(1.0);
                    precision_.set<5>(3.0);
                }
                bool equals(const VectorIdentifier& other)const
                {
                    return 
                          (std::abs)(this->id_.get<0>()-other.id_.get<0>())<this->precision_.get<0>()
                    &&    (std::abs)(this->id_.get<1>()-other.id_.get<1>())<this->precision_.get<1>()
                    &&    (std::abs)(this->id_.get<2>()-other.id_.get<2>())<this->precision_.get<2>()
                    &&    (std::abs)(this->id_.get<3>()-other.id_.get<3>())<this->precision_.get<3>()
                    &&    (std::abs)(this->id_.get<4>()-other.id_.get<4>())<this->precision_.get<4>()
                    &&    (std::abs)(this->id_.get<5>()-other.id_.get<5>())<this->precision_.get<5>();
                }
                const Tboost_vectors& getPrecision()const{return precision_;}
                const Tboost_vectors& getID()const {return id_;}
                Tboost_point getFrom()const{return Tboost_point(id_.get<0>(),id_.get<1>(),id_.get<2>());}
                Tboost_point getTo()const {return Tboost_point(id_.get<3>(),id_.get<4>(),id_.get<5>());}
                
        };
    }
}
