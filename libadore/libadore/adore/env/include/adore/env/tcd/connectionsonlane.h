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
#include <adore/view/alane.h>
#include <adore/env/tcd/controlledconnection.h>
#include <adore/view/alimitlineenroute.h>
#include <vector>
#include <algorithm>

namespace adore
{
namespace env
{

    /**
     * ConnectionsOnLane maps ControlledConnections ("signalized parts of the road") to a lane.
     * ConnectionsOnLane also implements adore::view::ALimitLineEnRoute to provide a view
     */
    class ConnectionsOnLane:public  adore::view::ALimitLineEnRoute
    {
        public:
            struct MappedConnection
            {
                private:
                    double s0_;/**< distance along lane: start of connection*/
                    double s1_;/**< distance along lane: end of connection*/
                    ControlledConnection* con_;/**< the connection*/
                public:
                    MappedConnection(double s0,double s1,ControlledConnection* con):s0_(s0),s1_(s1),con_(con){}
                    double getS0()const {return s0_;}
                    double getS1()const {return s1_;}
                    ControlledConnection* getConnection(){return con_;}
            };

        private:
            adore::view::ALane* lane_;      /**< the associated lane*/
            ControlledConnectionSet* conset_;/**< the set of known connections*/
            std::vector<MappedConnection> mappedcons_;/**< a list of connections on the lane*/
        public://methods from adore::view::ALimitLineEnRoute
            virtual bool hasLimitLine(double s0)override
            {
                for(auto& con:mappedcons_)
                {
                    if(con.getS1()>=s0)return true;
                }
                return false;
            }
            virtual adore::view::LimitLine getLimitLine(double t0,double s0)override 
            {
                adore::view::LimitLine ll;
                for(auto& con:mappedcons_)
                {
                    if(con.getS1()>s0)
                    {
                        ll.setCurrentState(con.getConnection()->convert(con.getConnection()->getState_byMinTime(t0)));
                        ll.setProgress(con.getS0());
                        return ll;
                    }
                }
                return ll;
            }

        public:
            ConnectionsOnLane(adore::view::ALane* lane,adore::env::ControlledConnectionSet* conset)
                :lane_(lane),conset_(conset){}
            /**
             *  (Re-)compute the mapping of connections to associated lane.
             */
            void update()
            {
                mappedcons_.clear();
                if(lane_->isValid())
                {
                    //find all connections with start and end point in lane, add them to vector
                    for(auto itpair = conset_->getAllConnections();//@TODO: get bounding box of lane
                        itpair.current()!=itpair.end();
                        itpair.current()++)
                    {
                        ControlledConnection* con = itpair.current()->second;
                        auto from = con->getID().getFrom();
                        auto to = con->getID().getTo();
                        double s0,n0,s1,n1;
                        lane_->toRelativeCoordinates(from.get<0>(),from.get<1>(),s0,n0);
                        if( lane_->getOffsetOfRightBorder(s0)<= n0 && n0<=lane_->getOffsetOfLeftBorder(s0) )
                        {
                            lane_->toRelativeCoordinates(to.get<0>(),to.get<1>(),s1,n1);
                            if( lane_->getOffsetOfRightBorder(s1)<= n1 && n1<=lane_->getOffsetOfLeftBorder(s1) )
                            {
                                if((from.get<0>()==to.get<0>() && from.get<1>()==to.get<1>()) || s0<s1)
                                {
                                    mappedcons_.push_back(MappedConnection(s0,s1,con));
                                }
                            }
                        }
                    }
                    //sort vector by start point
                    std::sort(mappedcons_.begin(), mappedcons_.end(), [](const MappedConnection& a,const MappedConnection& b) {return a.getS0()<=b.getS0();});
                }
            }
    };

}
}