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

#include <adore/mad/com_patterns.h>
#include <adore/mad/adoremath.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <adore/env/map/vectoridentifier.h>
#include <adore/env/map/localboxset.h>
#include <adore/view/limitline.h>
#include <adore/env/ego/vehiclemotionstate9d.h>

namespace adore
{
    namespace env
    {
        /**
         * Connection state represents whether and how a connection path of a junction may be used.
         */
        namespace ConnectionState
        {
            enum EConnectionState
            {
                UNAVAILABLE = 0,
                DARK = 1,                               
                STOP___THEN___PROCEED = 2,
                STOP___AND___REMAIN = 3,                /**< often equivalent to RED*/
                PRE___MOVEMENT = 4,                     /**< often equivalent to RED-YELLOW*/
                PERMISSIVE___MOVEMENT___ALLOWED = 5,    /**< often equivalent to GREEN, yield to head-on traffic*/
                PROTECTED___MOVEMENT___ALLOWED = 6,     /**< often equivalent to GREEN, e.g. green turn arrow*/
                PERMISSIVE__CLEARANCE = 7,              /**< often equivalent to YELLOW*/
                PROTECTED__CLEARANCE = 8,
                CAUTION___CONFLICTING___TRAFFIC = 9,
            };
        }

        /**
         * ConnectionStateEvent is similar to itsg5-SPATEM "MovementEvent": It combines a state with time information about the probable state change.
         * In its current version only time and state are included, future versions will include a more complete representation of the SPATEM intend.
         */
        struct ConnectionStateEvent
        {
            private:
            double minEndTime_;                         /**< earliest time at which this state invalidates*/
            double maxEndTime_;                         /**< latest time at which this state invalidates, optional*/
            double likelyTime_;                         /**< likeliest time at which this state invalidates, optional*/
            bool maxEndTime_present_;                   /**< if true, a reasonable value is contained in maxEndTime*/
            bool likelyTime_present_;                   /**< if true, a reasonable value is contained in likelyTime*/
            ConnectionState::EConnectionState state_;   /**< the traffic light or equivalent controller state for the connection*/
            public:
            ConnectionStateEvent(ConnectionState::EConnectionState state,double minEndTime,bool maxEndTime_present,bool likelyTime_present,double maxEndTime,double likelyTime)
            :minEndTime_(minEndTime),maxEndTime_(maxEndTime),likelyTime_(likelyTime),maxEndTime_present_(maxEndTime_present),likelyTime_present_(likelyTime_present),state_(state){}
            double getMinEndTime()const {return minEndTime_;}
            double getMaxEndTime()const {return maxEndTime_;}
            double getMaxOrMinEndTime()const{return maxEndTime_present_?maxEndTime_:minEndTime_;}
            double getLikelyTime()const {return likelyTime_;}
            bool hasMaxEndTime()const {return maxEndTime_present_;}
            bool hasLikelyTime()const {return likelyTime_present_;}
            ConnectionState::EConnectionState getState()const{return state_;}
        };


        /**
         * ControlledConnection aggregates all known states of a tcd-managed connection over time
         * A VectorIdentifier identifies a connection path through a junction by the start location (entering the junction) and end position (leaving the junction).
         */
        class ControlledConnection
        {
            public:
                typedef adore::env::VectorIdentifier TID;
            private:
                TID id_;/**< from-to-coordinate tuple identifies the connection*/
            public:
                std::vector<ConnectionStateEvent> data_;/**< list of switching events*/
                ControlledConnection()
                    :id_(0.0,0.0,0.0,0.0,0.0,0.0)
                {
                }
                ControlledConnection(double x0,double y0,double z0,double x1,double y1,double z1)
                    :id_(x0,y0,z0,x1,y1,z1)
                {
                }
                void setID(double x0,double y0,double z0,double x1,double y1,double z1)
                {
                    id_ = adore::env::VectorIdentifier(x0,y0,z0,x1,y1,z1);
                }
                const TID& getID()const{return id_;}
                /**
                 * insert a state
                 */
                void insertStateEvent(const ConnectionStateEvent& e)
                {
                    data_.push_back(e);
                }
                /**
                 * remove all previously known events
                 */
                void clear()
                {
                    data_.clear();
                }
                /**
                 * get the state at a certain time, following min switching time predictions
                 */
                ConnectionState::EConnectionState getState_byMinTime(double t)const
                {
                    ConnectionState::EConnectionState state = adore::env::ConnectionState::UNAVAILABLE;
                    for( ConnectionStateEvent e: data_)
                    {
                        if( t < e.getMinEndTime() )
                        {
                            state = e.getState();
                            break;
                        }
                    }
                    return state;
                }
                //@TODO: add additional state evaluation functions for max/likely
                /**
                 * convert from env naming convention to view naming convention
                 */
                adore::view::LimitLine::EState convert(ConnectionState::EConnectionState s)
                {
                    return (adore::view::LimitLine::EState)(int)s;
                }


        };

        /**
         * A set of ControlledConnection, maintained in an R-Tree for spatial access
         */
        class ControlledConnectionSet
        {
        private:
            struct Comparator
            {
                bool operator()(const ControlledConnection* a,const ControlledConnection* b)const 
                {
                    return a->getID().equals(b->getID());
                }
            };
            struct BoxGen
            {
                typedef boost::geometry::model::point<double,3,boost::geometry::cs::cartesian> Tboost_point;
                typedef boost::geometry::model::box<Tboost_point> Tboost_box;
                Tboost_box operator()(const ControlledConnection* con)const
                {
                    const VectorIdentifier::Tboost_vectors& vectors = con->getID().getID();
                    const VectorIdentifier::Tboost_vectors& precision = con->getID().getPrecision(); 
                    Tboost_box box;
                    box.min_corner().set<0>((std::min)(vectors.get<0>()-precision.get<0>(),vectors.get<3>()-precision.get<3>()));
                    box.min_corner().set<1>((std::min)(vectors.get<1>()-precision.get<1>(),vectors.get<4>()-precision.get<4>()));
                    box.min_corner().set<2>((std::min)(vectors.get<2>()-precision.get<2>(),vectors.get<5>()-precision.get<5>()));
                    box.max_corner().set<0>((std::max)(vectors.get<0>()+precision.get<0>(),vectors.get<3>()+precision.get<3>()));
                    box.max_corner().set<1>((std::max)(vectors.get<1>()+precision.get<1>(),vectors.get<4>()+precision.get<4>()));
                    box.max_corner().set<2>((std::max)(vectors.get<2>()+precision.get<2>(),vectors.get<5>()+precision.get<5>()));
                    return box;
                }
            };
        public: 
            typedef LocalBoxSet<ControlledConnection,BoxGen,Comparator> TLocalBoxSet;
            typedef adore::mad::AFeed<ControlledConnection> TConFeed;

        private:
            std::vector<TConFeed*> feeds_;
            TLocalBoxSet data_;
            double max_value_;
        public:
            /**
             * @brief Constructor for ControlledConnectionSet stores pointer to update feeds
             * @param connection_feed required, traffic light information updates
             * @param checkpoint_feed optional, connect to internal checkpoint updates: Use checkpoints only if required. For example a traffic prediction module must not connect to vehicle internal checkpoints.
             */
            ControlledConnectionSet(TConFeed* connection_feed,TConFeed* checkpoint_feed=nullptr)
            {
                max_value_ = 1e10;
                feeds_.push_back(connection_feed);
                if(checkpoint_feed!=nullptr)feeds_.push_back(checkpoint_feed);
            }
            ~ControlledConnectionSet()
            {
                for(TConFeed* feed:feeds_)delete feed;
                feeds_.clear();
            }
            /**
             * Update method accesses the controlled connection feed (trafficlight)  
             * to maintain a set of information about controlled connections in the vicinity of the vehicle.
             * Information for connections with distance bigger than radius to (X,Y) is discarded.
             */
            virtual void update(double radius, double X,double Y)
            {
                for(auto feed:feeds_)
                {
                    while(feed->hasNext())
                    {
                        ControlledConnection con;
                        feed->getNext(con);
                        data_.insert(con);
                    }
                }
                data_.refocus(X-radius,Y-radius,X+radius,Y+radius);
            }
            TLocalBoxSet::itpair getConnectionsInRegion(double x0,double y0,double x1,double y1)
            {
                return data_.getObjectsInRegion(x0,y0,x1,y1);
            }
            int size()const{return data_.size();}
            TLocalBoxSet::itpair getAllConnections()
            {
                return getConnectionsInRegion(-max_value_,-max_value_,max_value_,max_value_);
            }
        };

        /**
         * @brief Specialization of ControlledConnectionSet: Filters connections in range of *ego*.
         */
        class ControlledConnectionSet4Ego: public ControlledConnectionSet
        {
            public:
    			typedef adore::mad::AReader<VehicleMotionState9d> TVehicleMotionStateReader;
            private:
                TVehicleMotionStateReader* xreader_;
                VehicleMotionState9d x_;
            public:
                ControlledConnectionSet4Ego(TVehicleMotionStateReader* xreader,TConFeed* connection_feed,TConFeed* checkpoint_feed=nullptr)
                    :ControlledConnectionSet(connection_feed,checkpoint_feed),xreader_(xreader)
                {
                }
                virtual void update()
                {
                    xreader_->getData(x_);
                    ControlledConnectionSet::update(200.0,x_.getX(),x_.getY());//@TODO read parameter r
                }
        };
    }
}