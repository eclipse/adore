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
#include <adore/mad/vectorbasedvolumetree.h>
namespace adore
{
    namespace mad
    {
        /**
         * Space-time occupancy description in form of a cylinder
         */
        struct OccupancyCylinder
        {
            double rxy_;/**< spacial radius*/
            double x_,y_;/**< spacial center*/
            double t0_,t1_;/**< time interval*/
            double z0_,z1_;/**< elevation interval*/
            OccupancyCylinder(){}
            OccupancyCylinder(double rxy,double x,double y,double t0,double t1,double z0 = 0.0,double z1 = 3.0)
                :rxy_(rxy),x_(x),y_(y),t0_(t0),t1_(t1),z0_(z0),z1_(z1){}
            /**
             * test whether two OccupancyCylinders (this, other) intersect
             * @return true on intersection
             */
            inline bool intersects(const OccupancyCylinder& other)const
            {
                if(!adore::mad::overlaps(t0_,t1_,other.t0_,other.t1_))return false;
                if(!adore::mad::overlaps(z0_,z1_,other.z0_,other.z1_))return false;
                const double dx = other.x_-x_;
                const double dy = other.y_-y_;
                const double R = other.rxy_+rxy_;
                return dx*dx+dy*dy<=R*R;
            }
            /**
             * increase the inclusion radius and intervals: increase this, so that it includes other
             */
            void bound(const OccupancyCylinder& other)
            {
                const double dx = other.x_-x_;
                const double dy = other.y_-y_;
                const double R = (std::sqrt)(dx*dx+dy*dy) + other.rxy_;
                rxy_ = (std::max)(rxy_,R);
                t0_ = (std::min)(t0_,other.t0_);
                t1_ = (std::max)(t1_,other.t1_);
                z0_ = (std::min)(z0_,other.z0_);
                z1_ = (std::max)(z1_,other.z1_);
            }
        };

        struct OccupancyCylinderBoundingFunctor
        {
            typedef std::pair<std::pair<int,int>,OccupancyCylinder> IndexedVolumeType;
            typedef std::vector<IndexedVolumeType> VolumeVector;
            /**
             *  initializes bounding_volume to bound all OccupancyCylinders in the given interval:
             *  the average of first and last point is used to center the bounding volume
             */
            IndexedVolumeType operator()(const VolumeVector& base,int i0,int i1)const
            {
                IndexedVolumeType iv;
                iv.second = base[i0].second;
                iv.second.x_ = 0.5 * (base[i0].second.x_+base[i1].second.x_);
                iv.second.y_ = 0.5 * (base[i0].second.y_+base[i1].second.y_);
                for(int i=i0+1;i<=i1;i++)iv.second.bound(base[i].second);
                iv.first.first=i0;
                iv.first.second=i1;
                return iv;
            }
        };

        class OccupancyCylinderTree:public adore::mad::VectorBasedVolumeTree<OccupancyCylinder,OccupancyCylinderBoundingFunctor>
        {
            private:
            /**
             * computes distance between two cylinders, if they do not overlap. In case they overlap, the overlap distance is returned.
             */
            struct XYIntrusionFunctor
            {
                double operator()(const OccupancyCylinder& a,const OccupancyCylinder& b)const
                {
                    const double guard = 1.e10;
                    if(!adore::mad::overlaps(a.t0_,a.t1_,b.t0_,b.t1_))return guard;
                    if(!adore::mad::overlaps(a.z0_,a.z1_,b.z0_,b.z1_))return guard;
                    const double dx = a.x_-b.x_;
                    const double dy = a.y_-b.y_;
                    return std::sqrt(dx*dx+dy*dy)-a.rxy_-b.rxy_;
                }
            };
            /**
             * returns earliest time of overlap if two cylinders coincide spacially
             */
            struct EarliestOverlapFunctor
            {
                double max_time_;
                EarliestOverlapFunctor(double max_time):max_time_(max_time){}
                double operator()(const OccupancyCylinder& a,const OccupancyCylinder& b)const
                {
                    if(!adore::mad::overlaps(a.t0_,a.t1_,b.t0_,b.t1_))return max_time_;
                    if(!adore::mad::overlaps(a.z0_,a.z1_,b.z0_,b.z1_))return max_time_;
                    const double dx = a.x_-b.x_;
                    const double dy = a.y_-b.y_;
                    const double d = std::sqrt(dx*dx+dy*dy)-a.rxy_-b.rxy_;
                    if(d<0.0)
                    {
                        return (std::max)(a.t0_,b.t0_);
                    }
                    else
                    {
                        return max_time_;
                    }
                }
            };
            /**
             * computes the time, by which an overlap in xy,z is missed:
             * In case an overlap exists, the overlap time is returned as a negative value
             */
            struct TimeEncroachmentFunctor
            {
                double operator()(const OccupancyCylinder& a,const OccupancyCylinder& b)const
                {
                    const double guard = 1.e10;
                    if(!adore::mad::overlaps(a.z0_,a.z1_,b.z0_,b.z1_))return guard;
                    const double dx = a.x_-b.x_;
                    const double dy = a.y_-b.y_;
                    const double R = a.rxy_+b.rxy_;
                    if(dx*dx+dy*dy>R*R)return guard;
                    if(a.t0_<=b.t0_ && a.t1_>=b.t1_)return -(b.t1_-b.t0_);//b is included in a 
                    if(b.t0_<=a.t0_ && b.t1_>=a.t1_)return -(a.t1_-a.t0_);//a is included in b
                    if(a.t0_<=b.t0_ && b.t0_<=a.t1_)return -(a.t1_-b.t0_);//b starts in a 
                    if(b.t0_<=a.t0_ && a.t0_<=b.t1_)return -(b.t1_-a.t0_);//a starts in b
                    //no overlap between intervals
                    if(a.t1_<=b.t0_)return b.t0_-a.t1_;//a ends before b: return time gap from a to b
                    return a.t0_-b.t1_;//b ends before a: return time gap from b to a
                }
            };

            public:
            /**
             * evaluates, whether the two objects overlap in x,y and z at a certain point of time
             */
            bool collidesWith(const OccupancyCylinderTree& other)const 
            {
                XYIntrusionFunctor f;
                const double collision_distance = 0.0001;
                double result;
                return compute_min(other,collision_distance,result,f);
            }
            /**
             * evaluates the post-encroachment time between two objects, if their xyz projections overlap
             * @param max_time maximum time until which to evaluate
             * @return true, if an xyz projection overlap occurs
             * @param result contains postencroachmenttime, if overlap occured
             */
            bool getPostEncroachmentTime(const OccupancyCylinderTree& other,double max_time,double& result)const
            {
                TimeEncroachmentFunctor f;
                bool rv = compute_min(other,max_time,result,f);
                if(rv)result=(std::max)(result,0.0);
                return rv;
            }
            /**
             * evaluates the earliest collision time
             * @param max_time maximum time until which to evaluate
             * @return true if collision occurs
             * @param result contains earliest collision time, if overlap occured
             */
            bool getEarliestCollisionTime(const OccupancyCylinderTree& other,double max_time,double& result)const
            {
                EarliestOverlapFunctor f(max_time);
                bool rv = compute_min(other,max_time,result,f);
                return rv;
            }
        };

    }
}