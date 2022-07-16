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
 *   Daniel Heß - compute constraints, which help to avoid crosstraffic
 ********************************************************************************/


#pragma once

#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/env/threelaneviewdecoupled.h>
#include <unordered_set>
#include <vector>


namespace adore
{
  namespace apps
  {

    
    /**
     * @brief 
     */
    class CrosstrafficConstraintProvider
    {
      private:
        struct Conflict
        {
            double x,y,s,n,t0,t1,rxy;
            adore::view::TrafficObject::TTrackingID id;        
            Conflict(double x,double y,double s,double n,double t0,double t1,double rxy,adore::view::TrafficObject::TTrackingID id)
                :x(x),y(y),s(s),n(n),t0(t0),t1(t1),rxy(rxy),id(id){}
        };
      adore::env::ThreeLaneViewDecoupled three_lanes_;/**<lane-based representation of environment*/
      adore::env::AFactory::TOCPredictionSetReader* prediction_reader_;
      adore::env::AFactory::TOCPredictionSetWriter* conflict_set_writer_;
      
      adore::env::OccupancyCylinderPredictionSet ocp_set_;
      adore::env::AFactory::TVehicleMotionStateReader* vehicle_state_reader_;
      adore::params::APVehicle* pvehicle_;
      adore::params::APLongitudinalPlanner* plplanner_;
      double max_centerline_distance_;/**@TODO parameter*/

      public:
      virtual ~CrosstrafficConstraintProvider()
      {
        delete prediction_reader_;
        delete vehicle_state_reader_;
      }
      CrosstrafficConstraintProvider()
      {
        prediction_reader_ = adore::env::EnvFactoryInstance::get()->getExpectedPredictionSetReader();
        conflict_set_writer_ = adore::env::EnvFactoryInstance::get()->getConflictSetWriter();
        vehicle_state_reader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
        max_centerline_distance_ = 1.0;
        pvehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
        plplanner_ = adore::params::ParamsFactoryInstance::get()->getLongitudinalPlanner();
      }
      /**
       * @brief update data, views and recompute constraints
       * 
       */
      virtual void update() 
      {
        adore::env::VehicleMotionState9d vehicle_state;
        if(!vehicle_state_reader_->hasData())return;
        vehicle_state_reader_->getData(vehicle_state);

        three_lanes_.update();
        if(prediction_reader_->hasUpdate())
        {
            ocp_set_.clear();
            prediction_reader_->getData(ocp_set_);
        }

        if( three_lanes_.getCurrentLane()!=nullptr
        &&  three_lanes_.getCurrentLane()->isValid() )
        {
            double s_lower_bound,s_ego,n_ego,ds_ego;
            three_lanes_.getCurrentLane()->toRelativeCoordinates(vehicle_state.getX(),vehicle_state.getY(),s_ego,n_ego);
            ds_ego = vehicle_state.getvx();
            double acc  = (std::abs)(plplanner_->getAccLB()-0.05);
            double distance_to_point = plplanner_->getStopDistanceToConflictPoint();
            s_lower_bound = s_ego + 0.5 * ds_ego * ds_ego / acc + pvehicle_->get_a() + pvehicle_->get_b() + pvehicle_->get_c() + distance_to_point;

            std::unordered_set<adore::view::TrafficObject::TTrackingID> parallel_traffic_ids;

            add_ids(parallel_traffic_ids,three_lanes_.getCurrentLane()->getOnLaneTraffic());
            if( three_lanes_.getLeftLaneChange()->getTargetLane()!=nullptr 
            &&  three_lanes_.getLeftLaneChange()->getTargetLane()->isValid() )
            {
                add_ids(parallel_traffic_ids,three_lanes_.getLeftLaneChange()->getTargetLane()->getOnLaneTraffic());
            }
            if( three_lanes_.getRightLaneChange()->getTargetLane()!=nullptr 
            &&  three_lanes_.getRightLaneChange()->getTargetLane()->isValid() )
            {
                add_ids(parallel_traffic_ids,three_lanes_.getRightLaneChange()->getTargetLane()->getOnLaneTraffic());
            }

            std::vector<Conflict> conflict_set;

            for(auto& element:ocp_set_)
            {
                if(parallel_traffic_ids.find(element.trackingID_)==parallel_traffic_ids.end())
                {
                    // for(auto& cylinder:element.occupancy_.getLevel(element.occupancy_.getLevelCount()-1))
                    for(auto& cylinder:element.occupancy_.getLevel(0))
                    {
                        double x,y,s,n,xc,yc,zc;
                        x = cylinder.second.x_;
                        y = cylinder.second.y_;
                        three_lanes_.getCurrentLane()->toRelativeCoordinates(x,y,s,n);
                        if(s - cylinder.second.rxy_<s_lower_bound)continue;
                        three_lanes_.getCurrentLane()->toEucledianCoordinates(s,0.0,xc,yc,zc);
                        double d = std::sqrt((x-xc)*(x-xc)+(y-yc)*(y-yc));
                        if(d-cylinder.second.rxy_>max_centerline_distance_)continue;
                        // if(s + cylinder.second.rxy_< s_ego - pvehicle_->get_d() + ds_ego * (std::max)(0.0,cylinder.second.t0_-vehicle_state.getTime()))continue;
                        conflict_set.push_back(Conflict(xc,yc,s - cylinder.second.rxy_,n,cylinder.second.t0_,cylinder.second.t1_,cylinder.second.rxy_,element.trackingID_));
                    }
                }
            }

            std::sort(conflict_set.begin(),conflict_set.end(),[](Conflict a,Conflict b){return a.s<b.s;});

            std::cout<<"-----------------------------------------"<<std::endl;
            for(auto& c:conflict_set)
            {
                std::cout<<"s="<<c.s-s_lower_bound<<", t0="<<c.t0-vehicle_state.getTime()<<", t1="<<c.t1-vehicle_state.getTime()<<std::endl;
                break;
            }

            std::vector<adore::env::OccupancyCylinderPrediction> ocps;
            if(conflict_set.size()>0)
            {
                adore::env::OccupancyCylinderPredictionSet conflictset_ocp;//reusing class/data transfer
                auto item = conflict_set[0];
                adore::env::OccupancyCylinderPrediction ocp;
                adore::mad::OccupancyCylinder oc(item.rxy,item.x,item.y,item.t0,item.t1);
                ocp.occupancy_.insert(oc);
                ocp.trackingID_ = item.id;
                ocp.branchID_ = 0;
                ocp.predecessorID_ = -1;
                ocp.confidence_ = 1.0;
                ocps.push_back(ocp);
            }
            conflict_set_writer_->write(ocps);
        }

      }
      private:
      void add_ids(std::unordered_set<adore::view::TrafficObject::TTrackingID>& parallel_traffic_ids,const adore::view::TrafficQueue& queue)
      {
          for(const auto& object:queue)
          {
              parallel_traffic_ids.insert(object.getTrackingID());
          }
      }
    };
  }
}