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
 *   Thomas Lobig - initial implementation
 ********************************************************************************/

#pragma once
#include <adore/mad/com_patterns.h>
#include <adore/mad/arraymatrixtools.h>
#include <adore/params/afactory.h>
#include <adore/env/afactory.h>
#include <adore/env/map/speedlimit.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/apps/if_plotlab/plot_shape.h>
// #include <cstdio>
#include <sstream>
#include <map>

namespace adore
{
  namespace apps
  {
    /**
     * @brief an optimzed plotting application to plot annotations like speed limits
     * 
     */
    class PlotRoadAnnotations
    {
      private:
      adore::mad::AReader<adore::env::VehicleMotionState9d>* motion_state_reader_;
      adore::env::AFactory::TSpeedLimitFeed* speedlimit_feed_;

      DLR_TS::PlotLab::AFigureStub* figure_;
      adore::env::VehicleMotionState9d vehicle_state_;
      std::string prefix_;
      std::unordered_map<adore::env::TSpeedLimitID,adore::env::SpeedLimit> limits_;

      public:
      PlotRoadAnnotations(DLR_TS::PlotLab::AFigureStub* figure,std::string prefix)
      {
        motion_state_reader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
        speedlimit_feed_ = adore::env::EnvFactoryInstance::get()->getSpeedLimitFeed();
        figure_ = figure;      
        prefix_ = prefix;
      }

      ~PlotRoadAnnotations()
       {
       }

      void run()
      {
        if(motion_state_reader_->hasUpdate())
        {
          motion_state_reader_->getData(vehicle_state_);
        }
        adore::env::TSpeedLimitBundle limits_update;
        
        while (speedlimit_feed_->hasNext())
        {
          adore::env::SpeedLimit limit;
          speedlimit_feed_->getNext(limit);
          limits_update.push_back(limit);
        }

        for (auto limit : limits_update)
        {
          limits_.insert_or_assign(limit.id,limit);
        }

        plotSpeedLimits();
      
      }

      virtual void plotSpeedLimits()
      {          
        // adore::env::VehicleMotionState9d vehicle_state;
        // motion_state_reader_->getData(vehicle_state);
        auto position = vehicle_state_.getTime();
        auto time = vehicle_state_.getTime();

        std::stringstream buffer1;
        std::stringstream buffer2;

        for( auto [key, limit] : limits_)
        {
          buffer1.clear();
          buffer2.clear();
          buffer1 << "speedlimit_start/id" << key;
          buffer2 << "X-" << limit.value << " - id:" << key;
          // std::string hash
          // figure_->plotText(buffer1.str(),limit.startX, limit.startY,buffer2.str());
          
          buffer1.clear();
          buffer1 << "speedlimit_line/id" << limit.id;
          adore::PLOT::plotLine(buffer1.str(),limit.startX,limit.startY,limit.stopX,limit.stopY,0.5,"LineColor=0,0,1",figure_);
          // adore::PLOT::plotLine(buffer1.str(),limit.startX,limit.startY,limit.stopX,limit.stopY,0.5,"LineWidth=2;LineColor=0,0.2,1",figure_);

          // buffer1.clear();
          // buffer1 << "speedlimit_stop/id" << limit.id;
          // figure_->plotText(buffer1.str(),limit.startX, limit.startY,buffer2.str());
        }

        // removeDistanceBased(limits_,plot_distance);

        // while(tl_controller_feed_->hasNext())
        // {
        //   adore::PLOT::plotLine(ss_hashtag.str(),x_from,y_from,x_to,y_to,2,ss_color.str(),figure_);
        // }
      }

      void removeDistanceBased(std::unordered_map<std::string, std::pair<double,double>> &visible_objects, int distance)
      {
        // for (auto [key, limit] : limits_)
        // {
        //   double x = vehicle_state_.getX();
        //   double y = vehicle_state_.getY();
        //   if ((limit.startX - x)*(limit.startX - x)
        // }
      //   //remove tiles which are no longer visible
      //   std::vector<std::string> remove;

      //     adore::env::VehicleMotionState9d vehicle_state;
      //     positionReader_->getData(vehicle_state);
      //     double v_pos_x = vehicle_state.getX();
      //     double v_pos_y = vehicle_state.getY();

      //   for(auto it = visible_objects.begin();it!=visible_objects.end();it++)
      //   {
      //     auto coord = (*it).second;
      //     double distance_to_object = getDistanceToPoint(coord);
      //     if(distance_to_object > distance) remove.push_back((*it).first);
      //   }

      //   for(auto it = remove.begin();it!=remove.end();it++)
      //   { 
      //     figure_->erase((*it));
      //     visible_objects.erase((*it));
      //   }
        
      }      
    
    };
  }
}