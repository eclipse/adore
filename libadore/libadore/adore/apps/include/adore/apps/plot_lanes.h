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
 *   Daniel Heß - initial implementation
 *   Thomas Lobig
 ********************************************************************************/

#pragma once
#include <adore/apps/if_plotlab/geoTiles.h>
#include <adore/apps/if_plotlab/geoTiles_config.h>
#include <adore/apps/if_plotlab/plot_border.h>
#include <adore/apps/if_plotlab/plot_shape.h>
#include <adore/apps/if_plotlab/plot_views.h>
#include <adore/apps/if_plotlab/laneplot_config.h>
#include <adore/env/afactory.h>
#include <adore/env/borderbased/borderset.h>
#include <adore/params/afactory.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <list>
#include <dlib/matrix.h>
#include <chrono>
#include <deque>
#include <fstream>


namespace adore
{
  namespace apps
  {
    /**
     * @brief a optimzed plotting application to plot map borders, vehicles and environment information and background image satellite footage
     * 
     */
    class PlotLanes
    {
      private:
      adore::params::APVehicle* pvehicle_;
      adore::params::APMapProvider* pmap_;
      adore::mad::AReader<adore::env::VehicleMotionState9d>* positionReader_;
      adore::mad::AFeed<adore::env::BorderBased::Border>* borderfeed_;

      std::unordered_set<std::string> plot_tags_old_;
      std::unordered_set<std::string> plot_tags_current_;
      
      DLR_TS::PlotLab::AFigureStub* figure_;
      adore::env::BorderBased::BorderSet borderSet_;
      adore::env::VehicleMotionState9d position_;
      std::string prefix_;

      double fov_width_;
      double fov_height_;

      adore::PLOT::LanePlotConfig config_;

      struct info
      {
        std::string name_;
        bool visible_;
        info(bool visible,std::string name):name_(name),visible_(visible){}
      };
      std::map<adore::env::BorderBased::BorderID,info> plotMap_;

      void unfocus(adore::env::BorderBased::BorderID id)
      {
        auto result = plotMap_.find(id);
        if(result!=plotMap_.end())
        {
          if(result->second.visible_)
          {
            figure_->erase(result->second.name_);
          }
          plotMap_.erase(id);
        }
      }

      bool isVisible(adore::env::BorderBased::BorderID id)
      {
        auto result = plotMap_.find(id);
        return result!=plotMap_.end() && result->second.visible_;
      }
      void setVisible(adore::env::BorderBased::BorderID id,bool visible)
      {
        static long long counter = 0;
        auto result = plotMap_.find(id);
        if(result!=plotMap_.end())
        {
          result->second.visible_ = visible;
        }
        else
        {
          counter ++;
          std::stringstream ss;
          ss<<prefix_<<"/localmap/border"<<counter;
          plotMap_.emplace(std::make_pair(id,info(visible,ss.str())));
        }
      }
      std::string getName(adore::env::BorderBased::BorderID id)
      {
        auto result = plotMap_.find(id);
        return result->second.name_;
      }

      public:
      PlotLanes(DLR_TS::PlotLab::AFigureStub* figure,std::string prefix,const adore::PLOT::LanePlotConfig & config)
      {
        borderfeed_ = adore::env::EnvFactoryInstance::get()->getBorderFeed();
        positionReader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
        pvehicle_= adore::params::ParamsFactoryInstance::get()->getVehicle();
        pmap_ = adore::params::ParamsFactoryInstance::get()->getMapProvider();
        figure_ = figure;      
        prefix_ = prefix;
        fov_width_ = 640.0;
        fov_height_ = 480.0;
        config_ = config;
      }

      ~PlotLanes()
       {
       }

      void run()
      {

        plot_tags_old_.insert(plot_tags_current_.begin(),plot_tags_current_.end());
        plot_tags_current_.clear();

        // adore::env::BorderBased::BorderSet newBorderSet_;
        while(borderfeed_->hasNext())
        {
          
          bool to_be_plotted = false;
          auto border = new adore::env::BorderBased::Border();
          borderfeed_->getNext(*border);
          borderSet_.insert_border(border,true);
        }

        /// plot borders
        for(auto it=borderSet_.getAllBorders();it.first!=it.second;it.first++)
        {
            adore::env::BorderBased::Border* border = it.first->second;
            adore::env::BorderBased::Border* leftNeighbor = 0;
            bool do_plot = false;
            switch(border->m_type)
            {
              case adore::env::BorderBased::BorderType::DRIVING:
                do_plot = config_.plot_drive_lane;
                break;
              case adore::env::BorderBased::BorderType::EMERGENCY:
                do_plot = config_.plot_emergency_lane;
                break;
              case adore::env::BorderBased::BorderType::OTHER:
                do_plot = config_.plot_other_lane;
                break;
            }

            if(do_plot){
              if(border->m_left!=0)
              {
                leftNeighbor = borderSet_.getBorder(*(border->m_left));
                if (leftNeighbor != 0 && (!isVisible(border->m_id) || !isVisible(leftNeighbor->m_id)) )
                {
                  setVisible(border->m_id,true);
                  std::stringstream ss;
                  ss<<"border/"<<std::hex<<border;
                  // auto name = getName(border->m_id);
                  auto name = ss.str();
                  if (border->m_type == adore::env::BorderBased::BorderType::DRIVING)
                  {
                    auto highlightRightBorder = (!borderSet_.hasRightNeighbor(border)) || (borderSet_.getRightNeighbor(border)->m_type != adore::env::BorderBased::BorderType::DRIVING);
                    auto highlightLeftBorder = (!borderSet_.hasLeftNeighbor(leftNeighbor)) || (leftNeighbor->m_type != adore::env::BorderBased::BorderType::DRIVING);
                    // adore::PLOT::plotBorder_fancy(name,border,leftNeighbor,0.0,highlightLeftBorder,highlightRightBorder,config_,figure_);
                    adore::PLOT::plotBorder(name,border,leftNeighbor,0.0,"FillColor=0.4,0.4,0.4",figure_);
                  }
                  else
                  {
                    // adore::PLOT::plotBorder_fancy(name,border,leftNeighbor,0.0,false,false,config_,figure_);
                    adore::PLOT::plotBorder(name,border,leftNeighbor,0.0,"FillColor=1,0.8,0.8",figure_);
                  }
                }
              }  
            }
        }

        if(positionReader_->hasUpdate())
        {
          positionReader_->getData(position_);
        }

        for(auto s:plot_tags_old_)
        {
          if(plot_tags_current_.find(s)==plot_tags_current_.end())
          {
            figure_->erase(s);
          }
        }
      }   
    };
  }
}