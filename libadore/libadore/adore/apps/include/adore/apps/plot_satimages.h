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
#include <adore/env/afactory.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h>

namespace adore
{
  namespace apps
  {
    /**
     * @brief a optimzed plotting application to display satellite images in the background
     * 
     */
    class PlotSatImages
    {
      private:
      adore::mad::AReader<adore::env::VehicleMotionState9d>* positionReader_;
      
      DLR_TS::PlotLab::AFigureStub* figure_;
      DLR_TS::PlotLab::AFigureStub* map_figure_;
      adore::env::VehicleMotionState9d position_;
      std::string prefix_;

      adore::PLOT::GeoTiles * geoTiles_;
      std::set<std::pair<int,int>> visibleTiles_;
      double fov_width_;
      double fov_height_;
      int urlPreset_;

      double X0_;
      double Y0_;
      double X1_;
      double Y1_;
      double res_;
      bool mapImageParamsSet_;

      struct info
      {
        std::string name_;
        bool visible_;
        info(bool visible,std::string name):name_(name),visible_(visible){}
      };

      public:
      PlotSatImages(DLR_TS::PlotLab::AFigureStub* figure, std::string prefix, const adore::PLOT::GeoTilesConfig geoTiles_config)
      {
        positionReader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
        figure_ = figure;      
        prefix_ = prefix;
        fov_width_ = 640.0;
        fov_height_ = 480.0;
        urlPreset_ = 0;
        geoTiles_ = new adore::PLOT::GeoTiles(geoTiles_config.base_url,geoTiles_config.tile_width_meters);
        mapImageParamsSet_ = false;
        map_figure_ = nullptr;
      }

      ~PlotSatImages()
       {
         geoTiles_->~GeoTiles();
       }
      void setMapImageParams(double X0,double Y0,double X1,double Y1,double res,DLR_TS::PlotLab::AFigureStub* value)
      {
        map_figure_ = value;
        X0_=X0;Y0_=Y0;X1_=X1;Y1_=Y1;
        res_=res;
        mapImageParamsSet_=true;
      }

      void run()
      {
        if(positionReader_->hasUpdate())
        {
          positionReader_->getData(position_);
          plotPosition(prefix_+"/measuredPos",position_.getX(),position_.getY());
          if(mapImageParamsSet_)
          {
            std::string url = geoTiles_->getURL(X0_,Y0_,X1_,Y1_,res_);
            std::cout<<url<<std::endl;
            map_figure_->plotTexture("satmap",url,(X0_+X1_)*0.5,(Y0_+Y1_)*0.5,-1,0.0,std::abs(Y1_-Y0_),std::abs(X1_-X0_));
            map_figure_->setFixAxis(X0_,Y0_,-10.0,X1_,Y1_,10.0);
          }
        }
      }


      /**
       * @brief plotting a vehicle
       * 
       * @param name a tag used to id the vehicle
       * @param gX x position
       * @param gY y position
       * @param psi heading
       * @param L 
       * @param c 
       * @param d 
       * @param w 
       * @param options drawing options, cf. plotlablib
       */
      // void plotPosition(const std::string& name,double gX,double gY,double psi,double L,double c,double d,double w,const std::string& options,bool follow_vehicle=true)
      void plotPosition(const std::string& name,double gX,double gY)
      {
        double xmin = gX-fov_width_*0.5;
        double xmax = gX+fov_width_*0.5;
        double ymin = gY-fov_height_*0.5;
        double ymax = gY+fov_height_*0.5;

        //remove tiles which are no longer visible
        std::vector<std::pair<int,int>> remove;
        for(auto it = visibleTiles_.begin();it!=visibleTiles_.end();it++)
        {
          if(!geoTiles_->overlapsBox(*it,xmin-10.0,ymin-10.0,xmax+10.0,ymax+10.0))
          {
            remove.push_back(*it);
            figure_->erase(geoTiles_->getPlotID(*it));
          }
        }
        for(auto it = remove.begin();it!=remove.end();it++)
        {
          visibleTiles_.erase(*it);
          // eraseCount_ ++;
        }

        //find newly visible tiles
        int i0,j0,i1,j1;
        geoTiles_->getVisibleRange(xmin,ymin,xmax,ymax,i0,j0,i1,j1);
        for(int i=i0;i<=i1;i++)
        {
          for(int j=j0;j<=j1;j++)
          {
            auto id = std::make_pair(i,j);
            if(geoTiles_->overlapsBox(id,xmin,ymin,xmax,ymax))
            {
              if( visibleTiles_.find(id)==visibleTiles_.end() )
              {
                visibleTiles_.insert(id);
                std::string url = geoTiles_->getURL(id);
                std::cout<<url<<std::endl;
                std::string hashtag = geoTiles_->getPlotID(id);
                figure_->plotTexture(hashtag,url,geoTiles_->getCenterX(id),geoTiles_->getCenterY(id),-0.1,0.0,geoTiles_->getWidthM(),geoTiles_->getWidthM());
                // plotCount_++;
              }
            }
          }
        }
      }
    };
  }
}