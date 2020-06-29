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
 ********************************************************************************/

#pragma once
#include <adore/params/ap_vehicle.h>
#include <adore/params/ap_map_provider.h>
#include <adore/env/afactory.h>
#include <adore/env/borderbased/borderset.h>
#include <adore/fun/afactory.h>
#include <plotlablib/afigurestub.h>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include "if_plotlab/plot_border.h"

namespace adore
{
  namespace apps
  {
    /**
     * @brief a basic plotting application to plot map borders, vehicles and environment information
     * 
     */
    class BorderBird
    {
      private:
      adore::params::APVehicle* pvehicle_;
      adore::params::APMapProvider* pmap_;
      adore::mad::AReader<adore::env::VehicleMotionState9d>* positionReader_;
      adore::mad::AFeed<adore::env::BorderBased::Border>* borderfeed_;
      adore::mad::AReader<adore::fun::SetPointRequest>* sprReader_;
      adore::fun::SetPointRequest spr_;
      adore::env::AFactory::TParticipantSetReader* trafficReader_;

      std::unordered_set<int> visibleTraffic_;
      
      DLR_TS::PlotLab::AFigureStub* figure_;
      unsigned long long counter_;
      adore::env::BorderBased::BorderSet borderSet_;
      adore::env::VehicleMotionState9d position_;
      std::string prefix_;
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
      BorderBird(adore::env::AFactory* envfactory,adore::fun::AFactory* funfactory,DLR_TS::PlotLab::AFigureStub* figure,adore::params::APVehicle* pvehicle, adore::params::APMapProvider* pmap,std::string prefix)
      {
        counter_=0;
        borderfeed_ = envfactory->getBorderFeed();
        positionReader_ = envfactory->getVehicleMotionStateReader();
        sprReader_ = funfactory->getSetPointRequestReader();
        trafficReader_ = envfactory->getTrafficParticipantSetReader();
        pvehicle_= pvehicle;
        pmap_ = pmap;
        figure_ = figure;      
        prefix_ = prefix;
      }
      
      void run()
      {
        while(borderfeed_->hasNext())
        {
          /// plot borders
          bool to_be_plotted = false;
          auto border = new adore::env::BorderBased::Border();
          borderfeed_->getNext(*border);
          borderSet_.insert_border(border,true);

          auto left = borderSet_.getLeftNeighbor(border);
          auto right = borderSet_.getRightNeighbor(border);
          if(left!=0 /*&& !isVisible(border->m_id)*/)
          {
            setVisible(border->m_id,true);
            auto name = getName(border->m_id);
            adore::PLOT::plotBorder(name,border,left,0.0,"FillColor=0.8,0.8,0.8",figure_);
          }
          if(right!=0 /*&& !isVisible(right->m_id)*/)
          {
            setVisible(right->m_id,true);
            auto name = getName(right->m_id);
            adore::PLOT::plotBorder(name,right,border,0.0,"FillColor=0.8,0.8,0.8",figure_);
          }
        }
        if(positionReader_->hasUpdate())
        {
          positionReader_->getData(position_);

          //plot the vehicle
          const double L = pvehicle_->get_a()+pvehicle_->get_b();
          const double c = pvehicle_->get_c();
          const double d = pvehicle_->get_d();
          const double w = std::max(pvehicle_->get_wf(),pvehicle_->get_wr())*0.5;
          plotPosition(prefix_+"/measuredPos",position_.getX(),position_.getY(),position_.getPSI(),L,c,d,w,"LineColor=0,0,1");

          //unplot all borders, which are no longer visible
          double R = pmap_->getVisibiltyRadius() + 100.0;//delete radius
          double x0 = position_.getX() - R;
          double x1 = position_.getX() + R;
          double y0 = position_.getY() - R;
          double y1 = position_.getY() + R;
          for( auto it = borderSet_.getBordersOutsideRegion(x0,x1,y0,y1);it.current()!=it.end();it.current()++)
          {
            unfocus(it.current()->second->m_id);
          }
          borderSet_.removeBorders(borderSet_.getBordersOutsideRegion(x0,x1,y0,y1));
        }
        if(sprReader_->hasUpdate())
        {
          sprReader_->getData(spr_);
          static const int N = 100;
          double X[N];
          double Y[N];
          int k = (std::ceil)((double)spr_.setPoints.size()/(double)N);
          int count=0;
          for(int i=0;i<N;i++)
          {
            int j=i*k;
            if(j<spr_.setPoints.size())
            {
              count = i;
              X[i] = spr_.setPoints[j].x0ref.getX();
              Y[i] = spr_.setPoints[j].x0ref.getY();
            }
          }
          figure_->plot(prefix_+"spr",X,Y,1.0,count,"LineWidth=3;LineColor=1,0,0");
        }
        if(trafficReader_->hasUpdate())
        {
          adore::env::traffic::TParticipantSet tpset;
          trafficReader_->getData(tpset);
          std::unordered_set<int> newdata;
          for(auto& tp:tpset)
          {
            newdata.emplace(tp.getTrackingID());
            const double abcd = tp.getLength();
            const double c = 0.2 * abcd;
            const double d = 0.2 * abcd;
            const double L = abcd-c-d;
            const double w = tp.getWidth();
            auto pos = tp.getCenter();
            auto psi = tp.getYaw();
            pos(0) = pos(0) + cos(psi)*(-0.5*abcd+d);
            pos(1) = pos(1) + sin(psi)*(-0.5*abcd+d);
            std::stringstream ss;
            ss<<prefix_<<"/traffic/"<<tp.getTrackingID();
            // std::cout<<"i: "<<i<<" traffic id="<<tp.getTrackingID()<<", x="<<pos(0)<<", y="<<pos(1)<<"\n";
            plotPosition(ss.str(),pos(0),pos(1),psi,L,c,d,w*0.5,"LineColor=0,0,0");
          }
          // std::cout<<"---------------------------------\n";
          for(auto i:visibleTraffic_)
          {
            if(newdata.find(i)==newdata.end())
            {
              std::stringstream ss;
              ss<<prefix_<<"/traffic/"<<i;
              figure_->erase(ss.str());
            }
          }
          visibleTraffic_ = newdata;
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
      void plotPosition(const std::string& name,double gX,double gY,double psi,double L,double c,double d,double w,const std::string& options)
      {
        double X[12];
        double Y[12];
        X[0] = 0.0; X[1] = L; X[2] = L; X[3] = L + c; X[4] = L; X[5] = L + c; X[6] = L + c; X[7] = 0.0; X[8] = 0.0; X[9] = -d; X[10] = -d; X[11] = 0.0;
        Y[0] = -w; Y[1] = -w; Y[2] = +w; Y[3] = 0.0; Y[4] = -w; Y[5] = -w; Y[6] = +w; Y[7] = +w; Y[8] = -w; Y[9] = -w; Y[10] = +w; Y[11] = +w;

        double cpsi = std::cos(psi);
        double spsi = std::sin(psi);
        for(int i=0;i<12;i++)
        {
          double x = gX + cpsi * X[i] - spsi * Y[i];
          double y = gY + spsi * X[i] + cpsi * Y[i];
          X[i] = x;
          Y[i] = y;
        }

        figure_->plot(name,X,Y,0.25,12,options);
      }
    };
  }
}