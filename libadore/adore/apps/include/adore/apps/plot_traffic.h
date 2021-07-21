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
#include <adore/env/afactory.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h>
#include <unordered_set>


namespace adore
{
  namespace apps
  {
    /**
     * @brief a optimzed plotting application to plot other traffic
     * 
     */
    class PlotTraffic
    {
      private:
      adore::env::AFactory::TParticipantSetReader* trafficReader_;
      bool debug_plot_ids_;

      std::unordered_set<std::string> plot_tags_old_;
      std::unordered_set<std::string> plot_tags_current_;
      
      DLR_TS::PlotLab::AFigureStub* figure_;
      // adore::env::VehicleMotionState9d position_;

      std::string prefix_;

      public:
      PlotTraffic(DLR_TS::PlotLab::AFigureStub* figure, std::string prefix, bool debug_plot_ids) :
        figure_(figure), prefix_(prefix), debug_plot_ids_(debug_plot_ids)
      {
        trafficReader_ = adore::env::EnvFactoryInstance::get()->getTrafficParticipantSetReader();
      }

      ~PlotTraffic()
       {
       }

      void run()
      {
        plot_tags_old_.insert(plot_tags_current_.begin(),plot_tags_current_.end());
        plot_tags_current_.clear();
        // if(positionReader_->hasUpdate())
        // {
        //   positionReader_->getData(position_);
        //   t_ = position_.getTime();
        // }

        if(trafficReader_->hasUpdate())
        {
          // TODO is the feature to restrict plotting of traffic entities to a radius really useful?
          // double R = pmap_->getVisibiltyRadius() + 100.0;//delete radius
          adore::env::traffic::TParticipantSet tpset;
          trafficReader_->getData(tpset);
          for(auto& tp:tpset)
          {
            auto pos = tp.getCenter();
            auto psi = tp.getYaw();
            const double abcd = tp.getLength();
            const double c = 0.2 * abcd;
            const double d = 0.2 * abcd;
            const double L = abcd-c-d;
            const double w = tp.getWidth(); 
            pos(0) = pos(0) + cos(psi)*(-0.5*abcd+d);
            pos(1) = pos(1) + sin(psi)*(-0.5*abcd+d);
            // do not plot traffic outside of range
            // if(abs(pos(0)-position_.getX())>R || abs(pos(1)-position_.getY())>R)
            // {
            //   continue;
            // }
              
            std::stringstream ss;
            ss<<prefix_<<"/traffic/"<<tp.getTrackingID();
            plotPosition(ss.str(),pos(0),pos(1),psi,L,c,d,w*0.5,"LineColor=0,0,0",tp.getClassification(),tp.getTrackingID());
          }
        }

        for(auto s:plot_tags_old_)
        {
          if(plot_tags_current_.find(s)==plot_tags_current_.end())
          {
            figure_->erase(s);
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
      void plotPosition(const std::string& name,double gX,double gY,double psi,double L,double c,double d,double w,const std::string& options,adore::env::traffic::Participant::EClassification participant_type, int id)
      {
        auto length = L+c+d;
        auto rho = length*0.5-d;
        auto width = w*2;
        auto z = 1.1;        
        auto m_targetZoom = 20.0;

        std::string s = name+"/texture";
        std::stringstream image;
        if(participant_type==adore::env::traffic::Participant::CAR)image<<"../images/car"<<std::setw(3) <<std::setfill('0')<<((id%18) + 1)<<".png";
        else if(participant_type==adore::env::traffic::Participant::PEDESTRIAN)image<<"../images/person"<<std::setw(2) <<std::setfill('0')<<((id%10) + 1)<<".png";
        else if(participant_type==adore::env::traffic::Participant::PEDESTRIAN_GROUP)image<<"../images/person_group.png";
        else if(participant_type==adore::env::traffic::Participant::BUS)image<<"../images/bus.png";
        else if(participant_type==adore::env::traffic::Participant::HEAVY_TRUCK)image<<"../images/bus.png";
        else if(participant_type==adore::env::traffic::Participant::TRUCK)image<<"../images/bus.png";
        if(image.str().size()>0)
        {
          figure_->plotTexture(s,image.str(),gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
          plot_tags_current_.insert(s);
        }
        double cpsi = std::cos(psi);
        double spsi = std::sin(psi);
        double X[5];
        double Y[6];
        X[0] = gX+cpsi*(-d)-spsi*(-w);
        X[1] = gX+cpsi*(L+c)-spsi*(-w);
        X[2] = gX+cpsi*(L+c)-spsi*(+w);
        X[3] = gX+cpsi*(-d)-spsi*(+w);
        X[4] = gX+cpsi*(-d)-spsi*(-w);
        Y[0] = gY+spsi*(-d)+cpsi*(-w);
        Y[1] = gY+spsi*(L+c)+cpsi*(-w);
        Y[2] = gY+spsi*(L+c)+cpsi*(+w);
        Y[3] = gY+spsi*(-d)+cpsi*(+w);
        Y[4] = gY+spsi*(-d)+cpsi*(-w);
        std::string s2 = name+"/outline";
        figure_->plot(s2,X,Y,z+0.5,5,"LineColor=1,0,0");
        plot_tags_current_.insert(s2);
        if (debug_plot_ids_)
        {
          figure_->plotText(name,gX+w+0.5,gY,name);
        }
      }   
    };
  }
}