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

        if(trafficReader_->hasUpdate())
        {

          plot_tags_old_.insert(plot_tags_current_.begin(),plot_tags_current_.end());
          plot_tags_current_.clear();

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
              
            std::stringstream ss;
            ss<<prefix_<<"/traffic/"<<tp.getTrackingID();
            plotPosition(ss.str(),pos(0),pos(1),psi,L,c,d,w*0.5,tp.getVx(),tp.getVy(),"LineColor=0,0,0",tp.getClassification(),tp.getTrackingID(),tp.getStationID());
          }

          for(auto s:plot_tags_old_)
          {
            if(plot_tags_current_.find(s)==plot_tags_current_.end())
            {
              figure_->erase(s);
            }
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
      void plotPosition(const std::string& name,double gX,double gY,double psi,double L,double c,double d,double w,double vx,double vy,const std::string& options,adore::env::traffic::Participant::EClassification participant_type, int id, int v2xid = 0)
      {
        auto length = L+c+d;
        auto rho = length*0.5-d;
        double a = L*0.5;
        double b = L*0.5;
        auto width = w*2;
        auto z = 1.1;        
        auto m_targetZoom = 20.0;

        std::string s = name+"/texture";
        std::stringstream image;
        if(v2xid==111)image<<"../images/viewcar2.png";
        else if(v2xid==222)image<<"../images/FASCarE.png";
        else if(participant_type==adore::env::traffic::Participant::CAR)image<<"../images/car"<<std::setw(3) <<std::setfill('0')<<((id%18) + 1)<<".png";
        else if(participant_type==adore::env::traffic::Participant::PEDESTRIAN)image<<"../images/person"<<std::setw(2) <<std::setfill('0')<<((id%10) + 1)<<".png";
        else if(participant_type==adore::env::traffic::Participant::PEDESTRIAN_GROUP)image<<"../images/person_group.png";
        else if(participant_type==adore::env::traffic::Participant::BUS)image<<"../images/bus.png";
        else if(participant_type==adore::env::traffic::Participant::HEAVY_TRUCK)image<<"../images/bus.png";
        else if(participant_type==adore::env::traffic::Participant::TRUCK)image<<"../images/truck.png";
        if(image.str().size()>0)
        {
          figure_->plotTexture(s,image.str(),gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
          plot_tags_current_.insert(s);
        }
        double cpsi = std::cos(psi);
        double spsi = std::sin(psi);
        double X[14];
        double Y[14];
        double vscale = 1.0;
        double v = std::sqrt(vx*vx+vy*vy);
        double cv = v>0.1?vx/v:0.0;
        double sv = v>0.1?vy/v:0.0;
        double ca = 0.7071;
        double sa = 0.7071;
        double ascale = 0.2 * vscale;
        double ax = -v * ca * ascale - v * sa * ascale;
        double ay = -v * sa * ascale + v * ca * ascale;
        ax= 1.0;
        ay = 1.0;
        X[0] = gX + std::cos(psi) * (b) - std::sin(psi) * (-w);
        Y[0] = gY + std::sin(psi) * (b) + std::cos(psi) * (-w);
        X[1] = gX + std::cos(psi) * (a+b+c) - std::sin(psi) * (0.0);
        Y[1] = gY + std::sin(psi) * (a+b+c) + std::cos(psi) * (0.0);
        X[2] = gX + std::cos(psi) * (b) - std::sin(psi) * (w);
        Y[2] = gY + std::sin(psi) * (b) + std::cos(psi) * (w);
        X[3] = gX + std::cos(psi) * (b) - std::sin(psi) * (-w);
        Y[3] = gY + std::sin(psi) * (b) + std::cos(psi) * (-w);
        X[4] = gX + std::cos(psi) * (a+b+c) - std::sin(psi) * (-w);
        Y[4] = gY + std::sin(psi) * (a+b+c) + std::cos(psi) * (-w);
        X[5] = gX + std::cos(psi) * (a+b+c) - std::sin(psi) * (w);
        Y[5] = gY + std::sin(psi) * (a+b+c) + std::cos(psi) * (w);
        X[6] = gX + std::cos(psi) * (-d) - std::sin(psi) * (w);
        Y[6] = gY + std::sin(psi) * (-d) + std::cos(psi) * (w);
        X[7] = gX + std::cos(psi) * (-d) - std::sin(psi) * (-w);
        Y[7] = gY + std::sin(psi) * (-d) + std::cos(psi) * (-w);
        X[8] = gX + std::cos(psi) * (b) - std::sin(psi) * (-w);
        Y[8] = gY + std::sin(psi) * (b) + std::cos(psi) * (-w);
        X[9] = gX + std::cos(psi) * (b) - std::sin(psi) * (0.0);
        Y[9] = gY + std::sin(psi) * (b) + std::cos(psi) * (0.0);
        X[10] = gX + std::cos(psi) * (b + vx*vscale) - std::sin(psi) * (vy*vscale);
        Y[10] = gY + std::sin(psi) * (b + vx*vscale) + std::cos(psi) * (vy*vscale);
        X[11] = gX + std::cos(psi) * (b + vx*vscale - cv * ax - sv * ay) - std::sin(psi) * (vy*vscale -sv * ax + cv * ay);
        Y[11] = gY + std::sin(psi) * (b + vx*vscale - cv * ax - sv * ay) + std::cos(psi) * (vy*vscale -sv * ax + cv * ay);
        X[12] = gX + std::cos(psi) * (b + vx*vscale) - std::sin(psi) * (vy*vscale);
        Y[12] = gY + std::sin(psi) * (b + vx*vscale) + std::cos(psi) * (vy*vscale);
        X[13] = gX + std::cos(psi) * (b + vx*vscale - cv * ax + sv * ay) - std::sin(psi) * (vy*vscale -sv * ax - cv * ay);
        Y[13] = gY + std::sin(psi) * (b + vx*vscale - cv * ax + sv * ay) + std::cos(psi) * (vy*vscale -sv * ax - cv * ay);
        std::string s2 = name+"/outline";
        figure_->plot(s2,X,Y,z+0.5,14,"LineColor=1,0,0");
        plot_tags_current_.insert(s2);
        if (debug_plot_ids_)
        {
          figure_->plotText(name,gX+w+0.5,gY,name);
        }
      }   
    };
  }
}