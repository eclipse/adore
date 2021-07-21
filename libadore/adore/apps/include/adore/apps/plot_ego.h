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
#include <adore/params/ap_vehicle.h>
#include <adore/apps/if_plotlab/plot_shape.h>
#include <adore/env/afactory.h>
#include <adore/fun/afactory.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h>
#include <string>


namespace adore
{
  namespace apps
  {
    /**
     * @brief a optimzed plotting application to plot map borders, vehicles and environment information and background image satellite footage
     * 
     */
    class PlotEgo
    {
      private:
      adore::params::APVehicle* pvehicle_;
      adore::mad::AReader<adore::env::VehicleMotionState9d>* positionReader_;
      adore::env::VehicleMotionState9d position_;
      
      
      DLR_TS::PlotLab::AFigureStub* figure_;
      std::string prefix_;

      double fov_width_;
      double fov_height_;

      bool followMode_;

      struct info
      {
        std::string name_;
        bool visible_;
        info(bool visible,std::string name):name_(name),visible_(visible){}
      };
      public:
      PlotEgo(DLR_TS::PlotLab::AFigureStub* figure,adore::params::APVehicle* pvehicle,std::string prefix, int followMode)
      {
        positionReader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
        pvehicle_= pvehicle;
        figure_ = figure;      
        prefix_ = prefix;
        fov_width_ = 640.0;
        fov_height_ = 480.0;
        followMode_ = followMode != 0;
      }

      ~PlotEgo()
       {
       }

      void run()
      {

        if(positionReader_->hasUpdate())
        {
          positionReader_->getData(position_);
          
          //plot the vehicle
          const double L = pvehicle_->get_a()+pvehicle_->get_b();
          const double c = pvehicle_->get_c();
          const double d = pvehicle_->get_d();
          const double w = std::max(pvehicle_->get_wf(),pvehicle_->get_wr())*0.5;
          plotPosition(prefix_+"/measuredPos",position_.getX(),position_.getY(),position_.getPSI(),L,c,d,w,"LineColor=0,0,1");
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
      void plotPosition(const std::string& name,double gX,double gY,double psi,double L,double c,double d,double w,const std::string& options,bool follow_vehicle=true)
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
        
        double xmin = gX-fov_width_*0.5;
        double xmax = gX+fov_width_*0.5;
        double ymin = gY-fov_height_*0.5;
        double ymax = gY+fov_height_*0.5;


                
        /** TODO fix parameters */
        auto length = L+c+d;
        auto rho = length*0.5-d;
        auto width = w*2;
        auto z = 1.1;        
        auto m_targetZoom = 20.0;

        if(follow_vehicle)
        {
          // figure_->plotTexture(name+"vehiclealphablend","fascare_shadow.png",gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
          figure_->plotTexture(name+"vehiclealphablend","../images/fascare_hq.png",gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
        }
        else
        {
          // figure_->plotTexture(name+"vehiclealphablend","fascare_shadow.png",gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
          std::string s = name+"/texture";
          figure_->plotTexture(s,"../images/sumocar.png",gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
        }
        if (follow_vehicle && followMode_ != 0)
        {
          figure_->setViewPortOffsets(gX,gY,360.0*(psi / (2.0 *3.14159))-90.0,m_targetZoom);
        }
      }
    
    };
  }
}