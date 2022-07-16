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
#include <adore/params/afactory.h>
#include <adore/apps/if_plotlab/plot_shape.h>
#include <adore/env/afactory.h>
#include <adore/fun/afactory.h>
#include <adore/sim/afactory.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h>
#include <string>
#include <vector>



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
      adore::mad::AReader<adore::fun::VehicleMotionState9d>* positionReader_;
      adore::mad::AReader<adore::fun::VehicleMotionState9d>* simPositionReader_;
      adore::mad::AReader<adore::fun::VehicleExtendedState>* xxReader_;
      adore::env::AFactory::TNavigationGoalReader* goalReader_;
      adore::fun::VehicleMotionState9d position_;
      adore::fun::VehicleMotionState9d simPosition_;
      adore::fun::VehicleExtendedState extendedState_;
      
      
      DLR_TS::PlotLab::AFigureStub* figure_;
      DLR_TS::PlotLab::AFigureStub* map_figure_;
      
      std::string prefix_;
      std::string vehicle_picture_;
      std::string position_style_;
      std::string target_picture_;

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
      PlotEgo(DLR_TS::PlotLab::AFigureStub* figure,std::string prefix, int followMode)
      {
        positionReader_ = adore::fun::FunFactoryInstance::get()->getVehicleMotionStateReader();
        simPositionReader_ = adore::sim::SimFactoryInstance::get()->getVehicleMotionStateReader();//this is the true vehicle state, displayed in simulation
        pvehicle_= adore::params::ParamsFactoryInstance::get()->getVehicle();
        xxReader_ = adore::fun::FunFactoryInstance::get()->getVehicleExtendedStateReader();
        goalReader_ = adore::env::EnvFactoryInstance::get()->getNavigationGoalReader();
        figure_ = figure;     
        map_figure_ = nullptr; 
        prefix_ = prefix;
        fov_width_ = 640.0;
        fov_height_ = 480.0;
        followMode_ = followMode != 0;
        vehicle_picture_ = "../images/ego.png";
        target_picture_ = "../images/target.png";
        position_style_ = "LineColor=0,0,0;LineWidth=2.0";
      }
      void setPicture(std::string value)
      {
        vehicle_picture_ = value;
      }
      void setMapFigure(DLR_TS::PlotLab::AFigureStub* value)
      {
        map_figure_ = value;
      }

      ~PlotEgo()
       {
       }

      void run()
      {
        bool sim_mode = simPositionReader_->hasData();
        if(positionReader_->hasUpdate())
        {
          positionReader_->getData(position_);
          if(sim_mode)simPositionReader_->getData(simPosition_);

          //automation state
          if(xxReader_->hasData())
          {
            xxReader_->getData(extendedState_);
            if(extendedState_.getAutomaticControlAccelerationActive()
            && extendedState_.getAutomaticControlAccelerationOn()
            && extendedState_.getAutomaticControlSteeringOn())
            {
              position_style_ = "LineColor=0,0,1;LineWidth=2.0";
            }
            else
            {
              position_style_ = "LineColor=0,0.8,0;LineWidth=2.0";
            }
          }
          
          //plot the vehicle
          vehicle_picture_ = "../images/"+pvehicle_->get_vehicle_id()+".png";
          const double L = pvehicle_->get_a()+pvehicle_->get_b();
          const double c = pvehicle_->get_c();
          const double d = pvehicle_->get_d();
          // const double w = std::max(pvehicle_->get_wf(),pvehicle_->get_wr())*0.5;
          const double w = pvehicle_->get_bodyWidth()*0.5;
          const double delta = position_.getDelta();
          const double wheel_radius = 0.40;
          const double wheel_width = 0.20;
          if(sim_mode)
          {
            plotPosition(prefix_+"/truePos",vehicle_picture_,simPosition_.getX(),simPosition_.getY(),simPosition_.getPSI(),delta,simPosition_.getvx(),simPosition_.getvy(),simPosition_.getOmega(),L,c,d,w,wheel_radius,wheel_width,"LineColor=1,0,0",figure_);
            plotPosition(prefix_+"/measuredPos","",position_.getX(),position_.getY(),position_.getPSI(),delta,position_.getvx(),position_.getvy(),position_.getOmega(),L,c,d,w,wheel_radius,wheel_width,position_style_,figure_);
          }
          else
          {
            plotPosition(prefix_+"/measuredPos",vehicle_picture_,position_.getX(),position_.getY(),position_.getPSI(),delta,position_.getvx(),position_.getvy(),position_.getOmega(),L,c,d,w,wheel_radius,wheel_width,position_style_,figure_);
          }

          if(map_figure_!=nullptr)
          {
            plotPosition(prefix_+"/measuredPos",vehicle_picture_,position_.getX(),position_.getY(),position_.getPSI(),delta,position_.getvx(),position_.getvy(),position_.getOmega(),L,c,d,w,wheel_radius,wheel_width,position_style_,map_figure_);
            plotCircle(prefix_+"/ego_c1",position_.getX(),position_.getY(),20,"LineWidth=3.0;LineColor=0,0,1",map_figure_);
            plotCircle(prefix_+"/ego_c2",position_.getX(),position_.getY(),100,"LineWidth=3.0;LineColor=0,0,1",map_figure_);
            plotCircle(prefix_+"/ego_c3",position_.getX(),position_.getY(),300,"LineWidth=3.0;LineColor=0,0,1",map_figure_);
          }

          if ( followMode_ != 0 )
          {
            double rho = position_.getvx() * 1.5;
            double gX = position_.getX() + rho * (std::cos)(position_.getPSI());
            double gY = position_.getY() + rho * (std::sin)(position_.getPSI());;
            auto zoom = 50.0 / std::max(5.0,std::abs(position_.getvx())*2.0);
            figure_->setViewPortOffsets(gX,gY,360.0*(position_.getPSI() / (2.0 *3.14159))-90.0,zoom);
          }

        }
        {//plot the goal marker
          double scale = 1.0/725.0 * 10.0;
          double length=378;
          double width=1107;
          if(goalReader_->hasData())
          {//plot goal marker in local map with vehicle orientation
            adore::fun::NavigationGoal goal;
            goalReader_->getData(goal);
            figure_->plotTexture(prefix_+"/target",target_picture_,goal.target_.x_,goal.target_.y_,2.0,position_.getPSI(),width*scale,length*scale);
            if(map_figure_!=nullptr)
            {
              map_figure_->plotTexture(prefix_+"/target",target_picture_,goal.target_.x_,goal.target_.y_,2.0,0.0,width*scale,length*scale);
              plotCircle(prefix_+"/target_c1",goal.target_.x_,goal.target_.y_,20,"LineWidth=3.0;LineColor=1,0,0",map_figure_);
              plotCircle(prefix_+"/target_c2",goal.target_.x_,goal.target_.y_,100,"LineWidth=3.0;LineColor=1,0,0",map_figure_);
              plotCircle(prefix_+"/target_c3",goal.target_.x_,goal.target_.y_,300,"LineWidth=3.0;LineColor=1,0,0",map_figure_);
            }
          }
        }
      }

      void plotCircle(const std::string& name,double gX,double gY,double R,const std::string& options,DLR_TS::PlotLab::AFigureStub* figure)
      {
        static const int N=50;
        double X[N];
        double Y[N];
        for(int i=0;i<N;i++)
        {
          X[i] = gX + std::cos(M_PI*2.0*(double)i/(double)(N-1))*R;
          Y[i] = gY + std::sin(M_PI*2.0*(double)i/(double)(N-1))*R;
        }
        figure->plot(name,X,Y,2.0,N,options);
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
      void plotPosition(const std::string& name,const std::string& vehicle_picture,double gX,double gY,double psi,double delta,double vx,double vy,double omega,double L,double c,double d,double w,double wheel_radius,double wheel_width,const std::string& options,DLR_TS::PlotLab::AFigureStub* figure)
      {
        double X[200];
        double Y[200];
        std::vector<std::pair<double,double>> points;
        attach_vehicle_points2(gX,gY,psi,delta,vx,vy+(-d+(L+c+d)*0.5)*omega,L,c,d,w,wheel_radius,wheel_width,points);
        for(int i=0;i<points.size();i++){X[i]=points[i].first;Y[i]=points[i].second;}
        figure->plot(name,X,Y,1.5,points.size(),options);


        double xmin = gX-fov_width_*0.5;
        double xmax = gX+fov_width_*0.5;
        double ymin = gY-fov_height_*0.5;
        double ymax = gY+fov_height_*0.5;

        /** TODO fix parameters */
        auto length = L+c+d;
        auto rho = length*0.5-d;
        auto width = w*2;
        auto z = 1.1;        

        if(vehicle_picture!="")
        {
          figure->plotTexture(name+"vehiclealphablend",vehicle_picture,gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
        }
      }

      void attach_wheel_points(double x,double y, double delta, double wheel_radius,double wheel_width,std::vector<std::pair<double,double>>& points)
      {
        const double c = (std::cos)(delta);
        const double s = (std::sin)(delta);
        const double w = wheel_width * 0.5;
        const double r = wheel_radius;
        
        points.push_back(std::make_pair(x,y));
        points.push_back(std::make_pair(x + c * 0.0 - s * w,
                                        y + s * 0.0 + c * w));
        points.push_back(std::make_pair(x + c * (-r) - s * w,
                                        y + s * (-r) + c * w));
        points.push_back(std::make_pair(x + c * (-r) - s * (-w),
                                        y + s * (-r) + c * (-w)));
        points.push_back(std::make_pair(x + c * (+r) - s * (-w),
                                        y + s * (+r) + c * (-w)));
        points.push_back(std::make_pair(x + c * (+r) - s * (+w),
                                        y + s * (+r) + c * (+w)));
        points.push_back(std::make_pair(x + c * 0.0 - s * (+w),
                                        y + s * 0.0 + c * (+w)));
        points.push_back(std::make_pair(x + c * 0.0 - s * (-w),
                                        y + s * 0.0 + c * (-w)));
        points.push_back(std::make_pair(x,y));
      }

      void attach_vehicle_points2(double x,double y,double psi, double delta, double vx, double vy, double L,double c,double d,double w,double wheel_radius,double wheel_width,std::vector<std::pair<double,double>>& points)
      {
        const double cos_psi = (std::cos)(psi);
        const double sin_psi = (std::sin)(psi);
        const double ww = wheel_width*0.5;
        const double wing_mirror = 0.17;
        const double vscale = 1.0;

        points.push_back(std::make_pair(x,y));//COR
        attach_wheel_points(x + cos_psi * (0.0) - sin_psi * (-w+ww+wing_mirror),
                            y + sin_psi * (0.0) + cos_psi * (-w+ww+wing_mirror),
                            psi,wheel_radius,wheel_width,points);//RR
        attach_wheel_points(x + cos_psi * (0.0) - sin_psi * (+w-ww-wing_mirror),
                            y + sin_psi * (0.0) + cos_psi * (+w-ww-wing_mirror),
                            psi,wheel_radius,wheel_width,points);//RL
        points.push_back(std::make_pair(x,y));//COR
        points.push_back(std::make_pair(x + cos_psi * (L) - sin_psi * (0.0),
                                        y + sin_psi * (L) + cos_psi * (0.0)));//COF
        attach_wheel_points(x + cos_psi * (L) - sin_psi * (+w-ww-wing_mirror),
                            y + sin_psi * (L) + cos_psi * (+w-ww-wing_mirror),
                            psi+delta,wheel_radius,wheel_width,points);//FL
        attach_wheel_points(x + cos_psi * (L) - sin_psi * (-w+ww+wing_mirror),
                            y + sin_psi * (L) + cos_psi * (-w+ww+wing_mirror),
                            psi+delta,wheel_radius,wheel_width,points);//FR
        points.push_back(std::make_pair(x + cos_psi * (L) - sin_psi * (0.0),
                                        y + sin_psi * (L) + cos_psi * (0.0)));//COF
        points.push_back(std::make_pair(x + cos_psi * (L+c) - sin_psi * (0.0),
                                        y + sin_psi * (L+c) + cos_psi * (0.0)));//COFB
        points.push_back(std::make_pair(x + cos_psi * (L+c*0.5) - sin_psi * (w*0.5),
                                        y + sin_psi * (L+c*0.5) + cos_psi * (w*0.5)));
        points.push_back(std::make_pair(x + cos_psi * (L+c) - sin_psi * (0.0),
                                        y + sin_psi * (L+c) + cos_psi * (0.0)));//COFB
        points.push_back(std::make_pair(x + cos_psi * (L+c*0.5) - sin_psi * (-w*0.5),
                                        y + sin_psi * (L+c*0.5) + cos_psi * (-w*0.5)));
        points.push_back(std::make_pair(x + cos_psi * (L+c) - sin_psi * (0.0),
                                        y + sin_psi * (L+c) + cos_psi * (0.0)));//COFB
        points.push_back(std::make_pair(x + cos_psi * (L+c) - sin_psi * (w),
                                        y + sin_psi * (L+c) + cos_psi * (w)));
        points.push_back(std::make_pair(x + cos_psi * (-d) - sin_psi * (w),
                                        y + sin_psi * (-d) + cos_psi * (w)));
        points.push_back(std::make_pair(x + cos_psi * (-d) - sin_psi * (-w),
                                        y + sin_psi * (-d) + cos_psi * (-w)));
        points.push_back(std::make_pair(x + cos_psi * (L+c) - sin_psi * (-w),
                                        y + sin_psi * (L+c) + cos_psi * (-w)));
        points.push_back(std::make_pair(x + cos_psi * (L+c) - sin_psi * (0.0),
                                        y + sin_psi * (L+c) + cos_psi * (0.0)));//COFB

        points.push_back(std::make_pair(x + cos_psi * (-d+(L+c+d)*0.5) - sin_psi * (0.0),
                                        y + sin_psi * (-d+(L+c+d)*0.5) + cos_psi * (0.0)));//C
        points.push_back(std::make_pair(x + cos_psi * (-d+(L+c+d)*0.5 + vx*vscale) - sin_psi * (vy*vscale),
                                        y + sin_psi * (-d+(L+c+d)*0.5 + vx*vscale) + cos_psi * (vy*vscale)));//v-arrow-tip
        const double v = std::sqrt(vx*vx+vy*vy); 
        const double cv = v>0.1?vx/v:0.0;
        const double sv = v>0.1?vy/v:0.0;
        const double ax = 1.0;
        const double ay = 1.0;

        points.push_back(std::make_pair(x + cos_psi * (-d+(L+c+d)*0.5 + vx*vscale - cv *ax - sv*ay ) - sin_psi * (vy*vscale - sv *ax + cv*ay),
                                        y + sin_psi * (-d+(L+c+d)*0.5 + vx*vscale - cv *ax - sv*ay ) + cos_psi * (vy*vscale - sv *ax + cv*ay)));//v-arrow-left
        points.push_back(std::make_pair(x + cos_psi * (-d+(L+c+d)*0.5 + vx*vscale) - sin_psi * (vy*vscale),
                                        y + sin_psi * (-d+(L+c+d)*0.5 + vx*vscale) + cos_psi * (vy*vscale)));//v-arrow-tip
        points.push_back(std::make_pair(x + cos_psi * (-d+(L+c+d)*0.5 + vx*vscale - cv *ax + sv*ay ) - sin_psi * (vy*vscale - sv *ax - cv*ay),
                                        y + sin_psi * (-d+(L+c+d)*0.5 + vx*vscale - cv *ax + sv*ay ) + cos_psi * (vy*vscale - sv *ax - cv*ay)));//v-arrow-right


      }
    };
  }
}