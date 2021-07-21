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
// #include <adore/params/ap_map_provider.h>
// #include <adore/apps/if_plotlab/geoTiles.h>
// #include <adore/apps/if_plotlab/geoTiles_config.h>
#include <adore/apps/if_plotlab/plot_prediction.h>
// #include <adore/apps/if_plotlab/fancy_config.h>
#include <adore/apps/if_plotlab/prediction_config.h>
#include <adore/env/afactory.h>
// #include <adore/env/borderbased/borderset.h>
// #include <adore/fun/afactory.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h>
// #include <unordered_map>
#include <unordered_set>
#include <string>
// #include <list>
// #include <dlib/matrix.h>
// #include <chrono>
// #include <deque>
// #include <fstream>


namespace adore
{
  namespace apps
  {
    /**
     * @brief a plot module for handling prediction plots
     * 
     */
    class PlotPredictions
    {
      private:
      // adore::params::APVehicle* pvehicle_;
      // adore::params::APMapProvider* pmap_;
      adore::mad::AReader<adore::env::VehicleMotionState9d>* positionReader_;
      // adore::mad::AFeed<adore::env::BorderBased::Border>* borderfeed_;
      // adore::mad::AReader<adore::fun::SetPointRequest>* sprReader_;
      // adore::mad::AFeed<adore::env::ControlledConnection> * tl_controller_feed_;
      // adore::fun::SetPointRequest spr_;
      // adore::env::AFactory::TParticipantSetReader* trafficReader_;

      std::unordered_set<std::string> plot_tags_old_;
      std::unordered_set<std::string> plot_tags_current_;
      
      
      DLR_TS::PlotLab::AFigureStub* figure_;
      unsigned long long counter_;
      // adore::env::BorderBased::BorderSet borderSet_;
      adore::env::VehicleMotionState9d position_;
      std::string prefix_;

      // adore::PLOT::GeoTiles * geoTiles_;
      // std::set<std::pair<int,int>> visibleTiles_;
      // std::unordered_map<std::string, std::pair<double,double>> visible_TCD_controller_;
      adore::env::OccupancyCylinderPredictionSet worst_case_predictions_;
      adore::env::OccupancyCylinderPredictionSet expected_predictions_;
      adore::env::OccupancyCylinderPredictionSet desired_predictions_;
      adore::env::AFactory::TOCPredictionSetReader* worst_case_prediction_reader_;
      adore::env::AFactory::TOCPredictionSetReader* desired_prediction_reader_;
      adore::env::AFactory::TOCPredictionSetReader* expected_prediction_reader_;
      

      int plotCount_;
      int eraseCount_;
      double fov_width_;
      double fov_height_;
      // int urlPreset_;
      double t_;
      double t_prediction_max_;

      adore::PLOT::PredictionConfig config_;

      // struct info
      // {
      //   std::string name_;
      //   bool visible_;
      //   info(bool visible,std::string name):name_(name),visible_(visible){}
      // };
      // std::map<adore::env::BorderBased::BorderID,info> plotMap_;

      // void unfocus(adore::env::BorderBased::BorderID id)
      // {
      //   auto result = plotMap_.find(id);
      //   if(result!=plotMap_.end())
      //   {
      //     if(result->second.visible_)
      //     {
      //       figure_->erase(result->second.name_);
      //     }
      //     plotMap_.erase(id);
      //   }
      // }


      // bool isVisible(adore::env::BorderBased::BorderID id)
      // {
      //   auto result = plotMap_.find(id);
      //   return result!=plotMap_.end() && result->second.visible_;
      // }
      // void setVisible(adore::env::BorderBased::BorderID id,bool visible)
      // {
      //   static long long counter = 0;
      //   auto result = plotMap_.find(id);
      //   if(result!=plotMap_.end())
      //   {
      //     result->second.visible_ = visible;
      //   }
      //   else
      //   {
      //     counter ++;
      //     std::stringstream ss;
      //     ss<<prefix_<<"/localmap/border"<<counter;
      //     plotMap_.emplace(std::make_pair(id,info(visible,ss.str())));
      //   }
      // }
      // std::string getName(adore::env::BorderBased::BorderID id)
      // {
      //   auto result = plotMap_.find(id);
      //   return result->second.name_;
      // }

      public:
      // FancyBird(DLR_TS::PlotLab::AFigureStub* figure,adore::params::APVehicle* pvehicle, adore::params::APMapProvider* pmap,std::string prefix,const adore::PLOT::FancyBirdConfig & config, const adore::PLOT::GeoTilesConfig geoTiles_config)
      PlotPredictions(DLR_TS::PlotLab::AFigureStub* figure,adore::params::APVehicle* pvehicle, std::string prefix,const adore::PLOT::PredictionConfig & config)
      {
        counter_=0;
        // tl_controller_feed_ = adore::env::EnvFactoryInstance::get()->getControlledConnectionFeed();
        // borderfeed_ = adore::env::EnvFactoryInstance::get()->getBorderFeed();
        positionReader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
        // sprReader_ = adore::fun::FunFactoryInstance::get()->getSetPointRequestReader();
        // trafficReader_ = adore::env::EnvFactoryInstance::get()->getTrafficParticipantSetReader();
        // pvehicle_= pvehicle;
        // pmap_ = pmap;
        figure_ = figure;      
        prefix_ = prefix;
        t_ = 0;
        t_prediction_max_ = config_.t_prediction_max; // TODO: reconsidder where to get this max from, currently a plot property
        fov_width_ = 640.0;
        fov_height_ = 480.0;
        // urlPreset_ = 0;
        config_ = config;
        // geoTiles_ = new adore::PLOT::GeoTiles(geoTiles_config.base_url,geoTiles_config.tile_width_meters);

        worst_case_prediction_reader_ = adore::env::EnvFactoryInstance::get()->getWorstCasePredictionSetReader();
        desired_prediction_reader_ = adore::env::EnvFactoryInstance::get()->getDesiredPredictionSetReader();
        expected_prediction_reader_ = adore::env::EnvFactoryInstance::get()->getExpectedPredictionSetReader();
      }

      ~PlotPredictions()
       {
        //  geoTiles_->~GeoTiles();
       }

      void run()
      {

        plot_tags_old_.insert(plot_tags_current_.begin(),plot_tags_current_.end());
        plot_tags_current_.clear();

        // adore::env::BorderBased::BorderSet newBorderSet_;
        // while(borderfeed_->hasNext())
        // {
          
        //   bool to_be_plotted = false;
        //   auto border = new adore::env::BorderBased::Border();
        //   borderfeed_->getNext(*border);
        //   borderSet_.insert_border(border,true);
        // }

        /// plot borders
        // for(auto it=borderSet_.getAllBorders();it.first!=it.second;it.first++)
        // {
        //     adore::env::BorderBased::Border* border = it.first->second;
        //     adore::env::BorderBased::Border* leftNeighbor = 0;
        //     bool do_plot = false;
        //     switch(border->m_type)
        //     {
        //       case adore::env::BorderBased::BorderType::DRIVING:
        //         do_plot = config_.plot_drive_lane;
        //         break;
        //       case adore::env::BorderBased::BorderType::EMERGENCY:
        //         do_plot = config_.plot_emergency_lane;
        //         break;
        //       case adore::env::BorderBased::BorderType::OTHER:
        //         do_plot = config_.plot_other_lane;
        //         break;
        //     }

        //     if(do_plot){
        //       if(border->m_left!=0)
        //       {
        //         leftNeighbor = borderSet_.getBorder(*(border->m_left));
        //         if (leftNeighbor != 0 && (!isVisible(border->m_id) || !isVisible(leftNeighbor->m_id)) )
        //         {
        //           setVisible(border->m_id,true);
        //           auto name = getName(border->m_id);
        //           if (border->m_type == adore::env::BorderBased::BorderType::DRIVING)
        //           {
        //             auto highlightRightBorder = (!borderSet_.hasRightNeighbor(border)) || (borderSet_.getRightNeighbor(border)->m_type != adore::env::BorderBased::BorderType::DRIVING);
        //             auto highlightLeftBorder = (!borderSet_.hasLeftNeighbor(leftNeighbor)) || (leftNeighbor->m_type != adore::env::BorderBased::BorderType::DRIVING);
        //             adore::PLOT::plotBorder_fancy(name,border,leftNeighbor,0.0,highlightLeftBorder,highlightRightBorder,config_,figure_);
        //           }
        //           else
        //           {
        //             adore::PLOT::plotBorder_fancy(name,border,leftNeighbor,0.0,false,false,config_,figure_);
        //           }
        //         }
        //       }  
        //     }
        // }

        if(positionReader_->hasUpdate())
        {
          positionReader_->getData(position_);
          t_ = position_.getTime();
          
          // //plot the vehicle
          // const double L = pvehicle_->get_a()+pvehicle_->get_b();
          // const double c = pvehicle_->get_c();
          // const double d = pvehicle_->get_d();
          // const double w = std::max(pvehicle_->get_wf(),pvehicle_->get_wr())*0.5;
          // plotPosition(prefix_+"/measuredPos",position_.getX(),position_.getY(),position_.getPSI(),L,c,d,w,"LineColor=0,0,1");

          // //unplot all borders, which are no longer visible
          // double R = pmap_->getVisibiltyRadius() + 100.0;//delete radius
          // double x0 = position_.getX() - R;
          // double x1 = position_.getX() + R;
          // double y0 = position_.getY() - R;
          // double y1 = position_.getY() + R;
          // for( auto it = borderSet_.getBordersOutsideRegion(x0,x1,y0,y1);it.current()!=it.end();it.current()++)
          // {
          //   unfocus(it.current()->second->m_id);
          // }
          // borderSet_.removeBorders(borderSet_.getBordersOutsideRegion(x0,x1,y0,y1));
        }

        // if(sprReader_->hasUpdate())
        // {
        //   sprReader_->getData(spr_);
        //   static const int N = 100;
        //   double X[N];
        //   double Y[N];
        //   int k = (std::ceil)((double)spr_.setPoints.size()/(double)N);
        //   int count=0;
        //   for(int i=0;i<N;i++)
        //   {
        //     int j=i*k;
        //     if(j<spr_.setPoints.size())
        //     {
        //       count = i+1;
        //       X[i] = spr_.setPoints[j].x0ref.getX();
        //       Y[i] = spr_.setPoints[j].x0ref.getY();
        //     }
        //   }
        //   figure_->plot(prefix_+"spr",X,Y,1.0,count,config_.setpoint_plotoptions);
        // }


        // if(trafficReader_->hasUpdate())
        // {
        //   double R = pmap_->getVisibiltyRadius() + 100.0;//delete radius
        //   adore::env::traffic::TParticipantSet tpset;
        //   trafficReader_->getData(tpset);
        //   for(auto& tp:tpset)
        //   {
        //     auto pos = tp.getCenter();
        //     auto psi = tp.getYaw();
        //     const double abcd = tp.getLength();
        //     const double c = 0.2 * abcd;
        //     const double d = 0.2 * abcd;
        //     const double L = abcd-c-d;
        //     const double w = tp.getWidth(); 
        //     pos(0) = pos(0) + cos(psi)*(-0.5*abcd+d);
        //     pos(1) = pos(1) + sin(psi)*(-0.5*abcd+d);
        //     // do not plot traffic outside of range
        //     if(abs(pos(0)-position_.getX())>R || abs(pos(1)-position_.getY())>R)
        //     {
        //       continue;
        //     }
              
        //     std::stringstream ss;
        //     ss<<prefix_<<"/traffic/"<<tp.getTrackingID();
        //     // std::cout<<"i: "<<i<<" traffic id="<<tp.getTrackingID()<<", x="<<pos(0)<<", y="<<pos(1)<<"\n";
        //     plotPosition(ss.str(),pos(0),pos(1),psi,L,c,d,w*0.5,"LineColor=0,0,0",false);
        //   }

        //   if(config_.plot_traffic_lights) plotTLConnections();

        // }

        /* Worst Case Prediction Feed */
        if(config_.worst_case_.active_ and worst_case_prediction_reader_->hasUpdate())
        {
          std::stringstream tag;
          tag << prefix_ << "/wcp";
          worst_case_prediction_reader_->getData(worst_case_predictions_);
          PLOT::plotPredictionSet(worst_case_predictions_,t_,t_prediction_max_,config_.expected_,tag.str(),figure_,plot_tags_current_);
         }

        // /* Expected Prediction Feed */
        if(config_.expected_.active_ and expected_prediction_reader_->hasUpdate())
        {
          std::stringstream tag;
          tag << prefix_ << "/ecp";
          expected_prediction_reader_->getData(expected_predictions_);
          PLOT::plotPredictionSet(expected_predictions_,t_,t_prediction_max_,config_.expected_,tag.str(),figure_,plot_tags_current_);
        }

        // /* Desired Prediction Feed */
        if(config_.desired_.active_ and desired_prediction_reader_->hasUpdate())
        {
          std::stringstream tag;
          tag << prefix_ << "/dcp";
          desired_prediction_reader_->getData(desired_predictions_);
          PLOT::plotPredictionSet(desired_predictions_,t_,t_prediction_max_,config_.expected_,tag.str(),figure_,plot_tags_current_);
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
      // void plotPosition(const std::string& name,double gX,double gY,double psi,double L,double c,double d,double w,const std::string& options,bool follow_vehicle=true)
      // {

      //   double X[12];
      //   double Y[12];
      //   X[0] = 0.0; X[1] = L; X[2] = L; X[3] = L + c; X[4] = L; X[5] = L + c; X[6] = L + c; X[7] = 0.0; X[8] = 0.0; X[9] = -d; X[10] = -d; X[11] = 0.0;
      //   Y[0] = -w; Y[1] = -w; Y[2] = +w; Y[3] = 0.0; Y[4] = -w; Y[5] = -w; Y[6] = +w; Y[7] = +w; Y[8] = -w; Y[9] = -w; Y[10] = +w; Y[11] = +w;

      //   double cpsi = std::cos(psi);
      //   double spsi = std::sin(psi);
      //   for(int i=0;i<12;i++)
      //   {
      //     double x = gX + cpsi * X[i] - spsi * Y[i];
      //     double y = gY + spsi * X[i] + cpsi * Y[i];
      //     X[i] = x;
      //     Y[i] = y;
      //   }
      //   figure_->plot(name,X,Y,0.25,12,options);
      //   plot_tags_current_.insert(name);

      //   double xmin = gX-fov_width_*0.5;
      //   double xmax = gX+fov_width_*0.5;
      //   double ymin = gY-fov_height_*0.5;
      //   double ymax = gY+fov_height_*0.5;

      //   if(follow_vehicle)
      //   {
      //     //remove tiles which are no longer visible
      //     std::vector<std::pair<int,int>> remove;
      //     for(auto it = visibleTiles_.begin();it!=visibleTiles_.end();it++)
      //     {
      //       if(!geoTiles_->overlapsBox(*it,xmin-10.0,ymin-10.0,xmax+10.0,ymax+10.0))
      //       {
      //         remove.push_back(*it);
      //         figure_->erase(geoTiles_->getPlotID(*it));
      //       }
      //     }
      //     for(auto it = remove.begin();it!=remove.end();it++)
      //     {
      //       visibleTiles_.erase(*it);
      //       eraseCount_ ++;
      //     }

      //     //find newly visible tiles
      //     int i0,j0,i1,j1;
      //     geoTiles_->getVisibleRange(xmin,ymin,xmax,ymax,i0,j0,i1,j1);
      //     for(int i=i0;i<=i1;i++)
      //     {
      //       for(int j=j0;j<=j1;j++)
      //       {
      //         auto id = std::make_pair(i,j);
      //         if(geoTiles_->overlapsBox(id,xmin,ymin,xmax,ymax))
      //         {
      //           if( visibleTiles_.find(id)==visibleTiles_.end() )
      //           {
      //             visibleTiles_.insert(id);
      //             std::string url = geoTiles_->getURL(id);
      //             std::string hashtag = geoTiles_->getPlotID(id);
      //             figure_->plotTexture(hashtag,url,geoTiles_->getCenterX(id),geoTiles_->getCenterY(id),-0.1,0.0,geoTiles_->getWidthM(),geoTiles_->getWidthM());
      //             plotCount_++;
      //           }
      //         }
      //       }
      //     }
      //   }

        
      //     // code from legacy version:
      //   	// auto p = &_input.MoSAIC.VehicleInstance[_MoSAIC_IA._Index].AssistanceAutomation.CSA.Parameters.FUN.VehicleParameters;
      //     // double l = p->a+p->b+p->c+p->d;
      //     // double rho = l*0.5-p->d;
        
      //   /** TODO fix parameters */
      //   auto length = L+c+d;
      //   auto rho = length*0.5-d;
      //   auto width = w*2;
      //   auto z = 1.1;        
      //   auto m_targetZoom = 20.0;

      //   if(follow_vehicle)
      //   {
      //     // figure_->plotTexture(name+"vehiclealphablend","fascare_shadow.png",gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
      //     figure_->plotTexture(name+"vehiclealphablend","../images/fascare_hq.png",gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
      //   }
      //   else
      //   {
      //     // figure_->plotTexture(name+"vehiclealphablend","fascare_shadow.png",gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
      //     std::string s = name+"/texture";
      //     figure_->plotTexture(s,"../images/sumocar.png",gX+std::cos(psi)*rho,gY+std::sin(psi)*rho,z,psi,width,length);
      //     plot_tags_current_.insert(s);
      //   }
      //   if (follow_vehicle && config_.followMode != 0)
      //   {
      //     figure_->setViewPortOffsets(gX,gY,360.0*(psi / (2.0 *3.14159))-90.0,m_targetZoom);
      //   }
      // }
      // virtual void plotTLConnections()
      // {          
      //   adore::env::VehicleMotionState9d vehicle_state;
      //   positionReader_->getData(vehicle_state);
      //   auto time = vehicle_state.getTime();
      //   int plot_distance = 200;

      //   removeDistanceBased(visible_TCD_controller_,plot_distance);

      //   while(tl_controller_feed_->hasNext())
      //     {

      //       double r = 0,g = 0, b = 0;
      //       std::stringstream ss_hashtag, ss_color;
      //       adore::env::ControlledConnection con;

      //       tl_controller_feed_->getNext(con);

      //       auto from = con.getID().getFrom();
      //       double x_from = from.get<0>();
      //       double y_from = from.get<1>();
            
      //       auto to = con.getID().getTo();
      //       double x_to = to.get<0>();
      //       double y_to = to.get<1>();

      //       std::pair<double,double> pos_coord = std::make_pair(x_from,y_from);
      //       auto distance_to_object = getDistanceToPoint(pos_coord);

      //       if(distance_to_object > plot_distance)
      //         continue;

      //       adore::env::ConnectionState::EConnectionState state = con.getState_byMinTime(time);
            
      //       switch (state)
      //       {
      //       //green
      //       case adore::env::ConnectionState::PERMISSIVE___MOVEMENT___ALLOWED:
      //         g = 1;
      //         break;
      //       //red
      //       case adore::env::ConnectionState::STOP___AND___REMAIN:
      //         r = 1;
      //         break;
      //       // yellow
      //       case adore::env::ConnectionState::PERMISSIVE__CLEARANCE:
      //         r = 1, g = 1;
      //         break;
      //       // red-yellow
      //       case adore::env::ConnectionState::PRE___MOVEMENT:
      //         r = 1, g = 0.5;
      //         break;
      //       default:
      //         b = 1;
      //       }

      //       ss_hashtag <<"#con--"<< (int) x_from <<"::"<< (int) y_from<<"::" << (int) x_to<<"::" << (int)y_to;
      //       ss_color << "LineColor=" << r << "," << g << "," << b;
   
      //       visible_TCD_controller_[ss_hashtag.str()] = std::make_pair(x_from,y_from);
            
      //       adore::PLOT::plotLine(ss_hashtag.str(),x_from,y_from,x_to,y_to,2,ss_color.str(),figure_);
      //     }
      // }

      // void removeDistanceBased(std::unordered_map<std::string, std::pair<double,double>> &visible_objects, int distance)
      // {
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
        
      // }
      // double getDistanceToPoint(std::pair<double,double> &point)
      // {
      //     adore::env::VehicleMotionState9d vehicle_state;
      //     positionReader_->getData(vehicle_state);
      //     double v_pos_x = vehicle_state.getX();
      //     double v_pos_y = vehicle_state.getY();

      //     double distance_x = v_pos_x - point.first;
      //     double distance_y = v_pos_y - point.second;

      //     return (std::sqrt((distance_x * distance_x) + (distance_y*distance_y)));
      // }
    
    };
  }
}