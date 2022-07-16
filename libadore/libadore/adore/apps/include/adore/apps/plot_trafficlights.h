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
#include <adore/apps/if_plotlab/plot_shape.h>
#include <adore/env/afactory.h>
#include <adore/mad/com_patterns.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/env/tcd/controlledconnection.h>
#include <adore/env/borderbased/borderset.h>
#include <ros/console.h>

namespace adore
{
  namespace apps
  {
    /**
     * @brief a optimzed plotting application to plot map borders, vehicles and environment information and background image satellite footage
     * 
     */
    class PlotTrafficLights
    {
      private:
      adore::mad::AReader<adore::env::VehicleMotionState9d>* positionReader_;
      adore::mad::AFeed<adore::env::ControlledConnection> * tl_controller_feed_;
      adore::mad::AFeed<adore::env::BorderBased::Border>* borderfeed_;
      bool plot_connection_lines = true;

      // TODO consider not plotting controlled connections by distance, would maybe need those lines:
      // std::unordered_set<std::string> plot_tags_old_;
      // std::unordered_set<std::string> plot_tags_current_;
      
      
      DLR_TS::PlotLab::AFigureStub* figure_;
      adore::env::VehicleMotionState9d position_;
      std::string prefix_;

      std::unordered_map<std::string, std::pair<double,double>> visible_TCD_controller_;
      adore::env::BorderBased::BorderSet borderSet_;

      public:
      PlotTrafficLights(DLR_TS::PlotLab::AFigureStub* figure,std::string prefix, bool connection_lines = false)
      {
          tl_controller_feed_ = adore::env::EnvFactoryInstance::get()->getControlledConnectionFeed();
          positionReader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
          borderfeed_ = adore::env::EnvFactoryInstance::get()->getBorderFeed();
          
          plot_connection_lines = connection_lines;
          figure_ = figure;
          prefix_ = prefix;
      }

      ~PlotTrafficLights()
       {
         delete positionReader_;
         delete tl_controller_feed_;
         delete borderfeed_;
         delete figure_;
       }

      void run()
      {
        // TODO consider not plotting controlled connections by distance, would maybe need those lines:
        // plot_tags_old_.insert(plot_tags_current_.begin(),plot_tags_current_.end());
        // plot_tags_current_.clear();

        if(positionReader_->hasUpdate())
        {
          positionReader_->getData(position_);
        }

        while (borderfeed_->hasNext())
        {
            auto border = new adore::env::BorderBased::Border();
            borderfeed_->getNext(*border);
            borderSet_.insert_border(border, true);
        }

          plotTLConnections();
      }

      virtual void plotTLConnections()
      {          
        auto time = position_.getTime();
        int plot_distance = 400;

        removeDistanceBased(visible_TCD_controller_,plot_distance);

        while(tl_controller_feed_->hasNext())
        {
            double r = 0,g = 0, b = 0;
            std::stringstream ss_hashtag, ss_color, ss_tl_icon;
            adore::env::ControlledConnection con;

            tl_controller_feed_->getNext(con);

            auto from = con.getID().getFrom();
            double x_from = from.get<0>();
            double y_from = from.get<1>();
            
            auto to = con.getID().getTo();
            double x_to = to.get<0>();
            double y_to = to.get<1>();

            std::pair<double,double> pos_coord = std::make_pair(x_from,y_from);
            auto distance_to_object = getDistanceToPoint(pos_coord);

            if(distance_to_object > plot_distance)
              continue;

            adore::env::ConnectionState::EConnectionState state = con.getState_byMinTime(time);
            ss_hashtag <<"#con::"<< (int) x_from <<"::"<< (int) y_from<<"::" << (int) x_to<<"::" << (int)y_to;
            ss_tl_icon << ss_hashtag.str() << "::icon";

            std::string tl_img;

            // if(state == adore::env::ConnectionState::DARK)
            // {
            //     figure_->erase(ss_hashtag.str());
            //     figure_->erase(ss_tl_icon.str());
            //     visible_TCD_controller_.erase(ss_hashtag.str());
            //     visible_TCD_controller_.erase(ss_tl_icon.str());
            //     continue;
            // }

            ROS_DEBUG_STREAM("Received new states.");
            ROS_DEBUG_STREAM("x_from: " << x_from << "; y_from: " << y_from);
            ROS_DEBUG_STREAM("state: " << state);

            // https://www.car-2-car.org/fileadmin/documents/Basic_System_Profile/Release_1.5.0/C2CCC_RS_2077_SPATMAP_AutomotiveRequirements.pdf
            switch (state)
            {
                case adore::env::ConnectionState::DARK:
                  r = 0; g = 0; b = 0;
                  tl_img = "../images/tl_yellow.png";
                  break;
                  // 2 - green
                case adore::env::ConnectionState::STOP___THEN___PROCEED:
                    g = 1; b = 1;
                    tl_img = "../images/tl_green.png";
                    break;
                    // 3 - red
                case adore::env::ConnectionState::STOP___AND___REMAIN:
                    r = 1;
                    tl_img = "../images/tl_red.png";
                    break;
                    // 4 - red-yellow
                case adore::env::ConnectionState::PRE___MOVEMENT:
                    r = 0.5, g = 0.5;
                    tl_img = "../images/tl_red_yellow.png";
                    break;
                // 5 - green with conflicts
                case adore::env::ConnectionState::PERMISSIVE___MOVEMENT___ALLOWED:
                  tl_img = "../images/tl_green.png";
                    g = 0.5;
                    break;
                // 6 - green
                case adore::env::ConnectionState::PROTECTED___MOVEMENT___ALLOWED:
                    tl_img = "../images/tl_green.png";
                    g = 1;
                    break;
                // 7 - yellow- conflicts possible
                case adore::env::ConnectionState::PERMISSIVE__CLEARANCE:
                    r = 0.5, g = 0.5;
                    tl_img = "../images/tl_yellow.png";
                    break;
                // 8 - yellow- clear
                case adore::env::ConnectionState::PROTECTED__CLEARANCE:
                    r = 1, g = 1;
                    tl_img = "../images/tl_yellow.png";
                    break;
                // 9 - green
                case adore::env::ConnectionState::CAUTION___CONFLICTING___TRAFFIC:
                    b = 1;
                    tl_img = "../images/tl_yellow.png";
                    break;
                default:
                    std::cout << "Unknown state: " << state << std::endl;
                     tl_img = "../images/tl_yellow.png";
                    b = 1;
                    break;
            }

            ss_color << "LineColor=" << r << "," << g << "," << b;

            double heading = std::numeric_limits<double>::quiet_NaN();

            heading = getHeadingFromMap(x_from,y_from);

            if(std::isnan(heading))
              heading = getHeadingOfLine(x_from,y_from,x_to,y_to);

            figure_->plotTexture(ss_tl_icon.str(), tl_img, x_from, y_from, 1, heading, 3.5, 1.5);
            visible_TCD_controller_[ss_tl_icon.str()] = std::make_pair(x_from,y_from);

            if(plot_connection_lines)
            {  
              adore::PLOT::plotLine(ss_hashtag.str(),x_from,y_from,x_to,y_to,2,ss_color.str(),figure_);
              visible_TCD_controller_[ss_hashtag.str()] = std::make_pair(x_from,y_from);
            }
      
          }
      }

      double getHeadingOfLine(double x1,double y1,double x2,double y2)
      {
          return std::atan2(y2 - y1, x2 - x1) + (1.5 * M_PI);
      }

      double getHeadingFromMap(double x, double y)
      {
          if (borderSet_.size() == 0)
              return std::numeric_limits<double>::quiet_NaN();

          adore::env::BorderBased::BorderSubSet borders_near_connection_point =
              borderSet_.getBordersAtPoint(x, y, 3);

          if (borders_near_connection_point.size() == 0)
              return std::numeric_limits<double>::quiet_NaN();
          
          adore::env::BorderBased::Border * pred;
          pred =  borders_near_connection_point[0];

          double x1 = pred->m_path->getData()(1,0);
          double y1 = pred->m_path->getData()(2,0);
          double x2 = pred->m_path->getData()(1,1);
          double y2 = pred->m_path->getData()(2,1);

          return getHeadingOfLine(x1,y1,x2,y2);
      }
      void removeDistanceBased(std::unordered_map<std::string, std::pair<double,double>> &visible_objects, int distance)
      {
        //remove tiles which are no longer visible
        std::vector<std::string> remove;

        for(auto it = visible_objects.begin();it!=visible_objects.end();it++)
        {
          auto coord = (*it).second;
          double distance_to_object = getDistanceToPoint(coord);
          if(distance_to_object > distance) remove.push_back((*it).first);
        }

        for(auto it = remove.begin();it!=remove.end();it++)
        { 
          figure_->erase((*it));
          visible_objects.erase((*it));
        }
        
      }
      double getDistanceToPoint(std::pair<double,double> &point)
      {
        
          double v_pos_x = position_.getX();
          double v_pos_y = position_.getY();

          double distance_x = v_pos_x - point.first;
          double distance_y = v_pos_y - point.second;

          return (std::sqrt((distance_x * distance_x) + (distance_y*distance_y)));
      }
    
    };
  }
}