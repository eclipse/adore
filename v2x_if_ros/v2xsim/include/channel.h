/****************************************************************************/
// Eclipse ADORe, Automated Driving Open Research; see https://eclipse.org/adore
// Copyright (C) 2017-2020 German Aerospace Center (DLR).
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    channel.h
/// @author  Daniel Heß
/// @contact Daniel.Hess@DLR.de
/// @date 2020/04/02
/// @initialrelease X.X.X
///
// Channel class models probabilistic message transfer for v2x
#pragma once
#include <ros/ros.h>
#include <string>
#include <list>
#include <algorithm>
#include "station.h"
#include <v2xsim/V2XMetaSim.h>
#include <limits>
#include <random>
// #include <boost/random.hpp>

namespace v2xsim
{
    /**
     * Channel class models probabilistic message transfer for v2x.
     * The channel is modeled from the perspective of a single station with an associated position.
     * The channel model is based on the winner+ UMi (B1) LOS model with 3 dB log-normal shadowing
     * The model can be found here: 
     * Heino, P. & Meinilä, Juha & Kyösti, Pekka & Hentila, Lassi & Jämsä, Tommi & Suikkanen, Essi & Kunnari, E. & Narandzic, Milan. (2010). CP5-026 WINNER+ D5.3 v1.0 WINNER+ Final Channel Models. 
     */
    class Channel
    {
        private:
        ros::NodeHandle n_;
        Station* station_;
        std::list<V2XMetaSim> event_list_;
        std::default_random_engine generator;
        double t_agg_;
        double msgs_per_second_;
        double bytes_per_second_;
        double cutoff_distance_;
        double height_antenna_a; 
        double height_antenna_b; 
        double center_frequency; 
        double c0;  
        double tx_power; 
        double rx_sensivity; 
        double fading_std;
        
        void update_event_list(const v2xsim::V2XMetaSim& meta)
        {
            //erase old events
            double now = station_->getT();
            auto it = event_list_.begin();
            while(it!=event_list_.end())
            {
                if(it->time<now-t_agg_)
                {
                    it = event_list_.erase(it);
                }
                else
                {
                    it ++;
                }
            }
            //push back new event
            event_list_.push_back(meta);
            //sort by time
            //event_list_.sort([](const Tevent& a,const Tevent&b){return a.first<b.first;})
            //compute metrics
            msgs_per_second_ = event_list_.size()/t_agg_;
            bytes_per_second_ = 0.0;
            for(auto it=event_list_.begin();it!=event_list_.end();it++)bytes_per_second_+=it->bytecount;
            bytes_per_second_ /= t_agg_;
        }
        public:
        Channel(ros::NodeHandle n,Station* station,unsigned int seed):station_(station),n_(n)
        {
            t_agg_ = 0.1;
            msgs_per_second_ = 0.0;
            bytes_per_second_ = 0.0;
            cutoff_distance_ = 500.0;
            /*
            * Effektive antennaheigt is: heigt -1 
            * in mobil environment the min effective antenna height should be 1 m 
            * worst case assumption for SPATEM and MAPEM from RSU
            * RSU antenna is typically higher
            * ToDo: The stations should contain a antenna hight information
            */
            height_antenna_a = 2; 
            height_antenna_b = 2; 
             
            center_frequency = 5.9 ; // [GHz] Unit !
            c0 = 3 * std::pow(10, 8); //speed of light 
            tx_power = 23; //dBm
            rx_sensivity = -90; //dBm from cohda data sheet should be handled with care.  
            fading_std = 3; //dB
            
            generator.seed(seed);
        }

        void notify(const v2xsim::V2XMetaSim& meta,bool& received,double& delay)
        {
            const double dx = meta.location.x-station_->getX();
            const double dy = meta.location.y-station_->getY();
            const double dz = meta.location.z-station_->getZ();
            double distance = std::sqrt(dx*dx+dy*dy+dz*dz); 
            double breakpoint_distance =4*(height_antenna_a-(double)1)*(height_antenna_b-(double)1)*center_frequency*std::pow(10,9)/c0;
            double pathloss = std::numeric_limits<double>::max();

            if (distance<10) 
            // under correlation distance the modell is not valid but in this case we will not have any trouble with the distacne or power
            {
                pathloss = 70; 
                //placeholder with freespace loss value
            }
            else if (distance<breakpoint_distance)
            {
                pathloss = 22.7* std::log10(distance) + 27 + 20* std::log10(center_frequency) ;
            }
            else
            {
                pathloss = 40* std::log10(distance) + 7,56 -17.3*std::log10(height_antenna_a-(double)1) - 17.3*std::log10(height_antenna_b-(double)1) + 2.7 *std::log10(center_frequency);
            }
            std::normal_distribution<double> distribution(0,fading_std);
            double fading = distribution(generator);
            double pathloss_faded = pathloss+ fading;
            double received_power = tx_power - pathloss_faded + 2* 5; 
            //5 dBi  is the typical antenna gain  in mobil v2x antennas 2 times for tx and rx
            // std::cout<<"Distance: "<<distance<<" Pathloss: "<<pathloss <<" Fadind: "<<fading <<" Pathloss + Fading: "<<pathloss_faded  <<std::endl;
            if (received_power < rx_sensivity )
            {
                received = false;
                return;
            }
            update_event_list(meta);
            received = true;
        }
    };
}