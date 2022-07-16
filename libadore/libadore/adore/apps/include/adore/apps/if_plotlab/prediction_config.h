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
 *   Thomas Lobig - initial implementation
 ********************************************************************************/

#pragma once
#include <string>

namespace adore
{
  namespace PLOT
  {

    class PredictionConfig
    {
        public:
        struct prediction_config
        {
          struct color_range
          {
            double start_;
            double end_;
            color_range()
            {
              start_ = 0;
              end_ = 0;                
            }
            double getShade(double x, double x_max, double x_min = 0)
            {
              // if (start_ < end_)
                return start_+(x-x_min)/(x_max-x_min)*(end_-start_);
              // else
              //   return end_+(x-x_min)/(x_max-x_min)*(end_-start_);
            }
          };
          // ranges of RGB
          color_range r_,g_,b_;
          bool active_;
          prediction_config()
          {
            active_ = true;
          }
          double getShade(int rgb_index, double x, double x_max, double x_min = 0)
          {
            switch(rgb_index)
            {
              case 0:
                return r_.getShade(x,x_max,x_min);
              case 1:
                return g_.getShade(x,x_max,x_min);
              case 2:
                return b_.getShade(x,x_max,x_min);
              default:
                throw;
            }
          }
          
        };

        public:
        prediction_config desired_;
        prediction_config expected_;
        prediction_config worst_case_;
        prediction_config ego_;
        prediction_config static_;
        
        double t_prediction_max = 10.0;

        PredictionConfig()
        {
          // followMode = false;
          desired_.b_.start_    = 1;
          desired_.b_.end_      = 1;

          expected_.g_.start_   = 1;
          expected_.g_.end_     = 1;

          worst_case_.r_.start_ = 1;
          worst_case_.r_.end_   = 1;

          ego_.r_.start_        = 1;
          ego_.r_.end_          = 1;

          static_.r_.start_        = 1;
          static_.r_.end_          = 1;
          static_.b_.start_        = 1;
          static_.b_.end_          = 1;
        }
    };
  }
}