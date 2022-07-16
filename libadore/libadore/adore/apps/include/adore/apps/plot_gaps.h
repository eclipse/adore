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
#include <plotlablib/figurestubfactory.h>
#include <adore/env/afactory.h>
#include <unordered_set>
#include <vector>


namespace adore
{
  namespace apps
  {
    class GapQueuePlotter
    {
        private:
        std::unordered_set<std::string> current_tags_;
        std::vector<std::string> last_tags_;
        DLR_TS::PlotLab::AFigureStub* figure_;
        std::string prefix_;
        adore::env::AFactory::TGapQueueReader* gapReader_;      
        adore::env::AFactory::TVehicleMotionStateReader* egoStateReader_;
        public:
        GapQueuePlotter(DLR_TS::PlotLab::AFigureStub* figure,
                        std::string prefix,
                        adore::env::AFactory::TGapQueueReader* gapReader,
                        adore::env::AFactory::TVehicleMotionStateReader* egoStateReader)
        : figure_(figure),prefix_(prefix),gapReader_(gapReader),egoStateReader_(egoStateReader)
        {}
        void update()
        {
            adore::env::VehicleMotionState9d x;
            egoStateReader_->getData(x);
            adore::env::GapQueue q;
            if(gapReader_->hasUpdate())
            {   
                int i=0;
                gapReader_->getData(q);
                double rating_min = 1e10;
                double rating_max =-1e10;
                for( auto& g:q )
                {
                    rating_min = (std::min)(rating_min,g.rating);
                    rating_max = (std::max)(rating_max,g.rating);
                }
                for( auto& g:q )
                {
                    std::stringstream ss;
                    ss<<prefix_<<"/"<<i++;
                    plot(g,x,ss.str(),rating_min,rating_max);
                }
                for( auto& s: last_tags_)if(current_tags_.find(s)==current_tags_.end())figure_->erase(s);
                last_tags_.clear();
                for( auto& s: current_tags_) last_tags_.push_back(s);
                current_tags_.clear();
            }

        }
        void plot(adore::env::GapData& g,adore::env::VehicleMotionState9d x,std::string tag,double rating_min,double rating_max)
        {
            current_tags_.emplace(tag);
            static const int N=6;//number of plot points with arrow
            static const double a = 1.0;//length of arrow shaft
            static const double b = 0.5;//length of arrow head
            static const double rmin = 0.0;//color mix
            static const double rmax = 1.0;
            static const double gmin = 0.7;
            static const double gmax = 0.0;
            static const double bmin = 0.0;
            static const double bmax = 0.0;
            double X[N];
            double Y[N];
            X[0] = x.getX();
            Y[0] = x.getY();
            X[1] = g.anchor_X;
            Y[1] = g.anchor_Y;
            const double L = std::sqrt(g.anchor_dX *g.anchor_dX + g.anchor_dY*g.anchor_dY);
            std::stringstream options;
            if(rating_max<=rating_min)
            {
                options<<"LineColor=0,0.7,0;LineWidth=1";
            }
            else
            {
                const double p = (g.rating-rating_min)/(rating_max-rating_min);
                options<<"LineColor=";
                options<<( p * (rmax-rmin) + rmin)<<",";
                options<<( p * (gmax-gmin) + gmin)<<",";
                options<<( p * (bmax-bmin) + bmin)<<";";
                options<<"LineWidth=1";
            }
            if(L>0.1)
            {
                const double hx =  std::cos(M_PI*0.75) * g.anchor_dX/L * b - std::sin(M_PI*0.75) * g.anchor_dY/L * b;
                const double hy =  std::sin(M_PI*0.75) * g.anchor_dX/L * b + std::cos(M_PI*0.75) * g.anchor_dY/L * b;
                X[2] = g.anchor_X + g.anchor_dX/L * a;
                Y[2] = g.anchor_Y + g.anchor_dY/L * a;
                X[3] = X[2] + hx;
                Y[3] = Y[2] + hy;
                X[4] = X[2];
                Y[4] = Y[2];
                X[5] = X[2] - hy;
                Y[5] = Y[2] + hx;
                figure_->plot(tag,X,Y,x.getZ()+2.0,N,options.str());
            }
            else
            {
                figure_->plot(tag,X,Y,x.getZ()+2.0,2,options.str());
            }
        }
    };
    /**
     * @brief plots markers for traffic gaps 
     * 
     */
    class PlotGaps
    {
        private:
        adore::env::AFactory::TGapQueueReader* rightGapsReader_;            
        DLR_TS::PlotLab::AFigureStub* figure_;
        std::string prefix_;
        GapQueuePlotter* leftPlotter_;
        GapQueuePlotter* rightPlotter_;

        public:

        PlotGaps(std::string prefix):prefix_(prefix)
        {
            DLR_TS::PlotLab::FigureStubFactory fig_factory;
            figure_ = fig_factory.createFigureStub(2);
            auto egoStateReader = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
            leftPlotter_ = new GapQueuePlotter(figure_,prefix_+"/left",
                                adore::env::EnvFactoryInstance::get()->getGapQueueReaderLeftLane(),
                                egoStateReader);
            rightPlotter_ = new GapQueuePlotter(figure_,prefix_+"/right",
                                adore::env::EnvFactoryInstance::get()->getGapQueueReaderRightLane(),
                                egoStateReader);
        }
        ~PlotGaps()
        {
            delete leftPlotter_;
            delete rightPlotter_;
            delete figure_;
        }
        void update()
        {
            leftPlotter_->update();
            rightPlotter_->update();
        }
    };
  }
}