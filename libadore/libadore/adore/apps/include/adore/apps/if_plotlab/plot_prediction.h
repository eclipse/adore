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
 *   Robert Markowski
 ********************************************************************************/
#pragma once

#include <plotlablib/figurestubfactory.h>
#include <plotlablib/utility.h>
#include <adore/env/traffic/occupancycylinderprediction.h>
#include "prediction_config.h"
#include "plot_shape.h"
#include <unordered_set>

namespace adore
{

namespace PLOT
{

inline void plotCylinderTree(adore::mad::OccupancyCylinderTree tree, double t, double t_max, PLOT::PredictionConfig::prediction_config config, std::string prefix, DLR_TS::PlotLab::AFigureStub* figure, std::unordered_set<std::string> &plot_tags)
{
    unsigned int i=0;
    unsigned int stripCount=1;
    std::string incomplete_format = "LineWidth=1.0;LineColor=";
    int cylinder_count = tree.getLevel(0).size();
    double V1[500];
    double V2[500];
    double V3[500];

    std::ostringstream tag;
    tag << prefix << stripCount;

    // reminder: CircleStrip has a special data layout:
    // given a circle with center x,y,z and radius r and color rgba (1 float per channel)
    // v1[0] = x  v2[0] = y             v3[0] = z
    // v1[1] = r  v2[1] = rg (2 floats) v3[1] = ba (2 floats)
    // then the pattern repeats

    for(const auto& element:tree.getLevel(0))
    {
        const auto& cylinder = element.second;

        DLR_TS::PlotLab::TwoFloatsPacked rg, ba;
        double ti = adore::mad::bound(0.0,cylinder.t0_ - t,t_max);
        V1[i*2] = cylinder.x_;
        V1[i*2+1] = std::max(0.1,cylinder.rxy_-0.1);
        V2[i*2] = cylinder.y_;
        rg.fvalue[0] = config.getShade(0,ti,t_max,0.0);
        rg.fvalue[1] = config.getShade(1,ti,t_max,0.0);
        V2[i*2 + 1] = rg.dvalue;
        V3[i*2] = 1.0 + std::max(cylinder.t0_-t,0.0);
        ba.fvalue[0] = config.getShade(2,ti,t_max,0.0);
        ba.fvalue[1] = 1.0f;
        V3[i*2 + 1] = ba.dvalue;
        i++;

        ti = adore::mad::bound(0.0,cylinder.t1_ - t,t_max);
        V1[i*2] = cylinder.x_;
        V1[i*2+1] = std::max(0.1,cylinder.rxy_);
        V2[i*2] = cylinder.y_;
        rg.fvalue[0] = config.getShade(0,ti,t_max,0.0);
        rg.fvalue[1] = config.getShade(1,ti,t_max,0.0);
        V2[i*2 + 1] = rg.dvalue;
        V3[i*2] = 1.0 + std::max(cylinder.t1_-t,0.0);
        ba.fvalue[0] = config.getShade(2,ti,t_max,0.0);
        ba.fvalue[1] = 1.0f;
        V3[i*2 + 1] = ba.dvalue;
        i++;

        if (i > 127)
        {
            // std::cout << " warning: CylinderTree strip with too many cylinders: " << tree.getLevel(0).size() << std::endl;
            // break;
            plot_tags.insert(tag.str());
            figure->circlestrip(tag.str(),V1,V2,V3,i*2,"");
            tag.clear();
            stripCount++;
            tag << prefix << stripCount;
            i = 0;
        }
    }
    if (i > 0)
    {    
        // DLR_TS::PlotLab::TwoFloatsPacked drg, dba;
        // drg.dvalue = V2[1];
        // dba.dvalue = V3[1];
        // std::cout << " #DEBUG plot_prediction.h - 1st circle" << std::endl;
        // std::cout << " x " << V1[0] << " \ty " << V2[0] << " z " << V3[0] << std::endl;
        // std::cout << " r " << V1[1] << " \tr " << drg.fvalue[0] << " \tg " << drg.fvalue[1] << " \tb " << dba.fvalue[0] << " \ta " << dba.fvalue[1] << std::endl;

        // drg.dvalue = V2[i*2-1];
        // dba.dvalue = V3[i*2-1];
        // std::cout << " #DEBUG plot_prediction.h - last circle" << std::endl;
        // std::cout << " x " << V1[i*2-2] << " \ty " << V2[i*2-2] << " z " << V3[i*2-2] << std::endl;
        // std::cout << " r " << V1[i*2-1] << " \tr " << drg.fvalue[0] << " \tg " << drg.fvalue[1] << " \tb " << dba.fvalue[0] << " \ta " << dba.fvalue[1] << std::endl;
        // std::cout << " #DEBUG plot_prediction.h striplength " << tree.getLevel(0).size() << " i " << i << std::endl;
        plot_tags.insert(tag.str());
        figure->circlestrip(tag.str(),V1,V2,V3,i*2,"");
    }
 
}

inline void plotCylinderTree_onlyMean(adore::mad::OccupancyCylinderTree tree, double dz, PLOT::PredictionConfig::prediction_config config, std::string prefix, DLR_TS::PlotLab::AFigureStub* figure, std::unordered_set<std::string> &plot_tags)
{
    unsigned int i=0;
    unsigned int stripCount=1;
    std::ostringstream format;
    format<< "LineWidth=3.0;LineColor="<<config.r_.start_<<","<<config.g_.start_<<","<<config.b_.start_;
    int cylinder_count = tree.getLevel(0).size();
    static const int max_size = 128;
    static const int arrow_head_count = 3;
    double V1[500];
    double V2[500];
    double V3[500];
    static const double psi = 150.0/180.0*M_PI;//arrow head angle
    static const double cpsi = std::cos(psi);
    static const double spsi = std::sin(psi);


    std::ostringstream tag;
    tag << prefix << stripCount;

    for(const auto& element:tree.getLevel(0))
    {
        const auto& cylinder = element.second;
        V1[i] = cylinder.x_;
        V2[i] = cylinder.y_;
        V3[i] = 0.5*(cylinder.z0_+cylinder.z1_) + dz;
        i++;


        if (i >= max_size-arrow_head_count)
        {
            const double x1 = V1[i-1];
            const double y1 = V2[i-1];
            const double x0 = V1[i-2];
            const double y0 = V2[i-2];
            double li = 1.0/std::sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
            const double ex = (x1-x0)*li;
            const double ey = (y1-y0)*li;
            V1[i] = V1[i-1] + ex * cpsi - ey * spsi;
            V2[i] = V2[i-1] + ex * spsi + ey * cpsi;
            V3[i] = V3[i-1];
            V1[i+1] = V1[i-1];
            V2[i+1] = V2[i-1];
            V3[i+1] = V3[i-1];
            V1[i+2] = V1[i-1] + ex * cpsi + ey * spsi;
            V2[i+2] = V2[i-1] - ex * spsi + ey * cpsi;
            V3[i+2] = V3[i-1];
            plot_tags.insert(tag.str());
            figure->plot(tag.str(),V1,V2,V3,i+3,format.str());
            tag.clear();
            stripCount++;
            tag << prefix << stripCount;
            i = 0;
        }
    }
    if (i > 1)
    {    
            const double x1 = V1[i-1];
            const double y1 = V2[i-1];
            const double x0 = V1[i-2];
            const double y0 = V2[i-2];
            double li = 1.0/std::sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
            const double ex = (x1-x0)*li;
            const double ey = (y1-y0)*li;
            V1[i] = V1[i-1] + ex * cpsi - ey * spsi;
            V2[i] = V2[i-1] + ex * spsi + ey * cpsi;
            V3[i] = V3[i-1];
            V1[i+1] = V1[i-1];
            V2[i+1] = V2[i-1];
            V3[i+1] = V3[i-1];
            V1[i+2] = V1[i-1] + ex * cpsi + ey * spsi;
            V2[i+2] = V2[i-1] - ex * spsi + ey * cpsi;
            V3[i+2] = V3[i-1];
            plot_tags.insert(tag.str());
            figure->plot(tag.str(),V1,V2,V3,i+3,format.str());
    }
 
}

inline void plotPredictionSet(adore::env::OccupancyCylinderPredictionSet set, double t, double t_max, PLOT::PredictionConfig::prediction_config config, std::string subtopic, DLR_TS::PlotLab::AFigureStub* figure, std::unordered_set<std::string> &plot_tags)
{
    // unsigned int totalPredictionstripsCount = 0;
    for(const auto& prediction : set)
    {
        std::stringstream ss;
        ss<<subtopic<<"/"<<prediction.trackingID_<<"/"<<prediction.branchID_<<"/";
        plotCylinderTree(prediction.occupancy_, t, t_max, config, ss.str(), figure,plot_tags);
        // plot_tags.insert(ss.str());
        // totalPredictionstripsCount++;
    }
    // std::cout << " #debug plot_prediction.h total count of prediction strips: " << totalPredictionstripsCount << std::endl;
}


inline void plotPredictionSetMinimal(adore::env::OccupancyCylinderPredictionSet set, double dz, PLOT::PredictionConfig::prediction_config config, std::string subtopic, DLR_TS::PlotLab::AFigureStub* figure, std::unordered_set<std::string> &plot_tags)
{
    for(const auto& prediction : set)
    {
        std::stringstream ss;
        ss<<subtopic<<"/"<<prediction.trackingID_<<"/"<<prediction.branchID_<<"/";
        plotCylinderTree_onlyMean(prediction.occupancy_, dz, config, ss.str(), figure,plot_tags);
    }
}

// inline void unplotPrediction(std::string prefix, int trackingID, int branchID, DLR_TS::PlotLab::AFigureStub* figure)
// {
//     std::stringstream ss;
//     ss << prefix << "/" << trackingID << "/" << branchID << "/";
//     figure->erase_similar(ss.str());
// }

// inline void unplotPredictionSet(std::string prefix, adore::env::OccupancyCylinderPredictionSet old_set, adore::env::OccupancyCylinderPredictionSet new_set, DLR_TS::PlotLab::AFigureStub* figure)
// {
//     for(auto itPred=old_set.begin(); itPred!=old_set.end(); itPred++)
//     {
//         auto prediction = *itPred;
//         bool outdated = true;
//         for(auto itNew=new_set.begin(); itNew!=new_set.end(); itNew++)
//         {
//             if(prediction.trackingID_ == itNew->trackingID_ && prediction.branchID_ == itNew->branchID_)
//             {
//                 outdated = false;
//                 continue;
//             }
//         }
//         if(outdated)
//         {
//             std::stringstream ss;
//             ss << prefix << "/" << prediction.trackingID_ << "/" << prediction.branchID_ << "/";
//             figure->erase_similar(ss.str());
//         }
//     }
// }

}

}