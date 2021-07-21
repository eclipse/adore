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
        std::array<double,2> t_refs = {cylinder.t0_ - t,cylinder.t1_ - t};
        // double t_ref1 = cylinder.t0_ - t;
        // double t_ref2 = cylinder.t1_ - t;

        DLR_TS::PlotLab::TwoFloatsPacked rg, ba;
        // if(t_ref1 <= t_max)
        for(auto t_ref : t_refs)
        {
            if (t_ref < t_max)
            {
                V1[i*2] = cylinder.x_;
                V1[i*2+1] = cylinder.rxy_;
                V2[i*2] = cylinder.y_;
                rg.fvalue[0] = config.getShade(0,t_ref,t_max);
                rg.fvalue[1] = config.getShade(1,t_ref,t_max);
                V2[i*2 + 1] = rg.dvalue;
                V3[i*2] = 1.0 + t_ref;
                ba.fvalue[0] = config.getShade(2,t_ref,t_max);
                ba.fvalue[1] = 1.0f;
                V3[i*2 + 1] = ba.dvalue;
                i++;
            }
        }
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