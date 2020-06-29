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
#include <plotlablib/plcommands.h>
#include <adore/env/borderbased/borderset.h>
#include "plot_shape.h"


namespace adore
{

namespace PLOT
{

/**
* plotBorder - plots a border object, incl. left neighbor if !=0 into figure object
*/
inline void plotBorder(std::string name,adore::env::BorderBased::Border* right,adore::env::BorderBased::Border* left,double z,std::string options,DLR_TS::PlotLab::AFigureStub* figure,bool plotarrows = false)
{

    double X[DLR_TS::PlotLab::PLCom::max_size_points];
    double Y[DLR_TS::PlotLab::PLCom::max_size_points];
    double Xn[DLR_TS::PlotLab::PLCom::max_size_points];
    double Yn[DLR_TS::PlotLab::PLCom::max_size_points];
    int n = (std::min)(DLR_TS::PlotLab::PLCom::max_size_points/2,(int)right->m_path->getData().nc());
    right->m_path->writePointsToArray(0,n-1,0,X);
    right->m_path->writePointsToArray(0,n-1,1,Y);
    if(left!=0)
    {
        int m = (std::min)(DLR_TS::PlotLab::PLCom::max_size_points/2,(int)left->m_path->getData().nc());
        left->m_path->writePointsToArray(0,m-1,0,Xn);
        left->m_path->writePointsToArray(0,m-1,1,Yn);
        if(right->getNeighborDirection()==adore::env::BorderBased::Border::SAME_DIRECTION)
        {
            for(int i=0;i<m;i++)
            {
                X[n+m-i-1] = Xn[i];
                Y[n+m-i-1] = Yn[i];
            }
            if(plotarrows)plotArrow(name+"/a",(X[0]+Xn[0])/2.0,(Y[0]+Yn[0])/2.0,2.0,(X[n-1]+Xn[m-1])/2.0,(Y[n-1]+Yn[m-1])/2.0,1000.0,5.0/3.0,"LineWidth=5.0;LineColor=1,1,1",figure);
        }
        else
        {
            for(int i=0;i<m;i++)
            {
                X[n+i] = Xn[i];
                Y[n+i] = Yn[i];
            }
            if(plotarrows)plotArrow(name+"/a",(X[0]+Xn[m-1])/2.0,(Y[0]+Yn[m-1])/2.0,2.0,(X[n-1]+Xn[0])/2.0,(Y[n-1]+Yn[0])/2.0,1000.0,5.0/3.0,"LineWidth=5.0;LineColor=1,1,1",figure);
        }
        figure->patch(name+"/h",X,Y,z,n+m,options);

    }
    figure->plot(name,X,Y,z+0.1,n,"");
    X[1] = X[n-1];
    Y[1] = Y[n-1];
    //figure->plot(name+"/p",X,Y,z+0.1,2,"LineStyle=none;PointSize=5");
}
inline void plotBorderNavigation(adore::env::BorderBased::Border* right,adore::env::BorderBased::Border* left, double normedCost, DLR_TS::PlotLab::AFigureStub* figure)
{
    std::stringstream ss;
    ss << "FillColor=" << std::max<double>(std::min<double>(1,normedCost),0) << "," << std::min<double>(std::max<double>(0,1-normedCost),1) << ",0.0";
    plotBorder(right->m_id.toString(),right,left,0.5,ss.str().c_str(),figure);
}
inline void plotBorder(adore::env::BorderBased::Border* right,adore::env::BorderBased::Border* left,DLR_TS::PlotLab::AFigureStub* figure)
{
    std::string str = "";
    switch(right->m_type)
    {
    case adore::env::BorderBased::BorderType::DRIVING:
        {
            str = "FillColor=0.8,0.8,0.8";
            break;
        }
    case adore::env::BorderBased::BorderType::EMERGENCY:
        {
            str = "FillColor=1.0,0.5,0.15";
            break;
        }
    case adore::env::BorderBased::BorderType::OTHER:
        {
            "FillColor=1,1,1";
            break;
        }
    default:
        {
            str = str = "FillColor=0.75,0,0";
            break;
        }
    };
    plotBorder(right->m_id.toString(),right,left,0.0,str,figure);
}
/**
 * plots whole border set
 */
inline void plotBorderSet(adore::env::BorderBased::BorderSet &borderSet, DLR_TS::PlotLab::AFigureStub* figure)
{
    for(auto it=borderSet.getAllBorders();it.first!=it.second;it.first++)
    {
        adore::env::BorderBased::Border* border = it.first->second;
        adore::env::BorderBased::Border* neighbor = 0;
        if(border->m_left!=0)
        {
            neighbor = borderSet.getBorder(*(border->m_left));
        }
        plotBorder(border,neighbor,figure);
    }
}
/**
* unPlotBorder - removes border plot
*/
inline void unPlotBorder(adore::env::BorderBased::BorderID rightId,DLR_TS::PlotLab::AFigureStub* figure)
{
    figure->erase(rightId.toString());
    figure->erase(rightId.toString()+"/h");
    figure->erase(rightId.toString()+"/p");
}
/**
* unPlotBorder - removes border plot
*/
inline void unPlotBorder(adore::env::BorderBased::Border* right,DLR_TS::PlotLab::AFigureStub* figure)
{
    unPlotBorder(right->m_id, figure);
}
/**
* unPlotBorder - removes border plot
*/
inline void unPlotBorder(adore::env::BorderBased::Border* right,adore::env::BorderBased::Border* left,DLR_TS::PlotLab::AFigureStub* figure)
{
    unPlotBorder(right->m_id, figure);
}


} // end PLOT namespace

} // end adore namespace