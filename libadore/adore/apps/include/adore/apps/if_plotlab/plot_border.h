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
#include "laneplot_config.h"

namespace adore
{

namespace PLOT
{

/**
* plotBorder - plots a border object, incl. left neighbor if !=0 into figure object
*/
inline void plotBorder(std::string name,adore::env::BorderBased::Border* right,adore::env::BorderBased::Border* left,double z,std::string options,DLR_TS::PlotLab::AFigureStub* figure,bool plotarrows = false)
{
    static const int Nmax = DLR_TS::PlotLab::PLCom::max_size_points;
    double X[Nmax];
    double Y[Nmax];
    double Xt[Nmax];
    double Yt[Nmax];
    double Xn[Nmax];
    double Yn[Nmax];
    int n = (std::min)(Nmax/2,(int)right->m_path->getData().nc());
    right->m_path->writePointsToArray(0,n-1,0,X);
    right->m_path->writePointsToArray(0,n-1,1,Y);
    if(left!=0)
    {
        int m = (std::min)(Nmax/2,(int)left->m_path->getData().nc());
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

        //the patch variant
        // figure->patch(name+"/h",X,Y,z,n+m,options);

        //the tristrip variant
        double rightsmin = right->m_path->limitLo();
        double rightsmax = right->m_path->limitHi();
        double leftsmin = left->m_path->limitLo();
        double leftsmax = left->m_path->limitHi();
        int k = Nmax/2;
        for(int i=0;i<k;i++)
        {
            double sright = rightsmin + (rightsmax-rightsmin)/(double)(k-1)*(double)i;
            double sleft;
            if(right->getNeighborDirection()==adore::env::BorderBased::Border::SAME_DIRECTION)
            {
                sleft = leftsmin + (leftsmax-leftsmin)/(double)(k-1)*(double)i;
            } 
            else
            {
                sleft = leftsmax - (leftsmax-leftsmin)/(double)(k-1)*(double)i;
            }
            auto rightpos = right->m_path->f(sright);
            auto leftpos = left->m_path->f(sleft);
            Xt[i*2+0] = rightpos(0);
            Xt[i*2+1] = leftpos(0);
            Yt[i*2+0] = rightpos(1);
            Yt[i*2+1] = leftpos(1);          
        }
        figure->tristrip(name+"/h",Xt,Yt,z,k*2,options);
    }
    figure->plot(name,X,Y,z+0.1,n,"");
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
            str = "FillColor=1,1,1";
            break;
        }
    default:
        {
            str = "FillColor=0.75,0,0";
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

/**
 * @brief plots a border object, including left neighbor if !=0 into figure object
*/
void plotBorder_fancy(std::string name,adore::env::BorderBased::Border* right,adore::env::BorderBased::Border* left,double z,bool outlineLeft, bool outlineRight, adore::PLOT::LanePlotConfig config, DLR_TS::PlotLab::AFigureStub* figure)
{
    static const int max_points = DLR_TS::PlotLab::PLCom::max_size_points;
    int n = 0;
    int m = 0;
    std::string options = "";

    if(right!=0)
    {
        switch(right->m_type)
        {
        case adore::env::BorderBased::BorderType::DRIVING:
            {
                options = config.lane_fill_driveable_plotoptions;
                break;
            }
        case adore::env::BorderBased::BorderType::EMERGENCY:
            {
                options = config.lane_fill_emergency_plotoptions;
                break;
            }
        case adore::env::BorderBased::BorderType::OTHER:
            {
                options = config.lane_fill_other_plotoptions;
                break;
            }
        default:
            {
                options = config.lane_fill_default_plotoptions;
                break;
            }
        }
    }
    else
    {
        options = config.lane_fill_default_plotoptions;        
    }
    


    // // MATRX VARIANTE 1
    adoreMatrix<double,4L,0L> *r_data;
    adoreMatrix<double,4L,0L> *l_data;
    if (right!= 0)
    {
        r_data = &(right->m_path->getData());
        n = right->m_path->getData().nc();
    }
    if (left!=0)
    {
        l_data = &(left->m_path->getData());
        m = left->m_path->getData().nc();
    }
    // TODO investigate and fix : the following block seems to be no longer in use
    // if(n!=0 && m!=0)
    // {
    //     double XX[max_points];
    //     double YY[max_points];
    //     double * x_left = &((*l_data)(1,0));
    //     double * y_left = &((*l_data)(2,0));
    //     double * x_right = &((*r_data)(1,0));
    //     double * y_right = &((*r_data)(2,0));
    //     for (int i = 0; i < n; i++)
    //     {
    //     XX[i] = x_right[i];
    //     YY[i] = y_right[i];
    //     }
    //     if(right->getNeighborDirection()==adore::env::BorderBased::Border::SAME_DIRECTION)
    //     {
    //     for(int i=0;i<m;i++)
    //     {
    //         XX[n+m-i-1] = x_left[i];
    //         YY[n+m-i-1] = y_left[i];
    //     }
    //     }
    //     else  
    //     {
    //     for(int i=0;i<m;i++)
    //     {
    //         XX[n+i] = x_left[i];
    //         YY[n+i] = y_left[i];
    //     }
    //     }
    //     // figure->patch(name+"/h",XX,YY,z,n+m,options);
    // }

    if(right!=0 && left!=0)
    {
        //the tristrip variant
        double Xt[max_points];
        double Yt[max_points];
        double rightsmin = right->m_path->limitLo();
        double rightsmax = right->m_path->limitHi();
        double leftsmin = left->m_path->limitLo();
        double leftsmax = left->m_path->limitHi();
        int k = max_points/2;
        for(int i=0;i<k;i++)
        {
            double sright = rightsmin + (rightsmax-rightsmin)/(double)(k-1)*(double)i;
            double sleft;
            if(right->getNeighborDirection()==adore::env::BorderBased::Border::SAME_DIRECTION)
            {
                sleft = leftsmin + (leftsmax-leftsmin)/(double)(k-1)*(double)i;
            } 
            else
            {
                sleft = leftsmax - (leftsmax-leftsmin)/(double)(k-1)*(double)i;
            }
            auto rightpos = right->m_path->f(sright);
            auto leftpos = left->m_path->f(sleft);
            Xt[i*2+0] = rightpos(0);
            Xt[i*2+1] = leftpos(0);
            Yt[i*2+0] = rightpos(1);
            Yt[i*2+1] = leftpos(1);          
        }
        figure->tristrip(name+"/h",Xt,Yt,z,k*2,options);
    }


    if ( outlineRight )
    {
        figure->plot(name+"/r",&((*r_data)(1,0)),&((*r_data)(2,0)),z+0.1,n,config.border_outer_right_plotoptions);
    }
    if ( outlineLeft )
    {
        figure->plot(name+"/l",&((*l_data)(1,0)),&((*l_data)(2,0)),z+0.1,m,config.border_outer_left_plotoptions);
    }
}


} // end PLOT namespace

} // end adore namespace