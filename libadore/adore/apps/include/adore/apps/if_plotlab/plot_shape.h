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
#include <adore/mad/adoremath.h>
#include <adore/fun/setpointrequest.h>

namespace adore
{

namespace PLOT
{

inline void plotPosition(std::string tag, double x, double y, DLR_TS::PlotLab::AFigureStub* figure, 
    std::string options="LineColor=0,0,0;LineWidth=0;FillColor=1,0,0", double r=0.5)
{
    const int numPoints = 16;
    
    static double cosAngle[numPoints],sinAngle[numPoints];
    
    if(cosAngle[1] == 0.0)
    {
        for(int i = 0; i<numPoints; i++)
        {
            double angle = 2.0*M_PI*i/numPoints;
            cosAngle[i] = cos(angle);
            sinAngle[i] = sin(angle);
        }
    }
    
    double X[numPoints], Y[numPoints], Z[numPoints];
    for(int i = 0; i<numPoints; i++)
    {
        X[i] = x+cosAngle[i]*r;
        Y[i] = y+sinAngle[i]*r;
        Z[i] = 1;
    }
    figure->patch(tag,X,Y,Z,numPoints,options);
}

inline void plotCircle(std::string tag, double x, double y, double z, double r, DLR_TS::PlotLab::AFigureStub* figure, 
    std::string options="LineColor=1,0,0;LineWidth=1")
{
    const int numPoints = 16;
    
    static double cosAngle[numPoints],sinAngle[numPoints];
    
    if(cosAngle[1] == 0.0)
    {
        for(int i = 0; i<numPoints; i++)
        {
            double angle = 2.0*M_PI*i/numPoints;
            cosAngle[i] = cos(angle);
            sinAngle[i] = sin(angle);
        }
    }
    
    double X[numPoints+1], Y[numPoints+1], Z[numPoints+1];
    for(int i = 0; i<numPoints; i++)
    {
        X[i] = x+cosAngle[i]*r;
        Y[i] = y+sinAngle[i]*r;
        Z[i] = z;
    }
    X[numPoints]=X[0];
    Y[numPoints]=Y[0];
    Z[numPoints]=Z[0];
    figure->plot(tag,X,Y,Z,numPoints+1,options);
}

inline void plotCylinder(std::string tag, double x, double y, double z0, double z1, double r, double max_z_diff, DLR_TS::PlotLab::AFigureStub* figure,
    std::string options="LineColor=1,0,0;LineWidth=1")
{
    int i = 0;
    double h = 0;
    do
    {
        // create tag
        std::stringstream ss;
        ss << tag << "/n" << i;

        // calculate z-coordinate of circle, cap at z1
        h = std::min(z0+i*max_z_diff,z1);

        // plot circle
        plotCircle(ss.str(),x,y,h,r,figure,options);
        i++;
    } while (h < z1);    
}

inline void plotRectangle(std::string tag, double x, double y, double l, double w, 
    DLR_TS::PlotLab::AFigureStub* figure, std::string options="LineColor=0,0,0;LineWidth=0;FillColor=1,0,0", double alpha=0.0)
{			
    double pi = 3.141592654;
    double r;
    const int numOfEdges = 4;
    const int numPoints = 13;
    double x_rec[numPoints];
    double y_rec[numPoints];
    double z_rec[numPoints];
    int cc=0;
    double a=w/2.0;
    double b=l/2.0;
    double theta;
    double edges [numOfEdges]={atan2(a,b),atan2(a,-b),atan2(-a,-b)+2*pi   ,atan2(-a,b)+2*pi};
    double points[numPoints] = {atan2(a/2,b),atan2(a,b),atan2(a,b/2),atan2(a,-b/2),	atan2(a,-b),atan2(a/2,-b),atan2(-a/2,-b)+2*pi,atan2(-a,-b)+2*pi,atan2(-a,-b/2)+2*pi,atan2(-a,b/2)+2*pi,atan2(-a,b)+2*pi ,atan2(-a/2,b)+2*pi ,atan2(a,b)};
    
    for (int i=0; i<numPoints;i++)
    {
        theta = points[i];
        if( (theta>=0 && theta<edges[0]) || (theta>=edges[3] && theta<2*pi     ) )   
        {
            r=b/cos(theta);
            x_rec[cc] = r*cos(theta+alpha);
            y_rec[cc] = r*sin(theta+alpha);
            x_rec[cc] = x_rec[cc] + x;
            y_rec[cc] = y_rec[cc] + y;
            z_rec[cc] = 1.0;
            cc=cc+1; 
        }
        else if( theta>= edges[0] && theta < edges[1] )
        {
            r= a/cos(-pi/2+(theta)); 	
            x_rec[cc] = r*cos(theta+alpha);
            y_rec[cc] = r*sin(theta+alpha);
            x_rec[cc] = x_rec[cc] + x;
            y_rec[cc] = y_rec[cc] + y;   
            z_rec[cc] = 1.0;
            cc=cc+1;
        }
        else if  ( theta>= edges[1] && theta < edges[2] )
        {
            r= b/cos(-pi+(theta)); 	
            x_rec[cc] = r*cos(theta+alpha);
            y_rec[cc] = r*sin(theta+alpha);
            x_rec[cc] = x_rec[cc] + x;
            y_rec[cc] = y_rec[cc] + y; 
            z_rec[cc] = 1.0;
            cc=cc+1;
        }
        else if ( theta>= edges[2] && theta < edges[3] )
        {
            r= a/cos(pi/2+(theta)); 	
            x_rec[cc] = r*cos(theta+alpha);
            y_rec[cc] = r*sin(theta+alpha);
            x_rec[cc] = x_rec[cc] + x;
            y_rec[cc] = y_rec[cc] + y;
            z_rec[cc] = 1.0;
            cc=cc+1;
        }
    }
    figure->plot(tag,x_rec,y_rec,z_rec,numPoints,options);
}
inline void unPlotRectangle(std::string tag,DLR_TS::PlotLab::AFigureStub* figure)
{
    figure->erase(tag);

}
inline void plotArrow(std::string hashtag,double x,double y,double z,double alpha,double shaft_length,
    double head_length,std::string options,DLR_TS::PlotLab::AFigureStub* figure)
{
    double pi = 3.141592654;
    double head_angle = pi/4.0;
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);
    double tail_length = head_length*0.66;
    double X[7];
    double Y[7];
    double x1 = x + shaft_length * ca;
    double y1 = y + shaft_length * sa;


    X[0] = x - tail_length * sa;//left tail
    Y[0] = y + tail_length * ca;

    X[1] = x + tail_length * sa;//right tail
    Y[1] = y - tail_length * ca;

    X[2] = x;//base
    Y[2] = y;

    X[3] = x1;//tip
    Y[3] = y1;

    X[4] = x1 + head_length * std::cos(alpha + pi - head_angle);//left
    Y[4] = y1 + head_length * std::sin(alpha + pi - head_angle);

    X[5] = x1;//tip
    Y[5] = y1;

    X[6] = x1 + head_length * std::cos(alpha - pi + head_angle);//right
    Y[6] = y1 + head_length * std::sin(alpha - pi + head_angle);

    figure->plot(hashtag,X,Y,z,7,options);
}

inline void plotLine(std::string hashtag,double x0,double y0,double x1,double y1,double z,std::string options,DLR_TS::PlotLab::AFigureStub* figure)
{
    double X[2];
    double Y[2];

    X[0] = x0;
    X[1] = x1;

    Y[0] = y0;
    Y[1] = y1; 

    figure->plot(hashtag,X,Y,z,2,options);
}
inline void plotArrow(std::string hashtag,double x0,double y0,double z0,double x1,double y1,double shaft_length,double head_length,std::string options,DLR_TS::PlotLab::AFigureStub* figure)
{
    double max_length_ratio = 1.0;
    double max_width_ratio = 0.33;
    double alpha = atan2(y1-y0,x1-x0);
    double length = std::sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
    shaft_length = std::min(shaft_length,length*max_length_ratio);
    head_length = std::min(head_length,shaft_length*max_width_ratio);
    double front = (length-shaft_length)/2.0;
    plotArrow(hashtag,x0+front*std::cos(alpha),y0+front*std::sin(alpha),z0,alpha,shaft_length,head_length,options,figure);
}

inline void plotVectorField(std::string hashtag,double* X,double* Y,double* dX,double* dY,double z0,int size,std::string options,DLR_TS::PlotLab::AFigureStub* figure)
{
    for(int i=0;i<size;i++)
    {
        std::stringstream s;
        s<<hashtag<<"/"<<i;
        double L = std::sqrt(dX[i]*dX[i]+dY[i]*dY[i]);
        plotArrow(s.str(),X[i],Y[i],z0,X[i]+dX[i],Y[i]+dY[i],L,0.33*L,options,figure);
    }
}

const void plotTrajectory(std::string name, const adore::fun::SetPointRequest * const trajectory, std::string options,DLR_TS::PlotLab::AFigureStub* figure, double z = 0.0)
{
    static const size_t n_plot = 100;
    double X[n_plot];
    double Y[n_plot];
    size_t count=0;
    // int items = std::min(n_plot,trajectory->setPoints.size());
    for (auto & point : trajectory->setPoints)
    {
        X[count] = point.x0ref.getX();
        Y[count] = point.x0ref.getY();
        count++;
        if (count > n_plot) break;
    }
    figure->plot(name, X,Y,z,count,options);
}

inline void plotPath(std::string name,const adoreMatrix<double>& data,double z, std::string options,DLR_TS::PlotLab::AFigureStub* figure)
{
    static const int n_plot = 100;
    double X[100];
    double Y[100];
    int batch=0;
    for(int count=0;count<data.nc();count+=n_plot,batch++)
    {
        int items = (std::min)(n_plot,(int)data.nc()-count);
        for(int i=0;i<items;i++)
        {
            X[i] = data(1,count+i);
            Y[i] = data(2,count+i);
        }
        std::stringstream sbuf1;
        sbuf1<<name<<"/"<<batch;
        figure->plot(sbuf1.str(), X,Y,z,items,options);
    }
}
inline void plotData(std::string name,const adoreMatrix<double>& data,double z,int d1,int d2,std::string options,DLR_TS::PlotLab::AFigureStub* figure)
{
    static const int n_plot = 100;
    double X[100];
    double Y[100];
    int batch=0;
    for(int count=0;count<data.nc();count+=n_plot,batch++)
    {
        int items = (std::min)(n_plot,(int)data.nc()-count);
        for(int i=0;i<items;i++)
        {
            X[i] = data(d1,count+i);
            Y[i] = data(d2,count+i);
        }
        std::stringstream sbuf1;
        sbuf1<<name<<"/"<<batch;
        figure->plot(sbuf1.str(), X,Y,z,items,options);
    }
}

} // end PLOT namespace

} // end adore namespace