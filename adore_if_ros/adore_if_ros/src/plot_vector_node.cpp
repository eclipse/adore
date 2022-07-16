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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/
#include <plotlablib/figurestubfactory.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <adore_if_ros_msg/PointArray.h>


struct Plotter
{
    DLR_TS::PlotLab::FigureStubFactory fig_factory_;
    DLR_TS::PlotLab::AFigureStub* figure_;
    std::string topic_;
    std::string options_;
    ros::Subscriber sub_;
    int last_k_;
    Plotter(std::string topic,int id)
    {
        ros::NodeHandle n("~");
        sub_ = n.subscribe(topic,1,&Plotter::receive,this);
        figure_ = fig_factory_.createFigureStub(id);
        figure_->show();
        topic_ = topic;
        last_k_ = 0;
    }
    void receive(adore_if_ros_msg::PointArrayConstPtr pointArray)
    {
        static const int N = 128;
        double x[N+1];
        double y[N+1];
        double z[N+1];
        const int n = std::max((int)std::max((int)pointArray->x.size(),(int)pointArray->y.size()),(int)pointArray->z.size());
        const int k = std::ceil((double)n/(double)N);
        if(n==0)
        {
            std::cout<<"nothing to plot...";
            return;
        }
        for(int j=0;j<k;j++)
        {
            int g = std::min(N,n-j*N) + (j==0?0:1);
            for(int i=0;i<g;i++)
            {
                int h = N*j+i - (j==0?0:1);
                x[i] = pointArray->x.size()>0?pointArray->x[std::min(h,(int)std::min(n,(int)pointArray->x.size())-1)]:0.0;
                y[i] = pointArray->y.size()>0?pointArray->y[std::min(h,(int)std::min(n,(int)pointArray->y.size())-1)]:0.0;
                z[i] = pointArray->z.size()>0?pointArray->z[std::min(h,(int)std::min(n,(int)pointArray->z.size())-1)]:0.0;
            }
            std::stringstream ss;
            ss<<topic_<<"/"<<j;
            figure_->plot(ss.str(),x,y,z,g,options_);
        }
        for(int j=k;j<last_k_;j++)
        {
            std::stringstream ss;
            ss<<topic_<<"/"<<j;
            figure_->erase(ss.str());
        }
        last_k_=k;
    }

};

int main(int argc,char **argv)
{
    std::cout<<"plot_vector_node ---------------------------------------------------------------"<<std::endl;
    std::cout<<"arguments (first 2 reuqired): <topic:string> <figure-id:int0-9> <options:string>"<<std::endl;
    std::cout<<"                              <title:string> <xlabel:string> <ylabel:string>"<<std::endl;
    if(argc<3)std::cout<<"Wrong number of arguments, min 2!"<<std::endl;
    std::cout<<"--------------------------------------------------------------------------------"<<std::endl;
    if(argc<3)return 1;
    std::string topic(argv[1]);
    int figure_number = 0;
    try
    {
        figure_number = std::atoi(argv[2]);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout<<"second argument must be a number"<<std::endl;
        return 1;
    }
    ros::init(argc,argv,"plot_vector_node");
    Plotter plotter(topic,figure_number);
    if(argc>3)plotter.options_=argv[3];
    if(argc>4){plotter.figure_->setTitle(argv[4]);plotter.figure_->showAxis();}
    if(argc>5)plotter.figure_->setXLabel(argv[5]);
    if(argc>6)plotter.figure_->setYLabel(argv[6]);
    ros::spin();
}