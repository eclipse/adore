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
#include <geometry_msgs/Point.h>
#include <iostream>
#include <string>


struct Plotter
{
    DLR_TS::PlotLab::FigureStubFactory fig_factory_;
    DLR_TS::PlotLab::AFigureStub* figure_;
    std::string topic_;
    std::string options_;
    ros::Subscriber sub_;
    Plotter(std::string topic,int id)
    {
        ros::NodeHandle n("~");
        topic_ = topic;
        sub_ = n.subscribe(topic,1,&Plotter::receive,this);
        figure_ = fig_factory_.createFigureStub(id);
        figure_->show();
    }
    void receive(geometry_msgs::PointConstPtr point)
    {
        double x = point->x;
        double y = point->y;
        double z = point->z;
        figure_->append(topic_,&x,&y,&z,1,options_);
    }

};

int main(int argc,char **argv)
{
    std::cout<<"plot_scalar_node ---------------------------------------------------------------"<<std::endl;
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
    ros::init(argc,argv,"plot_scalar_node");
    Plotter plotter(topic,figure_number);
    if(argc>3)plotter.options_=argv[3];
    if(argc>4)plotter.figure_->setTitle(argv[4]);
    if(argc>5)plotter.figure_->setXLabel(argv[5]);
    if(argc>6)plotter.figure_->setYLabel(argv[6]);
    ros::spin();
}