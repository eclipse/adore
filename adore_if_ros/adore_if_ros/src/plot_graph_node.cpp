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
 *   Daniel Heß - Plotlab visualization of values on arbitrary ROS topics
 ********************************************************************************/
#include <plotlablib/figurestubfactory.h>
#include <ros/ros.h>
#include <adore_if_ros_msg/PlotStart.h>
#include <unordered_map>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>

class Handler
{
    private:
        DLR_TS::PlotLab::FigureStubFactory fig_factory_;
        DLR_TS::PlotLab::AFigureStub* figure_;
        double x_plot_;
        double y_plot_;
        double x_;
        double y_;
        bool x_valid_;
        bool y_valid_;
        ros::Subscriber x_subscriber_;
        ros::Subscriber y_subscriber_;
        std::string tag_;
        std::string options_;
    public:
        void plot()
        {
            if(x_valid_ && y_valid_)
            {
                figure_->append(tag_,&x_,&y_,1,options_);
                x_plot_ = x_;
                y_plot_ = y_;
            }
        }
        void receive_x(std_msgs::Float64ConstPtr msg)
        {
            x_ = msg->data;
            x_valid_ = true;
        }
        void receive_y(std_msgs::Float64ConstPtr msg)
        {
            y_ = msg->data;
            y_valid_ = true;
            if(x_!=x_plot_)plot();
        }
        Handler(ros::NodeHandle* n,adore_if_ros_msg::PlotStartConstPtr msg)
        {
            figure_ = fig_factory_.createFigureStub(msg->figure);
            figure_->show();
            x_valid_ = false;
            y_valid_ = false;
            x_plot_ = 0.0;
            y_plot_ = 0.0;
            tag_ = msg->tag;
            options_ = msg->options;
            x_subscriber_ = n->subscribe(msg->xtopic,1,&Handler::receive_x,this);
            y_subscriber_ = n->subscribe(msg->ytopic,1,&Handler::receive_y,this);
        }
};

class Dispatcher
{
    private:
        ros::Subscriber subscriber_;
        ros::NodeHandle* n_;
        std::unordered_map<std::string,Handler*> handlers_;
    public:
    void receive(adore_if_ros_msg::PlotStartConstPtr msg)
    {
        if(handlers_.find(msg->tag)==handlers_.end())
        {
            delete handlers_[msg->tag];
        }
        handlers_[msg->tag] = new Handler(n_,msg);
    }
    Dispatcher(ros::NodeHandle* n)
    {
        n_=n;
        subscriber_ = n->subscribe("plot",1,&Dispatcher::receive,this);
    }
};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"plot_graph_node");
    ros::NodeHandle n;
    Dispatcher dispatcher(&n);
    ros::spin();
}
