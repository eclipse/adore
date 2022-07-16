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
 *   Daniel Heß
 ********************************************************************************/

#include <plotlablib/figurestubfactory.h>
#include <adore_if_ros_msg/PlanningResult.h>
#include <ros/ros.h>


class Plotter
{
    private:
    DLR_TS::PlotLab::FigureStubFactory fig_factory_;
    DLR_TS::PlotLab::AFigureStub* figure_;
    std::string ns_;
    ros::Subscriber subscriber_;
    ros::Subscriber subscriber_spr_odom_;
    ros::Subscriber subscriber_spr_utm_;
    std::vector<std::string> styles_;
    bool always_plot_nominal_;
    public:
    void receive(adore_if_ros_msg::PlanningResultConstPtr msg)
    {
        int style_id = msg->id % styles_.size();
        if(msg->combined_maneuver_valid)
        {
            plotSPR(msg->combined_maneuver,ns_+"/presult/"+msg->name+"/c","LineWidth=5;"+styles_[style_id]);
        }
        else
        {
            eraseSPR(ns_+"/presult/"+msg->name+"/c");
        }
        if(msg->nominal_maneuver_valid && (always_plot_nominal_||msg->combined_maneuver_valid))
        {
            plotSPR(msg->nominal_maneuver,ns_+"/presult/"+msg->name+"/n","LineWidth=1;"+styles_[style_id]);
        }
        else
        {
            eraseSPR(ns_+"/presult/"+msg->name+"/n");
        }
    }
    void receive_spr_odom(adore_if_ros_msg::SetPointRequestConstPtr msg)
    {
        adore_if_ros_msg::SetPointRequest spr = *msg;
        plotSPR(spr,"spr_odom","LineColor=1,0,0;LineWidth=3");
    }
    void receive_spr_utm(adore_if_ros_msg::SetPointRequestConstPtr msg)
    {
        adore_if_ros_msg::SetPointRequest spr = *msg;
        plotSPR(spr,"spr_utm","LineColor=0,0,0;LineWidth=1");
    }
    void eraseSPR(std::string tag)
    {
        figure_->erase(tag);
    }
    void plotSPR(const adore_if_ros_msg::SetPointRequest& spr, std::string tag, std::string format)
    {
        static const int N = 100;
        double X[N];
        double Y[N];
        int k = (std::ceil)((double)spr.setPoints.size()/(double)N);
        int count=0;
        for(int i=0;i<N;i++)
        {
            int j=i*k;
            if(j<spr.setPoints.size())
            {
                count = i;
                X[i] = spr.setPoints[j].X;
                Y[i] = spr.setPoints[j].Y;
            }
        }
        figure_->plot(tag,X,Y,1.0,count,format);
    }
    Plotter()
    {
        always_plot_nominal_ = false;
        figure_ = fig_factory_.createFigureStub(2);
        ns_ = ros::this_node::getNamespace();
        ros::NodeHandle n;
        subscriber_ = n.subscribe("FUN/PlanningResult",10,&Plotter::receive,this);
        //subscriber_spr_odom_ = n.subscribe("FUN/SetPointRequest_odom",1,&Plotter::receive_spr_odom,this);
        subscriber_spr_utm_ = n.subscribe("FUN/SetPointRequest",1,&Plotter::receive_spr_utm,this);

        styles_.push_back("LineColor=0,0,1");
        styles_.push_back("LineColor=1,0,0");
        styles_.push_back("LineColor=0,0.75,0");
        styles_.push_back("LineColor=0.5,0.5,0");
        styles_.push_back("LineColor=1,0.7,0");
        styles_.push_back("LineColor=1,0,1");
    }
    ~Plotter()
    {
        delete figure_;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plot_trajectories_node");
    Plotter plotter;
    ros::spin();

    return 0;
}