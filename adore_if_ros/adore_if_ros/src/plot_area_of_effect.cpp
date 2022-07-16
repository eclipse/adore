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
#include <adore_if_ros_msg/AreaOfEffect.h>
#include <ros/ros.h>


class Plotter
{
    private:
    DLR_TS::PlotLab::FigureStubFactory fig_factory_;
    DLR_TS::PlotLab::AFigureStub* figure_;
    std::string ns_;
    ros::Subscriber subscriber_aoe_;
    ros::Subscriber subscriber_aoi_;
    int last_size_aoe_;
    int last_size_aoi_;
    public:
    void receive_aoe(adore_if_ros_msg::AreaOfEffectConstPtr msg)
    {
        last_size_aoe_ = plot(msg,"aoe","1.0,0,0",last_size_aoe_);
    }
    void receive_aoi(adore_if_ros_msg::AreaOfEffectConstPtr msg)
    {
        last_size_aoi_ = plot(msg,"aoi","0.9,0.5,0",last_size_aoi_);
    }
    int plot(adore_if_ros_msg::AreaOfEffectConstPtr msg,std::string tag,std::string color,int last_size)
    {
        static const int N = 100;
        double X[N+1];
        double Y[N+1];
        int count=0;
        int segments = (int)std::ceil((double)msg->X.size()/(double)N);
        for(int i=0;i<segments;i++)
        {
            count = 0;
            for(int j=0;j<N && j+i*N<msg->X.size();j++)
            {
                count++;
                X[j] = msg->X[i*N+j];
                Y[j] = msg->Y[i*N+j];
            }
            X[count] = msg->X[((i+1)%segments)*N];
            Y[count] = msg->Y[((i+1)%segments)*N];
            count++;
            std::stringstream ss;
            ss<<ns_<<"/"<<tag<<"/"<<i;
            figure_->plot(ss.str(),X,Y,1.0,count,"LineWidth=5;LineColor="+color);
        }
        for(int i=segments;i<last_size;i++)
        {
            std::stringstream ss;
            ss<<ns_<<"/"<<tag<<"/"<<i;
            figure_->erase(ss.str());
        }
        return segments;
    }
    Plotter()
    {
        figure_ = fig_factory_.createFigureStub(2);
        ns_ = ros::this_node::getNamespace();
        ros::NodeHandle n;
        subscriber_aoe_ = n.subscribe("ENV/areaofeffect",1,&Plotter::receive_aoe,this);
        subscriber_aoi_ = n.subscribe("ENV/areaofinterest",1,&Plotter::receive_aoi,this);
        last_size_aoe_ = 0;
        last_size_aoi_ = 0;
    }
    ~Plotter()
    {
        delete figure_;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plot_area_of_effect");
    Plotter plotter;
    ros::spin();

    return 0;
}