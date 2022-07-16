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
#include <adore_if_ros/funfactory.h>
#include <adore_if_ros/paramsfactory.h>


class Dashboard
{
    private:
    DLR_TS::PlotLab::AFigureStub* figure_xy_;
    DLR_TS::PlotLab::AFigureStub* figure_ex_;
    DLR_TS::PlotLab::AFigureStub* figure_ey_;
    DLR_TS::PlotLab::AFigureStub* figure_delta_;
    DLR_TS::PlotLab::AFigureStub* figure_sprx_;
    DLR_TS::PlotLab::AFigureStub* figure_odom_;
    DLR_TS::PlotLab::AFigureStub* figure_psi_;
    adore::fun::SetPointRequest spr_;
    ros::Subscriber subscriber_spr_;
    ros::Subscriber subscriber_odom_;
    ros::Subscriber subscriber_acceleration_;
    ros::Subscriber subscriber_steering_;
    ros::Subscriber subscriber_deltafb_;
    ros::Subscriber subscriber_axfb_;
    adore::fun::VehicleMotionState9d x_;
    adore::if_ROS::PARAMS_Factory* pfac_;
    adore::params::APVehicle* pveh_;
    double t_;
    double t0_spr_;
    double t0_plot_;
    double deltafb_;
    double axfb_;
    bool t0_plot_set_;


    void plot_spr()
    {
        if(spr_.setPoints.size()==0)return;
        static const int N=200;
        double X[N];
        double Y[N];
        int i;
        int k = std::max(1,(int)std::ceil(((double)spr_.setPoints.size())/(double)N));
        double t0 = spr_.setPoints.front().tStart;
        t0_spr_ = t0;
        std::stringstream ss;
        ss<<"t (s), t0="<<t0;
        figure_sprx_->setXLabel(ss.str());


        for(i=0;i<N && i*k<spr_.setPoints.size();i++)
        {
            auto& sp = spr_.setPoints[i*k];
            X[i] = sp.tStart-t0;
            Y[i] = sp.x0ref.getvx();
        }
        figure_sprx_->plot("vx",X,Y,i,"LineColor=0,0.5,0;LineWidth=2.0");

        for(i=0;i<N && i*k<spr_.setPoints.size();i++)
        {
            auto& sp = spr_.setPoints[i*k];
            X[i] = sp.tStart-t0;
            Y[i] = sp.x0ref.getAx();
        }
        figure_sprx_->plot("ax",X,Y,i,"LineColor=1,0,0;LineWidth=2.0");
    }

    void plot_x()
    {
        double t = x_.getTime();
        if(t!=t_ && spr_.setPoints.size()>0 && spr_.isActive(t))
        {
            t_=t;
            adore::fun::PlanarVehicleState10d xs = spr_.interpolateReference(t,pveh_);
            double dX = x_.getX()-xs.getX();
            double dY = x_.getY()-xs.getY();
            double cpsi = std::cos(xs.getPSI());
            double spsi = std::sin(xs.getPSI());
            double psi = x_.getPSI();
            double psis = xs.getPSI();
            double dx = cpsi * dX + spsi * dY;
            double dy =-spsi * dX + cpsi * dY;
            double dxpsi = cpsi * std::cos(x_.getPSI()) + spsi * std::sin(x_.getPSI());
            double dypsi =-spsi * std::cos(x_.getPSI()) + cpsi * std::sin(x_.getPSI());
            double evx = x_.getvx()-xs.getvx();
            double vx = x_.getvx();
            double vy = x_.getvy();
            double omega = x_.getOmega();
            double ax = x_.getAx();
            double axs = xs.getAx();
            double epsi = std::atan2(dypsi,dxpsi) * 180.0 / M_PI;
            double eomega = x_.getOmega()-xs.getOmega();
            double delta = x_.getDelta() / pveh_->get_steeringRatio() * 180.0/M_PI;
            double deltas = xs.getDelta() * 180.0/M_PI;
            double deltafb = deltafb_ / pveh_->get_steeringRatio() * 180.0/M_PI;
            double tplot = t - t0_plot_;
            figure_odom_->append("vx",&tplot,&vx,1,"LineColor=0,0,1;LineWidth=2");
            figure_odom_->append("vy",&tplot,&vy,1,"LineColor=0,0.7,0;LineWidth=2");
            figure_odom_->append("omega",&tplot,&omega,1,"LineColor=1,0,1;LineWidth=2");
            figure_odom_->append("ax",&tplot,&ax,1,"LineColor=1,0,0;LineWidth=2");
            figure_ex_->append("ex",&tplot,&dx,1,"LineColor=0,0,1;LineWidth=2.0");
            figure_ex_->append("evx",&tplot,&evx,1,"LineColor=0,0.5,0;LineWidth=2.0");
            figure_ex_->append("ax",&tplot,&ax,1,"LineColor=1,0,0;LineWidth=2.0");
            figure_ex_->append("axs",&tplot,&axs,1,"LineColor=1,0,0;LineWidth=1.0");
            figure_ex_->append("axfb",&tplot,&axfb_,1,"LineColor=1,0,1;LineWidth=1.0");
            figure_ey_->append("ey",&tplot,&dy,1,"LineColor=0,0,1;LineWidth=2.0");
            figure_ey_->append("epsi",&tplot,&epsi,1,"LineColor=0,0.5,0;LineWidth=2.0");
            figure_ey_->append("eomega",&tplot,&eomega,1,"LineColor=1,0,0;LineWidth=2.0");
            figure_delta_->append("delta",&tplot,&delta,1,"LineColor=0,0,1;LineWidth=2.0");
            figure_delta_->append("deltaff",&tplot,&deltas,1,"LineColor=1,0,0;LineWidth=2.0");
            figure_delta_->append("deltafb",&tplot,&deltafb,1,"LineColor=0,0.5,0;LineWidth=2.0");
            figure_psi_->append("psi",&tplot,&psi,1,"LineColor=0,0,1;LineWidth=2.0");
            figure_psi_->append("psis",&tplot,&psis,1,"LineColor=1,0,0;LineWidth=2.0");
            double X[2];X[0] = t-t0_spr_;X[1] = t-t0_spr_;
            double Y[2];Y[0] = 0.0;Y[1] = 0.0;
            figure_sprx_->plot("dtx",X,Y,2,"LineStyle=none;MarkerSize=5");

            //plot the vehicle
            const double L = pveh_->get_a()+pveh_->get_b();
            const double c = pveh_->get_c();
            const double d = pveh_->get_d();
            const double w = std::max(pveh_->get_wf(),pveh_->get_wr())*0.5;
            plotPosition("setPoint",xs.getX(),xs.getY(),xs.getPSI(),L,c,d,w,"LineColor=1,0,0");

        }
    }
      /**
       * @brief plotting a vehicle
       * 
       * @param name a tag used to id the vehicle
       * @param gX x position
       * @param gY y position
       * @param psi heading
       * @param L 
       * @param c 
       * @param d 
       * @param w 
       * @param options drawing options, cf. plotlablib
       */
      void plotPosition(const std::string& name,double gX,double gY,double psi,double L,double c,double d,double w,const std::string& options)
      {
        double X[12];
        double Y[12];
        X[0] = 0.0; X[1] = L; X[2] = L; X[3] = L + c; X[4] = L; X[5] = L + c; X[6] = L + c; X[7] = 0.0; X[8] = 0.0; X[9] = -d; X[10] = -d; X[11] = 0.0;
        Y[0] = -w; Y[1] = -w; Y[2] = +w; Y[3] = 0.0; Y[4] = -w; Y[5] = -w; Y[6] = +w; Y[7] = +w; Y[8] = -w; Y[9] = -w; Y[10] = +w; Y[11] = +w;

        double cpsi = std::cos(psi);
        double spsi = std::sin(psi);
        for(int i=0;i<12;i++)
        {
          double x = gX + cpsi * X[i] - spsi * Y[i];
          double y = gY + spsi * X[i] + cpsi * Y[i];
          X[i] = x;
          Y[i] = y;
        }

        figure_xy_->plot(name,X,Y,0.25,12,options);
      }

    public:
    void receive_accelerationRequest(std_msgs::Float32ConstPtr msg)
    {
        axfb_ = msg->data;
    }                        
    void receive_steeringRequest(std_msgs::Float32ConstPtr msg)
    {
        deltafb_ = msg->data;
    }                        
    void receive_spr(adore_if_ros_msg::SetPointRequestConstPtr msg)
    {
        adore::if_ROS::SetPointRequestConverter converter;
        converter(msg,&spr_);
        plot_spr();
    }

    void receive_odom(nav_msgs::OdometryConstPtr msg)
    {
        if(!t0_plot_set_)
        {
            t0_plot_ = msg->header.stamp.toSec();
            t0_plot_set_ = true;
        }
        x_.setTime(msg->header.stamp.toSec());
        x_.setX(msg->pose.pose.position.x);
        x_.setY(msg->pose.pose.position.y);
        x_.setZ(msg->pose.pose.position.z);
        adore::if_ROS::QuaternionConverter qc;
        x_.setPSI(qc.quaternionToHeading(msg->pose.pose));
        x_.setvx(msg->twist.twist.linear.x);
        x_.setvy(msg->twist.twist.linear.y);
        x_.setOmega(msg->twist.twist.angular.z);
        plot_x();
    }
    void receive_steering(std_msgs::Float32ConstPtr msg)
    {
        x_.setDelta(msg->data);
    }
    void receive_acceleration(std_msgs::Float32ConstPtr msg)
    {
        x_.setAx(msg->data);
    }

    Dashboard()
    :t_(0.0),t0_spr_(0.0),t0_plot_set_(false),t0_plot_(0.0),deltafb_(0.0),axfb_(0.0)
    {
        DLR_TS::PlotLab::FigureStubFactory fig_factory;

        figure_xy_ = fig_factory.createFigureStub(2);

        figure_odom_ = fig_factory.createFigureStub(3);
        figure_odom_->setTitle("Ego-State - vx (blue), vy (green), omega[rad/s] (pink), ax (red)");
        figure_odom_->setXLabel("t (s)");
        figure_odom_->setYLabel("");
        figure_odom_->showAxis();
        figure_odom_->showGrid();
        figure_odom_->show();        

        figure_sprx_ = fig_factory.createFigureStub(4);
        figure_sprx_->setTitle("SetPointRequest - vx*(green), ax*(red), t-t0(black)");
        figure_sprx_->setXLabel("t (s)");
        figure_sprx_->setYLabel("");
        figure_sprx_->showAxis();
        figure_sprx_->showGrid();
        figure_sprx_->show();        


        figure_ex_ = fig_factory.createFigureStub(5);
        figure_ex_->setTitle("longitudinal tracking error - ex (blue); vx-vx*(green); ax(red), axff(thin red), axfb(pink)");
        figure_ex_->setXLabel("t (s)");
        figure_ex_->setYLabel("e");
        figure_ex_->showAxis();
        figure_ex_->showGrid();
        figure_ex_->show();        

        figure_ey_ = fig_factory.createFigureStub(6);
        figure_ey_->setTitle("lateral tracking error - ey [m/s](blue), psi-psi* [deg](green), omega-omega*[rad/s](red)");
        figure_ey_->setXLabel("t (s)");
        figure_ey_->setYLabel("e");
        figure_ey_->showAxis();
        figure_ey_->showGrid();
        figure_ey_->show();        

        figure_delta_ = fig_factory.createFigureStub(7);
        figure_delta_->setTitle("steering angle - delta[deg] (blue), deltaff (red), deltafb (green)");
        figure_delta_->setXLabel("t (s)");
        figure_delta_->setYLabel("delta");
        figure_delta_->showAxis();
        figure_delta_->showGrid();
        figure_delta_->show();        

        figure_psi_ = fig_factory.createFigureStub(8);
        figure_psi_->setTitle("yaw angle - psi[deg] (blue), psi* (red)");
        figure_psi_->setXLabel("t (s)");
        figure_psi_->setYLabel("psi");
        figure_psi_->showAxis();
        figure_psi_->showGrid();
        figure_psi_->show();        



        ros::NodeHandle n;    
        subscriber_spr_ = n.subscribe("FUN/SetPointRequest",1,&Dashboard::receive_spr,this);
        subscriber_odom_ = n.subscribe("odom",1,&Dashboard::receive_odom,this);
        subscriber_acceleration_ = n.subscribe("VEH/ax",1,&Dashboard::receive_acceleration,this);
        subscriber_steering_ = n.subscribe("VEH/steering_angle_measured",1,&Dashboard::receive_steering,this);
        subscriber_deltafb_ = n.subscribe("FUN/MotionCommand/acceleration",1,&Dashboard::receive_accelerationRequest,this);
        subscriber_axfb_ = n.subscribe("FUN/MotionCommand/steeringAngle",1,&Dashboard::receive_steeringRequest,this);

        pfac_ = new adore::if_ROS::PARAMS_Factory(n,"");
        pveh_ = pfac_->getVehicle();
    }
};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"test_control_dashboard");
    Dashboard dashboard;
    ros::spin();
}