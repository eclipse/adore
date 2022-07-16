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
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/


#include <iostream>
#include <thread>
#include <adore/apps/test_lc_trajectory_planner.h>
#include <adore_if_ros/baseapp.h>
#include <plotlablib/figurestubfactory.h>
#include <adore/fun/tac/decoupled_lflc_planner.h>

bool terminated;
DLR_TS::PlotLab::AFigureStub *figure_lon1;
DLR_TS::PlotLab::AFigureStub *figure_lon2;
DLR_TS::PlotLab::AFigureStub *figure_lat1;
DLR_TS::PlotLab::AFigureStub *figure_lat2;
DLR_TS::PlotLab::AFigureStub *figure_psi;
DLR_TS::PlotLab::AFigureStub *figure_localmap;


int currently_plotted_cz = 0;
bool plot_conflictset;

namespace adore
{
namespace if_ROS
{
class TestLCTrajectoryPlannerNode : public Baseapp
{
public:
    adore::apps::TestLCTrajectoryPlanner *lc_;
    TestLCTrajectoryPlannerNode() {}
    void init(int argc, char **argv, double rate, std::string nodename)
    {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        lc_ = new adore::apps::TestLCTrajectoryPlanner(this->getFactory<ENV_Factory>(), this->getFactory<FUN_Factory>(), this->getParamsFactory(""));

        // timer callbacks
        std::function<void()> run_fcn(std::bind(&adore::apps::TestLCTrajectoryPlanner::run, lc_));
        Baseapp::addTimerCallback(run_fcn);
    }
    void performLCleft()
    {
        lc_->performLCLeft();
    }
    void performLCright()
    {
        lc_->performLCRight();
    }
};
} // namespace if_ROS
} // namespace adore

adore::if_ROS::TestLCTrajectoryPlannerNode* plannernode;

void kbinput()
{
    while (!terminated)
    {
        char c = std::cin.get();
        switch (c)
        {
        case 'i':
        {
            std::cout << "print info " << std::endl;
        }
        break;
        case 'l':
        {
            plannernode->performLCleft();
        }
        break;
        case 'r':
        {
            plannernode->performLCright();
        }
        break;
        }
    }
}

//void plot(std::function<adore::fun::DecoupledLFLCPlanner<20,5>*()> getPlanner)//adore::fun::DecoupledLFLCPlanner<20,5>* testplanner_)
void plot(adore::fun::DecoupledLFLCPlanner<20, 5> *testplanner)
{
    //testplanner_->plotBorderOverlapSet(figure_localmap);
    if (testplanner->hasValidPlan())
    {
        static const int N = 100;
        double XN[N];
        double YN[N];
        static const int n = 20;
        double xn[N];
        double yn[N];


        auto &lonsolver = testplanner->getProgressSolver();
        bool lon_valid = lonsolver.isFeasible() && lonsolver.isSolved();

        adore::mad::linspace(lonsolver.result_fun().limitLo(), lonsolver.result_fun().limitHi(), XN, N);
        adore::mad::linspace(lonsolver.result_fun().limitLo(), lonsolver.result_fun().limitHi(), xn, n);

        adore::mad::sample<double, 4>(&lonsolver.result_fun(), XN, YN, 0, (unsigned int)N);
        if (lon_valid)
            figure_lon1->plot("s", XN, YN, N, "LineColor=0,0,1");
        adore::mad::copyRowToArray(lonsolver.y(), yn, 0);
        figure_lon1->plot("sref", xn, yn, n, "LineStyle=none;PointSize=5;LineColor=0,0,1");
        adore::mad::copyRowToArray(lonsolver.ubx(), yn, 0);
        figure_lon1->plot("s_ub", xn, yn, n, "LineColor=0,0,1");
        // adore::mad::copyRowToArray(lonsolver.lbx(),yn,0);
        // figure_lon1->plot("s_lb",xn,yn,n,"LineColor=0,0,1");

        adore::mad::sample<double, 4>(&lonsolver.result_fun(), XN, YN, 1, (unsigned int)N);
        if (lon_valid)
            figure_lon2->plot("ds", XN, YN, N, "LineColor=0,0,1");
        adore::mad::copyRowToArray(lonsolver.y(), yn, 1);
        figure_lon2->plot("dsref", xn, yn, n, "LineColor=0,0,1");
        adore::mad::copyRowToArray(lonsolver.ubx(), yn, 1);
        figure_lon2->plot("ds_ub", xn, yn, n, "LineColor=0,0,1;LineWidth=3");
        adore::mad::copyRowToArray(lonsolver.lbx(), yn, 1);
        figure_lon2->plot("ds_lb", xn, yn, n, "LineColor=0,0,1;LineWidth=3");

        adore::mad::sample<double, 4>(&lonsolver.result_fun(), XN, YN, 2, (unsigned int)N);
        if (lon_valid)
            figure_lon2->plot("dds", XN, YN, N, "LineColor=1,0,0");
        adore::mad::copyRowToArray(lonsolver.ubx(), yn, 2);
        figure_lon2->plot("dds_ub", xn, yn, n, "LineColor=1,0,0;LineWidth=3");
        adore::mad::copyRowToArray(lonsolver.lbx(), yn, 2);
        figure_lon2->plot("dds_lb", xn, yn, n, "LineColor=1,0,0;LineWidth=3");

        adore::mad::sample<double, 4>(&lonsolver.result_fun(), XN, YN, 3, (unsigned int)N);
        if (lon_valid)
            figure_lon2->plot("ddds", XN, YN, N, "LineColor=0,0.5,0");


        auto &latsolver = testplanner->getOffsetSolver();
        bool lat_valid = latsolver.isSolved() && latsolver.isFeasible();

        adore::mad::sample<double, 4>(&latsolver.result_fun(), XN, YN, 0, (unsigned int)N);
        figure_lat1->plot("n", XN, YN, N, "LineColor=0,0,1");
        adore::mad::copyRowToArray(latsolver.y(), yn, 0);
        figure_lat1->plot("nref", xn, yn, n, "LineStyle=none;PointSize=5;LineColor=0,0,1");
        adore::mad::copyRowToArray(latsolver.ubx(), yn, 0);
        figure_lat1->plot("n_ub", xn, yn, n, "LineColor=0,0,1;LineWidth=3");
        adore::mad::copyRowToArray(latsolver.lbx(), yn, 0);
        figure_lat1->plot("n_lb", xn, yn, n, "LineColor=0,0,1;LineWidth=3");

        adore::mad::sample<double, 4>(&latsolver.result_fun(), XN, YN, 1, (unsigned int)N);
        figure_lat2->plot("dn", XN, YN, N, "LineColor=0,0,1");
        adore::mad::copyRowToArray(latsolver.y(), yn, 1);
        figure_lat2->plot("dnref", xn, yn, n, "LineStyle=none;PointSize=5;LineColor=0,0,1");
        adore::mad::copyRowToArray(latsolver.ubx(), yn, 1);
        // figure_lat2->plot("dn_ub",xn,yn,n,"LineColor=0,0,1;LineWidth=3");
        adore::mad::copyRowToArray(latsolver.lbx(), yn, 1);
        // figure_lat2->plot("dn_lb",xn,yn,n,"LineColor=0,0,1;LineWidth=3");

        adore::mad::sample<double, 4>(&latsolver.result_fun(), XN, YN, 2, (unsigned int)N);
        figure_lat2->plot("ddn", XN, YN, N, "LineColor=1,0,0");
        adore::mad::copyRowToArray(latsolver.y(), yn, 2);
        figure_lat2->plot("ddnref", xn, yn, n, "LineStyle=none;PointSize=5;LineColor=1,0,0");
        adore::mad::copyRowToArray(latsolver.ubx(), yn, 2);
        // figure_lat2->plot("ddn_ub",xn,yn,n,"LineColor=1,0,0;LineWidth=3");
        adore::mad::copyRowToArray(latsolver.lbx(), yn, 2);
        // figure_lat2->plot("ddn_lb",xn,yn,n,"LineColor=1,0,0;LineWidth=3");

        adore::mad::sample<double, 4>(&latsolver.result_fun(), XN, YN, 3, (unsigned int)N);
        if (lat_valid)
            figure_lat2->plot("dddn", XN, YN, N, "LineColor=0,0.5,0");
        // adore::mad::copyRowToArray(latsolver.ubu_hard(),yn,0);
        // figure_lat2->plot("dddn_ub",xn,yn,n,"LineColor=0,0.5,0;LineWidth=3");
        // adore::mad::copyRowToArray(latsolver.lbu_hard(),yn,0);
        // figure_lat2->plot("dddn_lb",xn,yn,n,"LineColor=0,0.5,0;LineWidth=3");

        if (lon_valid && lat_valid)
        {
            double PSI[N];
            double DELTA[N];
            double DDELTA[N];
            double OMEGA[N];
            double rS0[N];
            double rS1[N];
            double rS2[N];
            double rN0[N];
            double rN1[N];
            double rN2[N];

            int count;
            auto spr = testplanner->getSetPointRequest();
            for (int i = 0; i < spr->setPoints.size() && i < N; i++)
            {
                auto &sp = spr->setPoints[i];
                XN[i] = sp.tStart - spr->setPoints[0].tStart;
                PSI[i] = sp.x0ref.getPSI();
                OMEGA[i] = sp.x0ref.getOmega();
                DELTA[i] = sp.x0ref.getDelta();
                DDELTA[i] = sp.x0ref.getDDelta();
            }
            figure_psi->plot("psi", XN, PSI, count, "LineColor=0,0,1");
            figure_psi->plot("omega", XN, OMEGA, count, "LineColor=0.7,0,0.7");
            figure_psi->plot("delta", XN, DELTA, count, "LineColor=0,0.5,0");
            figure_psi->plot("ddelta", XN, DDELTA, count, "LineColor=0.7,0,0");

            for (int i = 0; i < spr->setPoints.size() && i < N; i++)
            {
                auto &sp = spr->setPoints[i];
                auto rc = testplanner->getRoadCoordinateConverter().toRoadCoordinates(sp.x0ref);
                XN[i] = sp.tStart - spr->setPoints[0].tStart;
                rS0[i] = rc.s0;
                rS1[i] = rc.s1;
                rS2[i] = rc.s2;
                rN0[i] = rc.n0;
                rN1[i] = rc.n1;
                rN2[i] = rc.n2;
                count = i;
            }
            figure_lat1->plot("n_test", XN, rN0, count, "LineColor=0,0,1");
            figure_lat2->plot("ds_test", XN, rN1, count, "LineColor=0,0,1");
            figure_lat2->plot("dds_test", XN, rN2, count, "LineColor=1,0,0");
            figure_lon1->plot("s_test", XN, rS0, count, "LineColor=0,0,1");
            figure_lon2->plot("ds_test", XN, rS1, count, "LineColor=0,0,1");
        }
    }
}

void initFigures()
{
    DLR_TS::PlotLab::FigureStubFactory fig_factory;

    figure_lon1 = fig_factory.createFigureStub(3);
    figure_lon1->setTitle("Longitudinal Plan - s");
    figure_lon1->setXLabel("t (s)");
    figure_lon1->setYLabel("s (m)");
    figure_lon1->showAxis();
    figure_lon1->showGrid();
    figure_lon1->show();
    figure_lon2 = fig_factory.createFigureStub(4);
    figure_lon2->setTitle("Longitudinal Plan - ds(b),dds(g),ddds(r)");
    figure_lon2->setXLabel("t (s)");
    figure_lon2->setYLabel("ds (m/s)");
    figure_lon2->showAxis();
    figure_lon2->showGrid();
    figure_lon2->show();
    figure_lat1 = fig_factory.createFigureStub(5);
    figure_lat1->setTitle("Lateral Plan - n");
    figure_lat1->setXLabel("t (s)");
    figure_lat1->setYLabel("n (m)");
    figure_lat1->show();
    figure_lat1->showAxis();
    figure_lat1->showGrid();
    figure_lat2 = fig_factory.createFigureStub(6);
    figure_lat2->setTitle("Lateral Plan - dn(b),ddn(r),dddn(g)");
    figure_lat2->setXLabel("t (s)");
    figure_lat2->setYLabel("dn (m/s)");
    figure_lat2->show();
    figure_lat2->showAxis();
    figure_lat2->showGrid();
    figure_psi = fig_factory.createFigureStub(7);
    figure_psi->setTitle("state trajectory - psi(b), omega(m), delta(g), ddelta(r)");
    figure_psi->setXLabel("t (s)");
    figure_psi->setYLabel("");
    figure_psi->show();
    figure_psi->showAxis();
    figure_psi->showGrid();
}

int main(int argc, char **argv)
{
    terminated = false;
    std::thread kbinput_thread(kbinput);
    adore::if_ROS::TestLCTrajectoryPlannerNode lcn;
    lcn.init(argc, argv, 10.0, "test_lc_trajectory_planner");

    // plot function
    bool render = true;
    lcn.getParam("/render", render);
    if (render)
    {
        initFigures();
        std::function<adore::fun::DecoupledLFLCPlanner<20, 5> *()> getPlanner = std::bind(&adore::apps::TestLCTrajectoryPlanner::getPlanner, lcn.lc_);
        std::function<void()> plot_fcn(std::bind(&plot, getPlanner()));
        lcn.addTimerCallback(plot_fcn);
    }
    plannernode = &lcn;

    lcn.run();
    terminated = true;
    kbinput_thread.join();
    return 0;
}
