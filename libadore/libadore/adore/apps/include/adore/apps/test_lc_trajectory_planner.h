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
 *   Matthias Nichting - initial implementation
 ********************************************************************************/

#pragma once

#include <adore/view/alanechangeview.h>
#include <adore/fun/afactory.h>
#include <adore/fun/tac/basiclanefollowingplanner.h>
#include <adore/fun/tac/basiclanechangeplanner.h>
#include <adore/fun/tac/decoupled_lflc_planner.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/env/borderbased/lanefollowingview.h>
#include <adore/env/borderbased/lanechangeview.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/traffic/trafficmap.h>
#include <adore/apps/if_plotlab/plot_border.h>
#include <iostream>

namespace adore
{
namespace apps
{
/**
     * @brief test implementation of a lane change trajectory planner
     * 
     */
class TestLCTrajectoryPlanner
{
private:
  typedef adore::fun::BasicLaneFollowingPlanner<20, 5> TPlanner;
  adore::env::AFactory *envFactory_;
  adore::fun::AFactory *funFactory_;
  adore::params::AFactory *paramsFactory_;
  adore::params::APTacticalPlanner *pTacticalPlanner_;
  adore::env::BorderBased::LocalRoadMap roadmap_;
  adore::env::BorderBased::LaneFollowingView lfv_;
  adore::env::BorderBased::LaneChangeView lcvr_;
  adore::env::BorderBased::LaneChangeView lcvl_;
  adore::mad::AReader<adore::fun::VehicleMotionState9d> *xreader_;
  adore::mad::AWriter<adore::fun::SetPointRequest> *wwriter_;
  adore::env::traffic::TrafficMap trafficMap_;
  adore::fun::VehicleMotionState9d x_;
  TPlanner *lfplanner_;
  adore::fun::BasicLaneChangePlanner<20, 5> *lcplanner_l_;
  adore::fun::BasicLaneChangePlanner<20, 5> *lcplanner_r_;
  adore::fun::DecoupledLFLCPlanner<20, 5> *active_planner_;
  adore::params::APVehicle *pvehicle_;
  double last_n_;
  bool perform_lc_left_, perform_lc_right_;

public:
  TestLCTrajectoryPlanner()
      : envFactory_(adore::env::EnvFactoryInstance::get()), funFactory_(adore::fun::FunFactoryInstance::get()), paramsFactory_(adore::params::ParamsFactoryInstance::get()),
        roadmap_(envFactory_, paramsFactory_), trafficMap_(roadmap_.getBorderSet(), envFactory_),
        lfv_(paramsFactory_, &roadmap_, &trafficMap_),
        lcvr_(paramsFactory_, &roadmap_, &lfv_, &trafficMap_),
        lcvl_(paramsFactory_, &roadmap_, &lfv_, &trafficMap_), last_n_(0.0)
  {

    pTacticalPlanner_ = paramsFactory_->getTacticalPlanner();
    lfplanner_ = new TPlanner(&lfv_,nullptr,nullptr,nullptr,
                              paramsFactory_->getLongitudinalPlanner(),
                              paramsFactory_->getLateralPlanner(),
                              paramsFactory_->getTacticalPlanner(),
                              paramsFactory_->getVehicle(),
                              paramsFactory_->getTrajectoryGeneration());
    lcplanner_l_ = new adore::fun::BasicLaneChangePlanner<20, 5>(&lcvl_, nullptr,nullptr,nullptr,nullptr,nullptr, 
                                                                 paramsFactory_->getLongitudinalPlanner(),
                                                                 paramsFactory_->getLateralPlanner(),
                                                                 paramsFactory_->getTacticalPlanner(),
                                                                 paramsFactory_->getVehicle(),
                                                                 paramsFactory_->getTrajectoryGeneration());
    lcplanner_r_ = new adore::fun::BasicLaneChangePlanner<20, 5>(&lcvr_, nullptr,nullptr,nullptr,nullptr,nullptr,
                                                                 paramsFactory_->getLongitudinalPlanner(),
                                                                 paramsFactory_->getLateralPlanner(),
                                                                 paramsFactory_->getTacticalPlanner(),
                                                                 paramsFactory_->getVehicle(),
                                                                 paramsFactory_->getTrajectoryGeneration());
    xreader_ = funFactory_->getVehicleMotionStateReader();
    wwriter_ = funFactory_->getSetPointRequestWriter();
    pvehicle_ = paramsFactory_->getVehicle();
    active_planner_ = lfplanner_;
  }
  void run()
  {
    roadmap_.update();
    trafficMap_.update();
    lfv_.update();
    lcvl_.update(adore::view::ALaneChangeView::LEFT);
    lcvr_.update(adore::view::ALaneChangeView::RIGHT);
    xreader_->getData(x_);
    auto x_replan = x_;
    bool reset = true;
    if (active_planner_->hasValidPlan())
    {
      auto spr = active_planner_->getSetPointRequest();
      double t = x_.getTime();
      if (spr->isActive(t))
      {
        auto x_ref = spr->interpolateReference(t, pvehicle_);
        double dx = x_.getX() - x_ref.getX();
        double dy = x_.getY() - x_ref.getY();
        if (dx * dx + dy * dy < pTacticalPlanner_->getResetRadius())
        {
          x_replan.setX(x_ref.getX());
          x_replan.setY(x_ref.getY());
          x_replan.setPSI(x_ref.getPSI());
          x_replan.setvx(x_ref.getvx());
          x_replan.setvy(x_ref.getvy());
          x_replan.setOmega(x_ref.getOmega());
          x_replan.setAx(x_ref.getAx());
          x_replan.setDelta(x_ref.getDelta());
          reset = false;
        }
      }
    }
    if (reset)
      std::cout << "TestTrajectoryPlanner: Reset initial state.\n";
    double s, n;
    lfv_.toRelativeCoordinates(x_replan.getX(), x_replan.getY(), s, n);
    if (!perform_lc_left_ && !perform_lc_right_)
    {
      auto olt = lfv_.getOnLaneTraffic();
      for (auto &obj : olt)
      {
        if (obj.getCurrentProgress() > s && obj.getCurrentProgress() < s + 30.0 && obj.getCurrentSpeed() < 0.8 * lfv_.getSpeedLimit(s) && lcvl_.getTargetLane()->isValid())
        {
          perform_lc_left_ = true;
          break;
        }
      }
      if (!perform_lc_left_ && lcvr_.getTargetLane()->isValid())
      {
        auto oltr = lcvr_.getTargetLane()->getOnLaneTraffic();
        perform_lc_right_ = true;
        for (auto &obj : oltr)
        {
          if (obj.getCurrentProgress() < s+35 && obj.getCurrentProgress() > s-2 && obj.getCurrentSpeed() < 0.95 * lfv_.getSpeedLimit(s))
          {
            perform_lc_right_ = false;
          }
        }
      }
    }

    if (last_n_ > 0.5 && n < 0.0 || last_n_ < -0.5 && n > 0.0) // lane change complete
    {
      perform_lc_left_ = false;
      perform_lc_right_ = false;
    }
    last_n_ = n;
    lfplanner_->compute(x_replan);
    if (lcvr_.getTargetLane()->isValid() && perform_lc_right_)
    {
      if (active_planner_ != lcplanner_r_)
      {
        // lcplanner_r_->setGapCoordinate(s);
      }
      lcplanner_r_->compute(x_replan);
    }
    if (lcvl_.getTargetLane()->isValid() && perform_lc_left_)
    {
      if (active_planner_ != lcplanner_l_)
      {
        // lcplanner_l_->setGapCoordinate(s);
      }
      lcplanner_l_->compute(x_replan);
    }
    if (lcvl_.getTargetLane()->isValid() && perform_lc_left_ && lcplanner_l_->hasValidPlan())
    {
      active_planner_ = lcplanner_l_;
    }
    else if (lcvr_.getTargetLane()->isValid() && perform_lc_right_ && lcplanner_r_->hasValidPlan())
    {
      active_planner_ = lcplanner_r_;
    }
    else
    {
      perform_lc_right_ = false;
      perform_lc_left_ = false;
      active_planner_ = lfplanner_;
    }
    if (active_planner_->hasValidPlan())
      wwriter_->write(*active_planner_->getSetPointRequest());
  }
  bool hasValidPlan()
  {
    return active_planner_->hasValidPlan();
  }
  void performLCLeft()
  {
    perform_lc_left_ = true;
    perform_lc_right_ = false;
  }
  void performLCRight()
  {
    perform_lc_left_ = false;
    perform_lc_right_ = true;
  }
  adore::fun::DecoupledLFLCPlanner<20, 5>::TProgressSolver &getProgressSolver()
  {
    return active_planner_->getProgressSolver();
  }
  adore::fun::DecoupledLFLCPlanner<20, 5>::TOffsetSolver &getOffsetSolver()
  {
    return active_planner_->getOffsetSolver();
  }
  adore::fun::DecoupledLFLCPlanner<20, 5> * getPlanner()
  {
    return active_planner_;
  }
  const adore::fun::SetPointRequest *getSetPointRequest()
  {
    return active_planner_->getSetPointRequest();
  }
  adore::fun::RoadCoordinateConverter &getRoadCoordinateConverter()
  {
    return active_planner_->getRoadCoordinateConverter();
  }

  void plotConflictSet(DLR_TS::PlotLab::AFigureStub *fig)
  {
    auto cornerpoints = lfv_.getCornerPoints();
    std::stringstream ss;
    std::string options_point = "LineColor=1.0,0.0,0.0;LineWidth=0;LineStyle=none;PointSize=5";
    int c = 0;
    //{
    // lfv_.toEucledianCoordinates(i->first,i->second,x[c],y[c],z[c]);
    //++c;
    //}
    //fig->plot(ss.str(),x,y,1.0,c,options_point);
    std::cout << "cornerpoitns size" << cornerpoints->size() << std::endl;
    double xc[cornerpoints->size()];
    double yc[cornerpoints->size()];
    double zc[cornerpoints->size()];
    c = 0;
    for (auto i = cornerpoints->begin(); i != cornerpoints->end(); ++i)
    {
      xc[c] = i->m_X;
      yc[c] = i->m_Y;
      ++c;
    }
    options_point = "LineColor=1.0,0.0,1.0;LineWidth=0;LineStyle=none;PointSize=6";
    fig->plot("cornerpoints", xc, yc, 2.0, c, options_point);
    std::cout << "now:conflictset" << std::endl;
    auto plotdata = lfv_.getConflictSetPlotData();
    std::string options;
    std::string options_normal = "LineColor=0.0,1.0,0.0;LineWidth=3";
    std::string options_highlighted = "LineColor=1.0,1.0,0.0;LineWidth=6";
    std::string cz_str = "cz_subgroup";
    std::string bo_str = "bo_number";
    int bo_int;
    int cz_int = 0;
    std::cout << "\n plot evaluated borders";
    for (auto it = plotdata->begin(); it != plotdata->end(); ++it)
    {
      bo_int = 0;
      std::cout << "number of cz:" << plotdata->size() << std::endl;
      for (auto ij = it->second.begin(); ij != it->second.end(); ++ij)
      {
        std::cout << "   path of this cz" << std::endl;
        options = (bo_int == 0 ? options_highlighted : options_normal);
        PLOT::plotPath(cz_str + std::to_string(cz_int) + bo_str + std::to_string(bo_int), roadmap_.getBorderSet()->getCenterline((*ij)->m_id).getData(), 0.18, options, fig);
        ++bo_int;
      }
      ++cz_int;
    }
  }
  void plotBorder(std::string name, adore::env::BorderBased::Border *right, adore::env::BorderBased::Border *left, double z, std::string options, DLR_TS::PlotLab::AFigureStub *figure_)
  {
    static const int nplot = 100;
    double X[nplot];
    double Y[nplot];
    double Xn[nplot];
    double Yn[nplot];
    int n = right->m_path->getData().nc();
    right->m_path->writePointsToArray(0, n - 1, 0, X);
    right->m_path->writePointsToArray(0, n - 1, 1, Y);
    if (left != 0)
    {
      int m = left->m_path->getData().nc();
      left->m_path->writePointsToArray(0, m - 1, 0, Xn);
      left->m_path->writePointsToArray(0, m - 1, 1, Yn);
      if (right->getNeighborDirection() == adore::env::BorderBased::Border::SAME_DIRECTION)
      {
        for (int i = 0; i < m; i++)
        {
          X[n + m - i - 1] = Xn[i];
          Y[n + m - i - 1] = Yn[i];
        }
      }
      else
      {
        for (int i = 0; i < m; i++)
        {
          X[n + i] = Xn[i];
          Y[n + i] = Yn[i];
        }
      }
      figure_->patch(name + "/h", X, Y, z, n + m, options);
    }
    figure_->plot(name, X, Y, z + 0.1, n, options);
    X[1] = X[n - 1];
    Y[1] = Y[n - 1];
    //figure_->plot(name+"/p",X,Y,z+0.1,2,"LineStyle=none;PointSize=5");
  }

  void plotBorderOverlapSet(DLR_TS::PlotLab::AFigureStub *fig)
  {
    std::cout << "plot border overlaps" << std::endl;
    auto overlapsets = lfv_.getOverlapSet();
    int os = -1;
    int count = 0;
    for (auto overlapset = overlapsets->begin(); overlapset != overlapsets->end(); ++overlapset)
    {
      ++os;
      int p = 0;
      for (auto overlap = overlapset->borderoverlaps_.begin(); overlap != overlapset->borderoverlaps_.end(); ++overlap)
      {
        p++;
        auto cpv = (*overlap)->getCornerPointVector();
        count += cpv->size();
        double x[cpv->size()];
        double y[cpv->size()];
        double z[cpv->size()];
        int i = 0;
        for (auto coordinate = cpv->begin(); coordinate != cpv->end(); ++coordinate)
        {
          x[i] = coordinate->m_X;
          y[i] = coordinate->m_Y;
          ++i;
        }
        double red = 1.0 - os / 6.1;
        double green = os / 6.1;
        std::string options_point = "LineColor=" + std::to_string(red) + "," + std::to_string(green) + ",1.0;LineWidth=0;LineStyle=none;PointSize=11";
        fig->plot("cornerpointsos" + std::to_string(p) + std::to_string(os), x, y, 2.0 + os, i, options_point);
        std::string optionsy = "LineColor=1.0,1.0,0.0;LineWidth=2;PointSize=1";
        std::string options = "LineColor=1.0,0.0,0.0;LineWidth=2;PointSize=1";
        std::string optionsg = "LineColor=0.0,1.0,0.0;LineWidth=2;PointSize=1";
        plotBorder("namelfvl" + std::to_string(p) + std::to_string(os), (*overlap)->m_first_left, 0, 5.0, optionsy, fig);
        plotBorder("namelfvr" + std::to_string(p) + std::to_string(os), (*overlap)->m_first_right, 0, 5.0, optionsy, fig);
        plotBorder("namesl" + std::to_string(p) + std::to_string(os), (*overlap)->m_second_left, 0, 10.0, optionsg, fig);
        plotBorder("namesr" + std::to_string(p) + std::to_string(os), (*overlap)->m_second_right, 0, 10.0, options, fig);
      }
      int counteri = 0;
      std::vector<env::BorderBased::Border *> borderlist;
      std::vector<int> ids;
      lfv_.getBordersToPrint(borderlist, ids);
      std::string optionsmagenta = "LineColor=1.0,0.0,1.0;LineWidth=2;PointSize=2";
      std::string optionsmagenta2 = "LineColor=1.0,0.0,1.0;LineWidth=8;PointSize=8";
      for (auto &b : borderlist)
      {
        if (std::find(ids.begin(), ids.end(), counteri) != ids.end())
        {
          plotBorder("borderpathC" + std::to_string(counteri), b, 0, 30.0, optionsmagenta2, fig);
        }
        else
        {
          plotBorder("borderpathC" + std::to_string(counteri), b, 0, 30.0, optionsmagenta, fig);
        }

        ++counteri;
      }
    }
  }
  void plotCZ(DLR_TS::PlotLab::AFigureStub *fig, int &i)
  {
    auto plotdata = lfv_.getConflictSetPlotData();
    if (i > plotdata->size() - 1)
      i = 0;
    std::string options;
    std::string options_normal = "LineColor=0.0,1.0,0.0;LineWidth=3";
    std::string options_highlighted = "LineColor=1.0,1.0,0.0;LineWidth=6";
    std::string cz_str = "cz_subgroup";
    std::string bo_str = "bo_number";
    int bo_int;
    int cz_int = 0;
    std::cout << "\n plot evaluated borders";
    bool plot = false;
    for (auto it = plotdata->begin(); it != plotdata->end(); ++it)
    {
      plot = i == cz_int ? true : false;
      bo_int = 0;
      std::cout << "number of cz:" << plotdata->size() << std::endl;
      for (auto ij = it->second.begin(); ij != it->second.end(); ++ij)
      {
        std::cout << "   path of this cz" << std::endl;
        options = (bo_int == 0 ? options_highlighted : options_normal);
        if (plot)
          PLOT::plotPath(cz_str + std::to_string(cz_int) + bo_str + std::to_string(bo_int), roadmap_.getBorderSet()->getCenterline((*ij)->m_id).getData(), 0.18, options, fig);
        else
          fig->erase(cz_str + std::to_string(cz_int) + bo_str + std::to_string(bo_int));
        ++bo_int;
      }
      ++cz_int;
    }
    ++i;
  }
};
} // namespace apps
} // namespace adore
