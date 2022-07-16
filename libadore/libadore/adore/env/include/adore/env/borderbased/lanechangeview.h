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
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/env/afactory.h>
#include <adore/view/alanechangeview.h>
#include <adore/env/borderbased/lanechangegeometry.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/borderbased/lanefollowingview.h>
#include <adore/params/afactory.h>
#include <adore/env/borderbased/conflictset.h>

namespace adore
{
namespace env
{
namespace BorderBased
{
/**
 * DependentLane is the target lane of a lanechange, which depends heavily on the coordinate system of the current/source lane.
 */
class [[deprecated("lanechangeview is superseded by threelaneviewdecoupled")]] DependentLane: public adore::view::ALane
{
private:
  LaneFollowingView* lfv_; /**< connected LaneFollowingView */
  LaneChangeGeometry* lcg_;/**< connected LaneChangeGeometry */
  adore::env::traffic::EgoLaneTraffic* elt_;   /**< traffic on this lane */
  adore::env::BorderBased::ConflictSet* cs_;   /**< conflict set */
public:
  DependentLane(LaneFollowingView* lfv,LaneChangeGeometry* lcg,adore::env::traffic::EgoLaneTraffic* elt,adore::env::BorderBased::ConflictSet* cs)
    :lfv_(lfv),lcg_(lcg),elt_(elt),cs_(cs){}
public:  // methods derived from ALane
  /**
   * @brief Check whether LaneChangeView is valid
   * 
   * @return true if representation is valid
   * @return false if representation is invalid
   */
  virtual bool isValid() const override
  {
    return lcg_->isValid();
  }
  virtual double getSMax() const override
  {
    return lcg_->getViewingDistance();
  }
  virtual double getSMin() const override
  {
    return 0.0;
  }
  /**
   * @brief Get the s-coordinate where the lane reaches the required width
   *
   * @return double s-coordinate of the position where the lane starts to have at least the required width
   */
  double getProgressOfWidthOpen() const override
  {
      return lcg_->m_s_lane_width_open;
  }
  /**
   * @brief Get the s-coordinate where the lane stops to have the required width
   *
   * @return double s-coordinate of the position where the lane ends to have at least the required width
   */
  double getProgressOfWidthClosed() const override
  {
      return lcg_->m_s_lane_width_closed;
  }
  /**
   * getOnLaneTraffic - return queue of traffic objects moving on lane, ordered by progress of objects on lane
   */
  virtual const adore::view::TrafficQueue& getOnLaneTraffic() const override
  {
    return elt_->getQueue();
  }
  /**
   * getConflictSet - return set of conflict zones, ordered by occurance along lane
   */
  virtual const adore::view::ConflictSet& getConflictSet() const override
  {
    return *cs_;
  }
  /**
   * getSpeedLimit - return the speed limit at a certain distance s along the lane
   */
  [[deprecated("lanechangeview is superseded by threelaneviewdecoupled")]]
  virtual double getSpeedLimit(double s) override
  {
    return 40.0;
  }
  [[deprecated("lanechangeview is superseded by threelaneviewdecoupled")]]
  virtual double getLeftIndicatorHint(double s) override { return 0.0; }

  [[deprecated("lanechangeview is superseded by threelaneviewdecoupled")]]
  virtual double getRightIndicatorHint(double s) override { return 0.0; }
  /**
   * hasSpeedRecommendation - return true, if a speed recommendation is available (GLOSA or other infrastructure advice)
   * at a certain distance s along the lane
   */
  virtual bool hasSpeedRecommendation(double s) const override
  {
    return false;
  }
  /**
   * getSpeedRecommendation - return a speed recommendation at a certain distance s along the lane
   */
  virtual double getSpeedRecommendation(double s) const override
  {
    return 0.0;
  }
  virtual double getNavigationCost(double s) override
  {
    return 0.0;
  }
  virtual void boundNavigationCost(double s0,double s1,double& cmin,double& cmax) 
  {
    cmin = 0.0;cmax = 0.0;
  }
  /**
   *  getHeading - return the heading of the lane at a distance s along the lane
   */
  virtual double getHeading(double s)override
  {
    return lfv_->getHeading(s);
  }
  /**
   * getCurvature - return the lane coordinate system's curvature kappa=1/R and its derivatives 1,2,... at a progress s
   * the derivative is given as 1: d kappa / ds, 2: d^2 kappa / ds^2, ...
   * if a derivative is unavailable, 0 will be returned
   */
  virtual double getCurvature(double s, int derivative) override
  {
    return lfv_->getCurvature(s,derivative);
  }
  /**
   * getOffsetOfLeftBorder - return the lateral offset of the left border at a progress s
   */
  virtual double getOffsetOfLeftBorder(double s) override
  {
    return lcg_->getOffsetOfLeftBorder(s);
  }
  /**
   * getOffsetOfRightBorder - return the lateral offset of the right border at a progress s
   */
  virtual double getOffsetOfRightBorder(double s) override
  {
    return lcg_->getOffsetOfRightBorder(s);
  }
  /**
   * coordinate transformation from euclidean (xe,ye) to road relative coordinates (s,n)
   */
  virtual void toRelativeCoordinates(double xe,double ye,double& s,double& n) override
  {
    lfv_->toRelativeCoordinates(xe,ye,s,n);
  }
  /**
   * coordinate transformation from road relative coordinates (s,n) to euclidean (xe,ye,ze) 
   */
  virtual void toEucledianCoordinates(double s,double n,double& xe,double& ye,double& ze)  override
  {
    lfv_->toEucledianCoordinates(s,n,xe,ye,ze);
  }
};

  /**
   * @brief LaneChangeView provides traffic related information for an adjacent lane.
   * 
   */
class LaneChangeView : public adore::view::ALaneChangeView
{
private:
  LaneFollowingView* lfv_; /**< connected LaneFollowingView */
  LaneChangeGeometry lcg_; /**< connected LaneChangeGeometry */
  LocalRoadMap* lrm_; /**< local road map with border cost and matched lane */
  adore::params::APLaneChangeView* apLCV_; /**< parameters */
  adore::env::traffic::EgoLaneTraffic elt_;   /**< traffic on this lane */
  adore::env::BorderBased::ConflictSet cs_;   /**< conflict set */
  DependentLane dl_;/**< lc target lane represented as a dependent lane*/

public:
/**
 * @brief Construct a new LaneChangeView object
 * 
 * @param paramsfactory parameter factory
 * @param localRoadMap local raod map
 * @param lfg LaneFollowingGeometry
 */
  LaneChangeView(adore::params::AFactory* paramsfactory, LocalRoadMap* localRoadMap,
                 LaneFollowingView* lfv,
                    adore::env::traffic::TrafficMap *trafficMap)
    : lfv_(lfv),lrm_(localRoadMap), elt_(trafficMap),cs_(lfv_->getGeometry()),
      dl_(lfv_,&lcg_,&elt_,&cs_)
  {
    apLCV_ = paramsfactory->getLaneChangeView();
  }

  LaneChangeView(LocalRoadMap* localRoadMap, LaneFollowingView* lfv, adore::env::traffic::TrafficMap *trafficMap) 
  : LaneChangeView(adore::params::ParamsFactoryInstance::get(), localRoadMap, lfv, trafficMap)
  {
    
    // LaneChangeView(adore::params::ParamsFactoryInstance::get(), lfv, trafficMap);
  }

  /**
   * @brief Update the LaneChangeView
   * 
   * @param direction direction of the LaneChangeView
   */
  [[deprecated("lanechangeview is superseded by threelaneviewdecoupled")]]
  void update(adore::view::ALaneChangeView::direction direction)
  {
    int lfg_adjacency_i_start = 0;
    lcg_.update(lfv_->getGeometry(), lrm_->getBorderSet(), lrm_->getVehicleState(), lfg_adjacency_i_start,
                direction, apLCV_->getLookBehind(), apLCV_->getLookAhead());
    elt_.mapVehiclesOnBorders(lfv_, *lcg_.getOuterBorders());
    cs_.update(lrm_->getBorderSet(),lrm_->getPrecedenceSet(), &elt_, lcg_.getInnerBorders(), lcg_.getOuterBorders());
  }
  LaneChangeGeometry *getGeometry() { return &lcg_; }

public:  // methods derived from ALaneChangeView
  /**
   * getSourceLane - return ALane pointer for the source lane of the lane change
   */
  virtual adore::view::ALane* getSourceLane()override 
  {
    return lfv_;
  }
  /**
   * getSourceLane - return ALane pointer for the target lane of the lane change
   */
  virtual adore::view::ALane* getTargetLane()override
  {
    return &dl_;
  }
  /**
   * getLCDirection - return the direction of a lane change leading to target lane
   */
  virtual adore::view::ALaneChangeView::direction getLCDirection() const override
  {
    return lcg_.getLCDirection();
  }
  /**
   * getProgressOfGateOpen - return progress s of the next opening of a gate (distance to end of solid line or otherwise
   * impassable lane border)
   */
  virtual double getProgressOfGateOpen() const override
  {
    return lcg_.getProgressOfGateOpen();
  }
  /**
   * getProgressOfGateClosed - return progress s of the closure of the next gate (distance to beginngin of solid line or
   * otherwise impassable lane border after gate)
   */
  virtual double getProgressOfGateClosed() const override
  {
    return lcg_.getProgressOfGateClosed();
  }
  /**
   * getOffsetOfStartOuterBorder - return lateral offset n of the outer border of the AV's current lane
   */
  virtual double getOffsetOfStartOuterBorder(double s)  override
  {
    if (getLCDirection() == adore::view::ALaneChangeView::LEFT)
    {
      return lfv_->getGeometry()->getOffsetOfRightBorder(s);
    }
    return lfv_->getGeometry()->getOffsetOfLeftBorder(s);
  }
  /**
   * getOffsetOfStartInnerBorder - return lateral offset n of the inner border of the AV's current lane
   */
  virtual double getOffsetOfSeparatingBorder(double s) override
  {
    if (getLCDirection() == adore::view::ALaneChangeView::RIGHT)
    {
      return lfv_->getGeometry()->getOffsetOfRightBorder(s);
    }
    return lfv_->getGeometry()->getOffsetOfLeftBorder(s);
    //return lcg_.getOffsetOfCenterBorder(s);
  }
  /**
   * getOffsetOfDestinationOuterBorder - return lateral offset n of the outer border of target lane
   */
  virtual double getOffsetOfDestinationOuterBorder(double s) override
  {
    if (getLCDirection() == adore::view::ALaneChangeView::RIGHT)
    {
      return lcg_.getOffsetOfRightBorder(s);
    }
    return lcg_.getOffsetOfLeftBorder(s);
  }
  virtual double getNavigationCostDifference() override
  {
      return 0.0;
  }


};
}  // namespace BorderBased
}  // namespace env
}  // namespace adore