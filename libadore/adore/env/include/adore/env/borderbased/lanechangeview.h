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
    : lrm_(localRoadMap), lfv_(lfv), elt_(trafficMap),cs_(lfv_->getGeometry())
  {
    apLCV_ = paramsfactory->getLaneChangeView();
  }
  /**
   * @brief Update the LaneChangeView
   * 
   * @param direction direction of the LaneChangeView
   */
  void update(adore::view::ALaneChangeView::direction direction)
  {
    int lfg_adjacency_i_start = 0;
    lcg_.update(lfv_->getGeometry(), lrm_->getBorderSet(), lrm_->getVehicleState(), lfg_adjacency_i_start,
                direction, apLCV_->getLookBehind(), apLCV_->getLookAhead());
    elt_.mapVehiclesOnBorders(lfv_, *lcg_.getRightBorders());
    cs_.update(lrm_->getBorderSet(),lrm_->getPrecedenceSet(), &elt_, lcg_.getInnerBorders(), lcg_.getOuterBorders());
  }

public:  // methods derived from ALane
  /**
   * @brief Check whether LaneChangeView is valid
   * 
   * @return true if representation is valid
   * @return false if representation is invalid
   */
  virtual bool isValid() const override
  {
    return lcg_.isValid();
  }
  /**
   * @brief Get the viewing distance 
   * 
   * Returns how far to the horizon the model of the lane extends, given as maximum progress along
   * lane.
   * @return double viewing distance
   */
  virtual double getViewingDistance() const override
  {
    return lcg_.getViewingDistance();
  }
  /**
   * getOnLaneTraffic - return queue of traffic objects moving on lane, ordered by progress of objects on lane
   */
  virtual const adore::view::TrafficQueue& getOnLaneTraffic() const override
  {
    return elt_.getQueue();
  }
  /**
   * getConflictSet - return set of conflict zones, ordered by occurance along lane
   */
  virtual const adore::view::ConflictSet& getConflictSet() const override
  {
    return cs_;
  }
  /**
   * getSpeedLimit - return the speed limit at a certain distance s along the lane
   */
  virtual double getSpeedLimit(double s) const override
  {
    return 20.0;
  }
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

public:  // methods derived from ALaneChangeView
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
   * getProgressOfWidthOpen - return progress s, where target lane is wide enough to contain AV for the first time
   */
  virtual double getProgressOfWidthOpen() const override
  {
    return lcg_.getProgressOfWidthOpen();
  }
  /**
   * getProgressOfWidthClosed - return progress s, where target lane is no longer wide enough to contain AV
   */
  virtual double getProgressOfWidthClosed() const override
  {
    return lcg_.getProgressOfWidthClosed();
  }
  /**
   * getOffsetOfStartOuterBorder - return lateral offset n of the outer border of the AV's current lane
   */
  virtual double getOffsetOfStartOuterBorder(double s) const override
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
};
}  // namespace BorderBased
}  // namespace env
}  // namespace adore