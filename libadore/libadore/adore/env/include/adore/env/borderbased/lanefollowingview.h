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
#include <adore/env/borderbased/lanefollowinggeometry.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/traffic/egolanetraffic.h>
#include <adore/params/afactory.h>
#include <adore/view/alane.h>
#include <adore/env/borderbased/conflictset.h>
//#include <adore/env/traffic/participant.h>


namespace adore
{
namespace env

{
namespace BorderBased
{
/**
 * @brief LaneFollowingview provides traffic related information for the current
 * lane
 *
 */

class [[deprecated("lanefollowingview is superseded by threelaneviewdecoupled")]] LaneFollowingView : public adore::view::ALane
{
public:
  typedef LaneFollowingGeometry<20, 200> TLaneFollowingGeometry;

private:
  LocalRoadMap *lrm_;                         /**< local road map */
  TLaneFollowingGeometry lfg_;                /**< LaneFollowingGeometry */
  adore::params::APLaneFollowingView *apLFV_; /**< parameters */
  adore::env::traffic::EgoLaneTraffic elt_;   /**< traffic on this lane */
  adore::env::BorderBased::ConflictSet cs_;               /**< conflict set */


public:
  /**
   * @brief Construct a new LaneFollowingView object
   *
   * @param paramsfactory parameter factory
   * @param localRoadMap local road map
   * @param trafficMap traffic map
   */
 LaneFollowingView(adore::params::AFactory *paramsfactory,
                    LocalRoadMap *localRoadMap,
                    adore::env::traffic::TrafficMap *trafficMap)
      : lrm_(localRoadMap), elt_(trafficMap), cs_(&lfg_)
  {
    apLFV_ = paramsfactory->getLaneFollowingView();
  }

  LaneFollowingView(LocalRoadMap *localRoadMap,
                    adore::env::traffic::TrafficMap *trafficMap)
      : LaneFollowingView(adore::params::ParamsFactoryInstance::get(),localRoadMap,trafficMap)
  {}




  std::unordered_multimap<view::ConflictZone*, std::vector<Border*>>* getConflictSetPlotData()
  {
    return getConflictSetPlotData();
  }
  /**
   * @brief update the LaneFollowingView
   *
   */
  [[deprecated("lanefollowingview is superseded by threelaneviewdecoupled")]]
  void update()
  {
    lfg_.update(lrm_->getBorderSet(), lrm_->getBorderTrace(),
                lrm_->getBorderCostMap(), lrm_->getMatchedBorder(),
                lrm_->getVehicleState(), apLFV_->getLookAhead(), apLFV_->getLookBehind(),
                apLFV_->getBaselineFitSmoothness(), lrm_->isNavigationActive());
    elt_.mapVehiclesOnBorders(this, *lfg_.getRightBorders()->getBorders());
    cs_.update(lrm_->getBorderSet(),lrm_->getPrecedenceSet(), &elt_, lfg_.getRightBorders()->getBorders(),lfg_.getLeftBorders()->getBorders());
  }
  LocalRoadMap *getLocalRoadMap() { return lrm_; }
  

  void getBordersToPrint(std::vector<Border*> &result,std::vector<int> &ids)
  {
   return cs_.getBordersToPrint(result,ids);
  }


  TLaneFollowingGeometry *getGeometry() { return &lfg_; }

public: // methods from ALane
  /**
   * isValid - return true if representation of lane is valid
   */
  virtual bool isValid() const override { return lfg_.isValid(); }
  virtual double getSMax() const override
  {
    return lfg_.getViewingDistance();
  }
  virtual double getSMin() const override
  {
    return 0.0;
  }
  /**
   * getProgressOfWidthOpen - returns the s-coordinate of the position where the lane starts to 
   * have at least the required width
   */
  virtual double getProgressOfWidthOpen() const override
  {
      return lfg_.m_s_lane_width_open;
  }
  /**
   * getProgressOfWidthClosed - returns the s-coordinate of the position where the lane ends to 
   * have at least the required width
   */
  virtual double getProgressOfWidthClosed() const override
  {
      return lfg_.m_s_lane_width_closed;
  }
  /**
   * getOnLaneTraffic - return queue of traffic objects moving on lane, ordered
   * by progress of objects on lane
   */
  virtual const adore::view::TrafficQueue &getOnLaneTraffic() const override
  {
    return elt_.getQueue();
  }
  /**
   * getConflictSet - return set of conflict zones, ordered by occurance along
   * lane
   */
  virtual const adore::view::ConflictSet &getConflictSet() const override
  {
    return (adore::view::ConflictSet&)cs_;
  }
  /**
   * getSpeedLimit - return the speed limit at a certain distance s along the
   * lane
   */
  [[deprecated("lanefollowingview is superseded by threelaneviewdecoupled")]]
  virtual double getSpeedLimit(double s) override { return 40.0; }

  [[deprecated("lanefollowingview is superseded by threelaneviewdecoupled")]]
  virtual double getLeftIndicatorHint(double s) override { return 0.0; }

  [[deprecated("lanefollowingview is superseded by threelaneviewdecoupled")]]
  virtual double getRightIndicatorHint(double s) override { return 0.0; }
  /**
   * hasSpeedRecommendation - return true, if a speed recommendation is
   * available (GLOSA or other infrastructure advice) at a certain distance s
   * along the lane
   */
  virtual bool hasSpeedRecommendation(double s) const override { return false; }
  /**
   * getSpeedRecommendation - return a speed recommendation at a certain
   * distance s along the lane
   */
  virtual double getSpeedRecommendation(double s) const override { return 0.0; }

  virtual double getNavigationCost(double s) override {return 0.0;}
  virtual void boundNavigationCost(double s0,double s1,double& cmin,double& cmax) override
  {
    cmin = 0.0;cmax = 0.0;
  }

public: // methods from ALane
  /**
   *  getHeading - return the heading of the lane at a distance s along the lane
   */
  virtual double getHeading(double s) override { return lfg_.getHeading(s); }
  /**
   * getCurvature - return the lane coordinate system's curvature kappa=1/R and
   * its derivatives 1,2,... at a progress s the derivative is given as 1: d
   * kappa / ds, 2: d^2 kappa / ds^2, ... if a derivative is unavailable, 0 will
   * be returned
   */
  virtual double getCurvature(double s, int derivative) override
  {
    return lfg_.getCurvature(s, derivative);
  }
  /**
   * getOffsetOfLeftBorder - return the lateral offset of the left border at a
   * progress s
   */
  virtual double getOffsetOfLeftBorder(double s) override
  {
    return lfg_.getOffsetOfLeftBorder(s);
  }
  /**
   * getOffsetOfRightBorder - return the lateral offset of the right border at a
   * progress s
   */
  virtual double getOffsetOfRightBorder(double s) override
  {
    return lfg_.getOffsetOfRightBorder(s);
  }
  /**
   * coordinate transformation from euclidean (xe,ye) to road relative
   * coordinates (s,n)
   */
  virtual void toRelativeCoordinates(double xe, double ye, double &s,
                                     double &n) override
  {
    lfg_.toRelativeCoordinates(xe, ye, s, n);
  }
  /**
   * coordinate transformation from road relative coordinates (s,n) to euclidean
   * (xe,ye,ze)
   */
  virtual void toEucledianCoordinates(double s, double n, double &xe,
                                      double &ye, double &ze) override
  {
    lfg_.toEucledianCoordinates(s, n, xe, ye, ze);
  }

  std::vector<Coordinate> *getCornerPoints() { return cs_.getCornerPoints(); }
  std::vector<BorderOverlapSet> *getOverlapSet() { return cs_.getOverlapSet(); }
  std::vector<Border *> *get_right_borders_of_conf_lanes()
  {
    return cs_.get_right_borders_of_conf_lanes();
  }

  
  
};
} // namespace BorderBased
} // namespace env
} // namespace adore
