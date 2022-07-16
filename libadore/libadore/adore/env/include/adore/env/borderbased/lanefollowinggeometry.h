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
 *   Matthias Nichting
 ********************************************************************************/


#pragma once
#include <adore/mad/adoremath.h>
#include <adore/mad/fun_essentials.h>
#include <adore/mad/centerline.h>
#include <adore/env/borderbased/borderaccumulator.h>
#include <adore/env/borderbased/bordertrace.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/env/borderbased/bordercostmap.h>
#include <adore/mad/linearfunctiontypedefs.h>

#include "csaps.h"

namespace adore
{
  namespace env
  {
    namespace BorderBased
    {
      template <int PolyFitPoints, int PolyEvaluatePoints>
      /**
       * @brief A class with a geometry description of the current lane
       * 
       */
      class LaneFollowingGeometry
      {

      public:
        using function_type_xyz = adore::mad::function_type_xyz;
        using function_type2d = adore::mad::function_type2d;
        using function_type_scalar = adore::mad::function_type_scalar;
        using velocity_profile = function_type_scalar;
        velocity_profile m_velocity_fct;
        function_type_xyz m_leftBorder_fct; /**< function: s-coordinate -> euclidian coordinates for left borders*/
        function_type_xyz m_rightBorder_fct; /**< function: s-coordinate -> euclidian coordinates for right borders*/
        function_type_xyz m_centerRaw_fct;  /**< function: s-coordinate -> euclidian coordinates for raw centerline*/
        function_type_xyz m_centerSmoothed_fct;  /**< function: s-coordinate -> euclidian coordinates for smoothed centerline*/
        function_type_xyz m_centerSmoothedDerivative1_fct;
        function_type_xyz m_centerSmoothedDerivative2_fct;
        function_type2d m_centerNormal_fct;
        function_type_scalar m_centerHeading_fct;
        function_type_scalar m_centerSmoothedCurvature_fct;
        function_type_scalar m_centerSmoothedCurvatureDerivative_fct;
        function_type_scalar m_leftDistance_fct; /**< function: s-coordinate -> distance to left border */
        function_type_scalar m_rightDistance_fct; /**< function: s-coordinate -> distance to right border */
        function_type_scalar m_navigationCost_fct; /**< function: s-coordinate -> distance to goal */
        double m_planning_time;
        double m_s_min;                         /**< s-coordinate of viewing horizon start */
        double m_s_max;                         /**< s-coordinate of viewing horizon end */
        bool m_view_valid;                      /**< indicate whether view is valid */
        double m_min_view_distance;
        BorderAccumulator m_rightBorders;
        BorderAccumulator m_leftBorders;
        int m_startIndexInRightBorders;
        Border* m_start;
        int m_egoBorderIndex;
        double m_s_lane_width_open;
        double m_s_lane_width_closed;
        double m_vehicle_width;                              /**< ego vehicle width */

        double Sbuf[PolyEvaluatePoints];
        double Xbuf[PolyEvaluatePoints];
        double Ybuf[PolyEvaluatePoints];
        double nXbuf[PolyEvaluatePoints];
        double nYbuf[PolyEvaluatePoints];
        double dXbuf[PolyEvaluatePoints];
        double dYbuf[PolyEvaluatePoints];
        double ddXbuf[PolyEvaluatePoints];
        double ddYbuf[PolyEvaluatePoints];
        double dddXbuf[PolyEvaluatePoints];
        double dddYbuf[PolyEvaluatePoints];

      public:
      /**
       * @brief Construct a new LaneFollowingGeometry object
       * 
       */
        LaneFollowingGeometry()
        {
          m_planning_time = 15.0;
          m_min_view_distance = 5;  // m
          m_centerSmoothed_fct.setData(dlib::zeros_matrix<double>(4, PolyEvaluatePoints));
          m_centerSmoothedDerivative1_fct.setData(dlib::zeros_matrix<double>(4, PolyEvaluatePoints));
          m_centerSmoothedDerivative2_fct.setData(dlib::zeros_matrix<double>(4, PolyEvaluatePoints));
          m_centerNormal_fct.setData(dlib::zeros_matrix<double>(3, PolyEvaluatePoints));
          m_centerHeading_fct.setData(dlib::zeros_matrix<double>(2, PolyEvaluatePoints));
          m_centerSmoothedCurvature_fct.setData(dlib::zeros_matrix<double>(2, PolyEvaluatePoints));
          m_centerSmoothedCurvatureDerivative_fct.setData(dlib::zeros_matrix<double>(2, PolyEvaluatePoints));
          m_leftDistance_fct.setData(dlib::zeros_matrix<double>(2, PolyEvaluatePoints));
          m_rightDistance_fct.setData(dlib::zeros_matrix<double>(2, PolyEvaluatePoints));
          m_vehicle_width = 1.8;
          m_view_valid = false;
        }
        /**
         * @brief update the road geometry
         * 
         * The chosen BorderAccumulationStrategy is BASFollowStraight by default
         * @param borderSet set of all borders
         * @param borderTrace set of historical borders
         * @param borderCost navigation cost
         * @param start start/matched border
         * @param ego ego state
         * @param lookahead_distance how far to look in driving direction
         * @param lookbehind_distance how far to look behind. Note: this parameter requires a borderTrace length of at least equal value
         * @param smoothness smoothing value for polynom fit
         * @param activate_navigation flag whether navigation is activated (default: false) 
         */
        void update(BorderSet* borderSet,
                    BorderTrace* borderTrace,
                    BorderCostMap* borderCost,
                    Border* start,
                    adore::env::VehicleMotionState9d* ego,
                    double lookahead_distance,
                    double lookbehind_distance,
                    double smoothness = 0.05,
                    bool activate_navigation = false
                    )
        {
          m_view_valid = false;  // set to true at end
          m_start = start;
          if(start==nullptr) return; //if the start node is not available, then the view is not valid

          const double excess_distance_border = 10.0;
          borderTrace->setDistanceLimit(lookbehind_distance + excess_distance_border);

          /* go back n steps in view */
          double s0_border =
              start->m_path->getClosestParameter(ego->getX(), ego->getY(), 1, 2);  // position on current border
          double walkback_distance = s0_border;
          m_rightBorders.clear();
          m_egoBorderIndex = 0;
          for (auto it = borderTrace->rbegin(); it != borderTrace->rend() && !(it->first == start->m_id); it++)
          {
            auto current = borderSet->getBorder(it->first);
            if (current == 0)
            {
              // the border is out of scope - clear everything to prevent gaps
              m_rightBorders.clear();
              walkback_distance = s0_border;
            }
            else
            {
              m_rightBorders.append(current, false);  //@TODO check how the direction can be determined
              walkback_distance += it->second;
            }
            m_egoBorderIndex++;  // the border with the ego vehicle is further to the front
          }

					if(activate_navigation)
					{
						BorderBased::BASFollowNavigation bas(start,borderSet,borderCost,lookahead_distance+s0_border + excess_distance_border);
            m_rightBorders.append(&bas);
					}
					else
					{
            BASFollowStraight bas(start, borderSet, lookahead_distance + s0_border + excess_distance_border);
            m_rightBorders.append(&bas);
					}
          
          BASNeighbor basNeighbor(m_rightBorders.getBorders(), borderSet,
                                                        BASNeighbor::LEFT);
          m_leftBorders.clear();
          m_leftBorders.append(&basNeighbor);

          m_leftBorders.defineFunction(m_leftBorder_fct);
          m_rightBorders.defineFunction(m_rightBorder_fct);

          /* create the center line function */
          adoreMatrix<double, 0, 0> center_data;
          center_data.set_size(4, m_leftBorder_fct.getData().nc() + m_rightBorder_fct.getData().nc() - 2);
          int K = adore::mad::computeCenterline(m_leftBorder_fct.getData(), m_rightBorder_fct.getData(), center_data);
          m_centerRaw_fct.setData(dlib::colm(center_data, dlib::range(0, K - 1)));

          /* create s grid for smoothing of centerline */
          double s0_raw = m_centerRaw_fct.getClosestParameter(ego->getX(), ego->getY(), 1, 2);
          double review_distance = std::min(walkback_distance, lookbehind_distance);
          double preview_distance = lookahead_distance;
          m_s_min = (std::max)(s0_raw - review_distance, (std::min)(s0_raw, m_centerRaw_fct.limitLo() + 0.1));
          m_s_max = (std::min)(s0_raw + preview_distance, m_centerRaw_fct.limitHi() - 0.1);
          if (m_s_max - m_s_min < m_min_view_distance)
            return;  // TODO is this not an error that should be handled?

          /* sample the raw centerline at grid points */
          adore::mad::linspace(m_s_min, m_s_max, Sbuf, PolyFitPoints);
          for (int i = 0; i < PolyFitPoints; i++)
          {
            Xbuf[i] = m_centerRaw_fct.fi(Sbuf[i], 0);
            Ybuf[i] = m_centerRaw_fct.fi(Sbuf[i], 1);
          }
        

          /* smooth centerline - compute piecewise poly regression */

          csaps::DoubleArray sdata(PolyFitPoints);
          csaps::DoubleArray xdata(PolyFitPoints);
          csaps::DoubleArray ydata(PolyFitPoints);

          
          // TODO this should be an intermediate solution
          // it is still way faster than the previous hand rolled
          // cubic spline implementation, but alternatives
          // need to be considered for refactoring (e.g. gsl)
          for (int i = 0; i < PolyFitPoints; ++i)
          {
            sdata(i) = Sbuf[i];
            xdata(i) = Xbuf[i];
            ydata(i) = Ybuf[i];
          }

          csaps::UnivariateCubicSmoothingSpline splineX(sdata, xdata, smoothness);
          csaps::UnivariateCubicSmoothingSpline splineY(sdata, ydata, smoothness);

          /* sample smoothed centerline and evaluate its curvature and curv-derivative at grid points*/
          adore::mad::linspace(m_s_min, m_s_max, Sbuf, PolyEvaluatePoints);
          csaps::DoubleArray xidata;
          csaps::DoubleArray dxidata;
          csaps::DoubleArray ddxidata;
          csaps::DoubleArray dddxidata;
          csaps::DoubleArray yidata;
          csaps::DoubleArray dyidata;
          csaps::DoubleArray ddyidata;
          csaps::DoubleArray dddyidata;
          
          std::tie(xidata,dxidata,ddxidata,dddxidata) = splineX(PolyEvaluatePoints, sdata);
          std::tie(yidata,dyidata,ddyidata,dddyidata) = splineY(PolyEvaluatePoints, sdata);


          for (int i = 0; i < PolyEvaluatePoints; ++i)
          {
            Xbuf[i] = xidata(i);
            dXbuf[i] = dxidata(i);
            ddXbuf[i] = ddxidata(i);
            dddXbuf[i] = dddxidata(i);
            Ybuf[i] = yidata(i);
            dYbuf[i] = dyidata(i);
            ddYbuf[i] = ddyidata(i);
            dddYbuf[i] = dddyidata(i);
          }
          
          double si = Sbuf[0];  // integral: path length
          double Li = 0.0;      // length of path segment
          double dLi = 1.0;     // integral: path length -> derivative ratio
          for (int i = 0; i < PolyEvaluatePoints; i++)
          {
            Sbuf[i] = si;
            dLi = std::sqrt(dXbuf[i] * dXbuf[i] + dYbuf[i] * dYbuf[i]);
            if (i + 1 < PolyEvaluatePoints)
            {
              double dx = Xbuf[i + 1] - Xbuf[i];
              double dy = Ybuf[i + 1] - Ybuf[i];
              Li = (std::sqrt)(dx * dx + dy * dy);  // approximate distance along center line
            }

            m_centerSmoothed_fct.getData()(0, i) = si;
            m_centerSmoothed_fct.getData()(1, i) = Xbuf[i];
            m_centerSmoothed_fct.getData()(2, i) = Ybuf[i];

            m_centerSmoothedDerivative1_fct.getData()(0, i) = si;
            m_centerSmoothedDerivative1_fct.getData()(1, i) = dXbuf[i] / dLi;
            m_centerSmoothedDerivative1_fct.getData()(2, i) = dYbuf[i] / dLi;

            m_centerSmoothedDerivative2_fct.getData()(0, i) = si;
            m_centerSmoothedDerivative2_fct.getData()(1, i) =
                ddXbuf[i] / dLi /
                dLi;  //@TODO: approximation: derivative d²q/dt²*(dt/ds)²+dq/dt*d²t/ds² missing part dq/dt*d²t/ds²
            m_centerSmoothedDerivative2_fct.getData()(2, i) =
                ddYbuf[i] / dLi /
                dLi;  //@TODO: approximation: derivative d²q/dt²*(dt/ds)²+dq/dt*d²t/ds² missing part dq/dt*d²t/ds²

            // normal vector
            nXbuf[i] = -dYbuf[i] / dLi;
            nYbuf[i] = dXbuf[i] / dLi;
            m_centerNormal_fct.getData()(0, i) = si;
            m_centerNormal_fct.getData()(1, i) = nXbuf[i];
            m_centerNormal_fct.getData()(2, i) = nYbuf[i];
            m_centerHeading_fct.getData()(0, i) = si;
            m_centerHeading_fct.getData()(1, i) = std::atan2(dYbuf[i], dXbuf[i]);

            m_centerSmoothedCurvature_fct.getData()(0, i) = si;
            m_centerSmoothedCurvature_fct.getData()(1, i) =
                (ddYbuf[i] * dXbuf[i] - ddXbuf[i] * dYbuf[i]) / (dXbuf[i] * dXbuf[i] + dYbuf[i] * dYbuf[i]);

            m_centerSmoothedCurvatureDerivative_fct.getData()(0, i) = si;
            m_centerSmoothedCurvatureDerivative_fct.getData()(1, i) =
                (dddYbuf[i] * dXbuf[i] - dddXbuf[i] * dYbuf[i]) / (dXbuf[i] * dXbuf[i] + dYbuf[i] * dYbuf[i]) -
                (ddYbuf[i] * dXbuf[i] - ddXbuf[i] * dYbuf[i]) * (2.0 * ddYbuf[i] * dYbuf[i] + 2.0 * ddXbuf[i] * dXbuf[i]) /
                    (dXbuf[i] * dXbuf[i] + dYbuf[i] * dYbuf[i]) / (dXbuf[i] * dXbuf[i] + dYbuf[i] * dYbuf[i]);

            // integrate the path length
            if (i + 1 < PolyEvaluatePoints)
            {
              si += Li;
            }
          }
          m_s_max = si;

          /* angular correction: correct heading in order to prevent jumps between -pi and pi */
          adore::mad::createAngularContinuity(m_centerHeading_fct.getData(),1);

          /* compute distance from smoothed centerline to borders*/
          double s_intersect;
          double distance;
          double max_intersect_distance = 5.0;//maximum distance from smooth baseline to border
          bool extend_fringes = false;  // true - extend fringes of border in order to guarantee 
                                        //        intersection with smoothed centerline normals
          m_view_valid = true;          // set view valid to false, if lane boundary cannot be found inside max_intersect_distance
          // left
          s_intersect = m_leftBorder_fct.limitLo();     // initialize at border's beginning
          // s_intersect = m_s_min;//initialize at baseline start
          for (int i = 0; i < PolyEvaluatePoints; i++)  // step through smoothed centerline points and compute distance
          {
            bool rv = m_leftBorder_fct.getNextIntersectionWithVector2d(s_intersect, Xbuf[i], Ybuf[i], nXbuf[i], nYbuf[i], s_intersect,
                                                            distance, max_intersect_distance, extend_fringes);
            // m_view_valid = m_view_valid && rv;
            m_leftDistance_fct.getData()(0, i) = Sbuf[i];
            m_leftDistance_fct.getData()(1, i) = distance;
          }
          // right
          s_intersect = m_rightBorder_fct.limitLo();    // initialize at border's beginning
          // s_intersect = m_s_min;//initialize at baseline start
          for (int i = 0; i < PolyEvaluatePoints; i++)  // step through smoothed centerline points and compute distance
          {
            bool rv = m_rightBorder_fct.getNextIntersectionWithVector2d(s_intersect, Xbuf[i], Ybuf[i], -nXbuf[i], -nYbuf[i],
                                                              s_intersect, distance, max_intersect_distance, extend_fringes);
            // m_view_valid = m_view_valid && rv;
            m_rightDistance_fct.getData()(0, i) = Sbuf[i];
            m_rightDistance_fct.getData()(1, i) = -distance;
          }
          bool width_open = false;
          m_s_lane_width_open = 0.0;
          m_s_lane_width_closed = 10000.0;
          auto left = m_leftDistance_fct.getData();
          for (int i = 0; i < left.nc(); i++)
          {
              double d = (std::abs)(left(1, i)) + (std::abs)(m_rightDistance_fct.fi(left(0, i), 0)) - m_vehicle_width;
              if (width_open)
              {
                  if (d < 0)
                  {
                      m_s_lane_width_closed = left(0, i);
                      break;
                  }
              }
              else
              {
                  if (d > 0)
                  {
                      m_s_lane_width_open = left(0, i);
                      width_open = true;
                  }
              }
          }

          m_view_valid =  computeNavigationCost(activate_navigation,borderCost);

        }


        bool computeNavigationCost(bool activate_navigation,BorderCostMap* borderCostMap)
        {
          if(activate_navigation)
          {
            double d_sample = 5.0;
            std::vector<adoreMatrix<double,3,1>> navcost_vector;
            for(Border* rb:(*m_rightBorders.getBorders()))
            {
                auto c = borderCostMap->find(rb->m_id);
                if(c==borderCostMap->end())return false;
                adoreMatrix<double,3,1> value;
                value(0) = c->second.getDistanceToGoal();
                value(1) = rb->m_id.m_first.m_X;
                value(2) = rb->m_id.m_first.m_Y;
                navcost_vector.push_back(value);
                if(rb->m_path!=nullptr && rb->getLength()>d_sample)
                {
                  for(double d = d_sample;d<rb->getLength();d+=d_sample)
                  {
                    auto p = rb->m_path->f(d);
                    value(0) = c->second.getDistanceToGoal() + d;
                    value(1) = p(0);
                    value(2) = p(1);
                    navcost_vector.push_back(value);
                  }
                }
            }
            {
                Border* rb = *m_rightBorders.getBorders()->rbegin();
                auto c = borderCostMap->find(rb->m_id);
                if(c==borderCostMap->end())return false;
                adoreMatrix<double,3,1> value;
                value(0) = std::max(0.0,c->second.getDistanceToGoal()-rb->getLength());
                value(1) = rb->m_id.m_last.m_X;
                value(2) = rb->m_id.m_last.m_Y;
                navcost_vector.push_back(value);
                if(rb->m_path!=nullptr && rb->getLength()>d_sample)
                {
                  for(double d = d_sample;d<rb->getLength();d+=d_sample)
                  {
                    auto p = rb->m_path->f(d);
                    value(0) = c->second.getDistanceToGoal() + d;
                    value(1) = p(0);
                    value(2) = p(1);
                    navcost_vector.push_back(value);
                  }
                }
            }
            //project navigation cost points to baseline
            std::vector<double> svalues;
            std::vector<double> cvalues;
            auto xstart = m_centerSmoothed_fct(m_centerSmoothed_fct.limitLo());
            auto xend = m_centerSmoothed_fct(m_centerSmoothed_fct.limitHi());
            for(auto& value:navcost_vector)
            {
                double tmp=0;//not required
                double s = m_centerSmoothed_fct.getClosestParameter(value(1),value(2),1,2,tmp);
                double c;//cost
                if(s==m_centerSmoothed_fct.limitLo())//potentially located before interval start
                {
                    double dx = value(1)-xstart(0);
                    double dy = value(2)-xstart(1);
                    double d = std::sqrt(dx*dx+dy*dy);
                    if(d>1.5*d_sample)continue;
                    c = std::max(0.0,value(0) - d);
                }
                else if(s==m_centerSmoothed_fct.limitHi())//potentially located after interval end
                {
                    double dx = value(1)-xend(0);
                    double dy = value(2)-xend(1);
                    double d = std::sqrt(dx*dx+dy*dy);
                    if(d>1.5*d_sample)continue;
                    c = std::max(0.0,value(0) + d);
                }
                else
                {  
                    c = value(0); 
                }
                if( svalues.size()>0 && s<svalues[svalues.size()-1])continue;
                else if( svalues.size()>0 && s==svalues[svalues.size()-1])
                {
                  if(c<cvalues[cvalues.size()-1])
                  {
                    svalues[svalues.size()-1] = s;
                    cvalues[svalues.size()-1] = c;
                  }
                }
                else
                {
                    svalues.push_back(s);
                    cvalues.push_back(c);
                }        
            }
            if(svalues.size()>0)
            {
                m_navigationCost_fct.setData(dlib::zeros_matrix<double>(2, svalues.size()));
                for(int i=0;i<svalues.size();i++)
                {
                    m_navigationCost_fct.getData()(0,i) = svalues[i];
                    m_navigationCost_fct.getData()(1,i) = cvalues[i];
                }            
            }
            else
            {
                m_navigationCost_fct.setData(dlib::zeros_matrix<double>(2, 2));
                m_navigationCost_fct.getData()(0,0) = m_centerSmoothed_fct.limitLo();
                m_navigationCost_fct.getData()(0,1) = m_centerSmoothed_fct.limitHi();
                m_navigationCost_fct.getData()(1,0) = 1e10;
                m_navigationCost_fct.getData()(1,1) = 1e10;
            }
          }
          else
          {
            m_navigationCost_fct.setData(dlib::zeros_matrix<double>(2, 2));
            m_navigationCost_fct.getData()(0,0) = m_centerSmoothed_fct.limitLo();
            m_navigationCost_fct.getData()(0,1) = m_centerSmoothed_fct.limitHi();
            m_navigationCost_fct.getData()(1,0) = 1.0e10;
            m_navigationCost_fct.getData()(1,1) = 1.0e10;
          }
          return true;
        }

        /**
         * @brief Modify a lane boundary to exclude a given point.
         * 
         * Modifies left or right lane boundary in such a way that the specified point no longer
         * resides inside the lane after the operation. The lane boundary with minimum distance to the point is chosen for
         * modification.
         * @param X x-coordinate of the point to exclude
         * @param Y y-coordinate of the point to exclude
         * @param s_min 
         * @param s_safety 
         * @param n_safety 
         * @param s_off 
         */
        void excludeObstaclePoint(double X, double Y, double s_min, double s_safety, double n_safety, double s_off = 0.0)
        {
          double s, n;
          this->toLocalCoordinates(X, Y, s, n);
          s = adore::mad::bound(m_centerSmoothed_fct.limitLo(), s + s_off, m_centerSmoothed_fct.limitHi());
          ;
          if (s > s_min)
          {
            double l = m_leftDistance_fct(s);
            double r = m_rightDistance_fct(s);
            if (n > 0.0)
            {
              for (int i = m_leftDistance_fct.findIndex((std::max)(s - s_safety, m_centerSmoothed_fct.limitLo()));
                  i < m_leftDistance_fct.getData().nc(); i++)
              {
                m_leftDistance_fct.getData()(1, i) = (std::min)(n - n_safety, m_leftDistance_fct.getData()(1, i));
                if (m_leftDistance_fct.getData()(0, i) > s + s_safety)
                  break;
              }
            }
            else
            {
              for (int i = m_rightDistance_fct.findIndex((std::max)(s - s_safety, m_centerSmoothed_fct.limitLo()));
                  i < m_rightDistance_fct.getData().nc(); i++)
              {
                m_rightDistance_fct.getData()(1, i) = (std::max)(n + n_safety, m_rightDistance_fct.getData()(1, i));
                if (m_rightDistance_fct.getData()(0, i) > s + s_safety)
                  break;
              }
            }
          }
        }
        /**
         * @brief Get the right borders of the LaneFollowingGeometry
         * 
         * @return BorderAccumulator* BorderAccumulator that holds the right borders
         */
        BorderAccumulator* getRightBorders()
        {
          return &m_rightBorders;
        }
        /**
         * @brief Get the left borders of the LaneFollowingGeometry
         * 
         * @return BorderAccumulator* BorderAccumulator that holds the left borders
         */
        BorderAccumulator* getLeftBorders()
        {
          return &m_leftBorders;
        }
        /**
         * @brief Get the best matching border for a given ego position
         * @param max_deviation if a point located max_deviation from X,Y is inside lfg, a match can be found
         * @return nullptr if no match can be found, otherwise border pointer
         */
        Border* getBestMatchingBorder(double X,double Y,double max_deviation)
        {
          double s,n;
          toRelativeCoordinates(X,Y,s,n);
          if( s<=0.0 || s>=getViewingDistance() )return nullptr;
          double nmin = - max_deviation + (std::min)(getOffsetOfLeftBorder(s),getOffsetOfRightBorder(s));
          double nmax = + max_deviation + (std::max)(getOffsetOfLeftBorder(s),getOffsetOfRightBorder(s));
          if(n<nmin||nmax<n)
          {
            return nullptr;
          }

          double Xc,Yc,Zc;
          toEucledianCoordinates(s,0,Xc,Yc,Zc);

          auto it_left = m_leftBorders.getBorders()->begin();
          auto it_right = m_rightBorders.getBorders()->begin();
          
          while (it_left!=m_leftBorders.getBorders()->end()
             &&  it_right!=m_rightBorders.getBorders()->end())
          {
              if((*it_right)->isPointInsideLane(*it_left,Xc,Yc))
              {
                return *it_right;
              }
              else
              {
                it_left ++;
                it_right ++;
              }
              
          }
          return nullptr;
        }


        /**
         * @brief Check whether the LaneFollowingGeometry is valid
         * 
         * @return true if the LaneFollowingGeometry is valid
         * @return false if the LaneFollowingGeometry is invalid
         */
        bool isValid() const
        {
          return m_view_valid;
        }
        /**
         * @brief Get the viewing distance
         * 
         * @return double viewing distance
         */
        double getViewingDistance()  const
        {
          return m_s_max - m_s_min;
        }
        double getSMax()const
        {
          return m_s_max;
        }
        double getSMin()const
        {
          return m_s_min;
        }
        /**
         * @brief Get the s-coordinate where the lane reaches the required width
         *
         * @return double s-coordinate of the position where the lane starts to have at least the required width
         */
        double getProgressOfWidthOpen() const
        {
            return m_s_lane_width_open;
        }
        /**
         * @brief Get the s-coordinate where the lane stops to have the required width
         *
         * @return double s-coordinate of the position where the lane ends to have at least the required width
         */
        double getProgressOfWidthClosed() const
        {
            return m_s_lane_width_closed;
        }
        /**
         * @brief Get the centerline of the lane
         * 
         * @return function_type_xyz* centerline of the lane
         */
        function_type_xyz* getCenterline()  
        {
          // return &m_centerRaw_fct;
          return &m_centerSmoothed_fct;
        }
        /**
         * @brief Get the Centerline Normal object
         * 
         * @return function_type2d* 
         */
        function_type2d* getCenterlineNormal() 
        {
          return &m_centerNormal_fct;
        }
        /**
         * @brief Get the curvature of the lane at a certain position
         * 
         * @param s s-coordinate of the position
         * @param derivative which derivative of the curvature should be evaluated
         * @return double resulting curvature at the evaluated position
         */
        double getCurvature(double s, int derivative) 
        {
          s=adore::mad::bound(m_s_min,s+m_s_min,m_s_max);
          if (derivative<=0)
          {
            return m_centerSmoothedCurvature_fct(s);
          }
          else if(derivative<=1)
          {
            return m_centerSmoothedCurvatureDerivative_fct(s);
          }
          else
          {
            return 0.0;
          }
        }
        /**
         * @brief Get the begin()-iterator of the BAContainer of the right borders
         * 
         * @return BAContainer::iterator begin()-iterator
         */
        BAContainer::iterator begin()
        {
          return m_rightBorders.getBorders()->begin();
        }
        /**
         * @brief Get the end()-iterator of the BAContainer of the right borders
         * 
         * @return BAContainer::iterator end()-iterator
         */
        BAContainer::iterator end()
        {
          return m_rightBorders.getBorders()->end();
        }
        /**
         * @brief Get the heading at a certain position
         * 
         * @param s s-coordinate of the position
         * @return double heading at the position given position
         */
        double getHeading(double s) 
        {
          s=adore::mad::bound(m_s_min,s+m_s_min,m_s_max);
          return m_centerHeading_fct(s);
        }
        /**
         * @brief Get the offset of the left border at a certain position
         * 
         * @param s s-coordinate of the position
         * @return double offset of the left border at the given position
         */
        double getOffsetOfLeftBorder(double s) 
        {
          s=adore::mad::bound(m_s_min,s+m_s_min,m_s_max);
          return m_leftDistance_fct(s);
        }
        /**
         * @brief Get the function that holds the offset of left borders
         * 
         * @return function_type_scalar* function that holds the offset of left borders
         */
        function_type_scalar* getOffsetOfLeftBorderFct() 
        {
          return &m_leftDistance_fct;
        }
        /**
         * @brief Get the offset of the right border at a certain position
         * 
         * @param s s-coordinate of the position
         * @return double offset of the right border at the given position
         */
        double getOffsetOfRightBorder(double s) 
        {
          s=adore::mad::bound(m_s_min,s+m_s_min,m_s_max);
          return m_rightDistance_fct(s);
        }
        /**
         * @brief Get the function that holds the offset of right borders
         * 
         * @return function_type_scalar* function that holds the offset of right borders
         */
        function_type_scalar* getOffsetOfRightBorderFct() 
        {
          return &m_rightDistance_fct;
        }

        function_type_xyz* getLeftBorderFct() 
        {return &m_leftBorder_fct;}
        function_type_xyz* getRightBorderFct()
        {return &m_rightBorder_fct;}

        /**
         * @brief Transform from euclidian to relative coordinates
         * 
         * @param Xg euclidian x-coordinate
         * @param Yg euclidian y-coordinate
         * @param s relative s-coordinate
         * @param n relative n-coordinate
         */
        void toRelativeCoordinates(double Xg, double Yg, double& s, double& n) 
        {
          if(!adore::mad::toRelativeWithNormalExtrapolation(Xg,Yg,&m_centerSmoothed_fct,&m_centerNormal_fct,s,n))
          {
              s = m_centerSmoothed_fct.getClosestParameter(Xg, Yg, 1, 2, n);
          }
          s-=m_s_min;
        }
        /**
         * @brief Transform from relative to euclidian coordinates
         * 
         * @param s relative s-coordinate
         * @param n relative n-coordinate
         * @param Xg euclidian x-coordinate
         * @param Yg euclidian y-coordinate
         * @param Zg euclidian z-coordinate
         */
        void toEucledianCoordinates(double s, double n, double& Xg, double& Yg, double& Zg) 
        {
          s=adore::mad::bound(m_s_min,s+m_s_min,m_s_max);
          adore::mad::fromRelative(s,n,&m_centerSmoothed_fct,&m_centerNormal_fct,Xg,Yg,Zg);
          Zg=0.0;
        }
      };
    }  // namespace BorderBased
  }  // namespace env
}  // namespace adore