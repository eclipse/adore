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
#include <adore/mad/adoremath.h>
#include <adore/mad/fun_essentials.h>
#include <adore/mad/centerline.h>
#include <adore/mad/cubicpiecewisefunction.h>
#include <adore/mad/cubicpiecewisefunctionstatic.h>
#include <adore/env/borderbased/borderaccumulator.h>
#include <adore/env/borderbased/bordertrace.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/env/borderbased/bordercostmap.h>

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
        // typedefs
        typedef adore::mad::LLinearPiecewiseFunctionM<double, 1> velocity_profile;
        typedef adore::mad::LLinearPiecewiseFunctionM<double, 3> function_type_xyz;
        typedef adore::mad::LLinearPiecewiseFunctionM<double, 2> function_type2d;
        typedef adore::mad::LLinearPiecewiseFunctionM<double, 1> function_type_scalar;
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
        adore::mad::CubicPiecewiseFunctionStatic<PolyFitPoints> m_cp_fit_X;
        adore::mad::CubicPiecewiseFunctionStatic<PolyFitPoints> m_cp_fit_Y;

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
        double Wbuf[PolyEvaluatePoints];

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
         * @param smoothness smoothing value for polynom fit
         * @param activate_navigation flag whether navigation is activated (default: false) 
         */
        void update(BorderSet* borderSet,
                    BorderTrace* borderTrace,
                    BorderCostMap* borderCost,
                    Border* start,
                    adore::env::VehicleMotionState9d* ego,
                    double lookahead_distance,
                    double smoothness = 0.05,
                    bool activate_navigation = false
                    )
        {
          m_view_valid = false;  // set to true at end
          m_start = start;
          if(start==nullptr) return; //if the start node is not available, then the view is not valid

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
						BorderBased::BASFollowNavigation bas(start,borderSet,borderCost,lookahead_distance+s0_border);
            m_rightBorders.append(&bas);
					}
					else
					{
            BASFollowStraight bas(start, borderSet, lookahead_distance + s0_border);
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
          //180919NiM double preview_speed = hasSpeedLimit(s0_raw) ? getSpeedLimit(s0_raw) : 50.0 / 3.6;
          double preview_speed = 50.0/3.6;
          double review_distance = (std::min)(walkback_distance, preview_speed * m_planning_time / 3.0);
          double preview_distance = preview_speed * m_planning_time;
          m_s_min = (std::max)(s0_raw - review_distance, (std::min)(s0_raw, m_centerRaw_fct.limitLo() + 5.0));
          m_s_max = (std::min)(s0_raw + preview_distance, m_centerRaw_fct.limitHi() - 5.0);
          if (m_s_max - m_s_min < m_min_view_distance)
            return;

          /* sample the raw centerline at grid points */
          adore::mad::linspace(m_s_min, m_s_max, Sbuf, PolyFitPoints);
          for (int i = 0; i < PolyFitPoints; i++)
          {
            Xbuf[i] = m_centerRaw_fct.fi(Sbuf[i], 0);
            Ybuf[i] = m_centerRaw_fct.fi(Sbuf[i], 1);
            Wbuf[i] = 1.0;
          }

          /* smooth centerline - compute piecewise poly regression */
          m_cp_fit_X.fit(Sbuf, Xbuf, Wbuf, smoothness);
          m_cp_fit_Y.fit(Sbuf, Ybuf, Wbuf, smoothness);

          /* sample smoothed centerline and evaluate its curvature and curv-derivative at grid points*/
          adore::mad::linspace(m_s_min, m_s_max, Sbuf, PolyEvaluatePoints);
          m_cp_fit_X.evaluate_ordered(PolyEvaluatePoints, Sbuf, Xbuf, dXbuf, ddXbuf, dddXbuf);
          m_cp_fit_Y.evaluate_ordered(PolyEvaluatePoints, Sbuf, Ybuf, dYbuf, ddYbuf, dddYbuf);
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
          double max_intersect_distance = 10.0;
          bool extend_fringes = false;  // true - extend fringes of border in order to guarantee 
                                        //        intersection with smoothed centerline normals

          // left
          s_intersect = m_leftBorder_fct.limitLo();     // initialize at border's beginning
          for (int i = 0; i < PolyEvaluatePoints; i++)  // step through smoothed centerline points and compute distance
          {
            m_leftBorder_fct.getNextIntersectionWithVector2d(s_intersect, Xbuf[i], Ybuf[i], nXbuf[i], nYbuf[i], s_intersect,
                                                            distance, max_intersect_distance, extend_fringes);
            m_leftDistance_fct.getData()(0, i) = Sbuf[i];
            m_leftDistance_fct.getData()(1, i) = distance;
          }
          // right
          s_intersect = m_rightBorder_fct.limitLo();    // initialize at border's beginning
          for (int i = 0; i < PolyEvaluatePoints; i++)  // step through smoothed centerline points and compute distance
          {
            m_rightBorder_fct.getNextIntersectionWithVector2d(s_intersect, Xbuf[i], Ybuf[i], -nXbuf[i], -nYbuf[i],
                                                              s_intersect, distance, max_intersect_distance, extend_fringes);
            m_rightDistance_fct.getData()(0, i) = Sbuf[i];
            m_rightDistance_fct.getData()(1, i) = -distance;
          }

          m_view_valid = true;
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
          s = m_centerSmoothed_fct.getClosestParameter(Xg, Yg, 1, 2, n) - m_s_min;  // compute parameter and lateral deviation
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
          auto c = getCenterline()->f(adore::mad::bound(getCenterline()->limitLo(), s , getCenterline()->limitHi()));
          auto offset = getCenterlineNormal()->f(
                              adore::mad::bound(getCenterlineNormal()->limitLo(), 
                                              s , 
                                              getCenterlineNormal()->limitHi())) * n;
          Xg = c(0) + offset(0);
          Yg = c(1) + offset(1);
          Zg = 0.0;
        }
      };
    }  // namespace BorderBased
  }  // namespace env
}  // namespace adore