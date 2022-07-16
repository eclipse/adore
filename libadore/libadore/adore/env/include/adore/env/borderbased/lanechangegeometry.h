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
 * 	 Robert Markowski
 *   Matthias Nichting
 *   Thomas Lobig - refactoring
 ********************************************************************************/

#pragma once
// TODO refactor alangechangeview to have LEFT RIGHT enum in a seperate FILE!
#include <adore/view/alanechangeview.h>
#include <adore/mad/adoremath.h>
#include <adore/mad/fun_essentials.h>
#include <unordered_set>
#include <adore/env/borderbased/bordergraph.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/mad/linearfunctiontypedefs.h>
namespace adore
{
namespace env
{
namespace BorderBased
{
/**
       * @brief A class with a geometry description of a lane next to the current lane
       * 
       */
class LaneChangeGeometry
{

public:
  using function_type_xyz = adore::mad::function_type_xyz;
  using function_type2d = adore::mad::function_type2d;
  using function_type_scalar = adore::mad::function_type_scalar;
  using velocity_profile = function_type_scalar;
  typedef std::unordered_set<BorderBased::Node *, BorderBased::NodeHasher> TBorderSubSet;
  function_type_xyz m_leftBorder_fct;   /**< function: s-coordinate -> euclidian coordinates for left borders */
  function_type_xyz m_rightBorder_fct;  /**< function: s-coordinate -> euclidian coordinates for right borders */
  function_type_xyz m_centerBorder_fct; /**< function: s-coordinate -> euclidian coordinates for center borders*/
  function_type_xyz m_reference_fct;
  function_type_scalar m_leftBorderDistance_fct;   /**< function: s-coordinate -> distance to left border */
  function_type_scalar m_rightBorderDistance_fct;  /**< function: s-coordinate -> distance to right border */
  function_type_scalar m_centerBorderDistance_fct; /**< function: s-coordinate -> distance to center border */
  double m_s_lane_width_open;                      /**< s-coordinate where lane starts to have the required width for driving on that lane*/
  double m_s_lane_width_closed;                    /**< s-coordinate where lane ends to have the required width */
  double m_s_viewing_distance;                     /**< s-coordinate of the end of the viewing horizon */
  int m_lfg_adjacency_i0;
  int m_lfg_adjacency_i1;
  double m_adjacency_s0;
  double m_adjacency_s1;
  Border *m_lfg_adjacency_b0;
  Border *m_lfg_adjacency_b1;
  BorderBased::Node *m_lfg_adjacency_bn0;
  BorderBased::Node *m_lfg_adjacency_bn1;
  TBorderSubSet m_borderSubSet;
  BorderBased::BAContainer m_borderSequence;
  BorderBased::BAContainer m_innerBorderSequence;
  adore::view::ALaneChangeView::direction m_direction; /**< direction of lane change view */
  double m_vehicle_width;                              /**< ego vehicle width */
  bool m_view_valid;                                   /**< flag whether view is valid */

public:
  /**
 * @brief Construct a new LaneChangeGeometry object
 * 
 */
  LaneChangeGeometry() : m_leftBorder_fct(1, 0.0), m_rightBorder_fct(1, 0.0), m_view_valid(false)
  {
    m_vehicle_width = 1.8;
  }
  /**
         * @brief Check whether the LaneChangeGeometry is valid
         * 
         * @return true if the LaneChangeGeometry is valid
         * @return false if the LaneChangeGeometry is invalid
         */
  bool isValid() const
  {
    return m_view_valid;
  }
  /**
   * @brief Clear the LaneChangeGeometry
   * 
   */
  void clearGeometry()
  {
    for (auto it = m_borderSubSet.begin(); it != m_borderSubSet.end(); it++)
    {
      delete *it;
    }
    m_borderSubSet.clear();
    m_borderSequence.clear();
    m_innerBorderSequence.clear();
  }
  template <int lfg_a, int lfg_b>
  /**
   * @brief update the road geometry
   * 
   * @param lfg associated LaneFollowingGeometry
   * @param borderSet set of all borders
   * @param ego ego state
   * @param lfg_adjacency_i_start 
   * @param direction direction of the LaneChangeGeometry
   * @param review_distance how far to look backwards
   * @param preview_distance how far to look forwards
   * @param adjacency_lower_limit 
   */
  void update(LaneFollowingGeometry<lfg_a, lfg_b> *lfg, BorderSet *borderSet, adore::env::VehicleMotionState9d *ego,
              int lfg_adjacency_i_start, adore::view::ALaneChangeView::direction direction, double review_distance,
              double preview_distance, double adjacency_lower_limit = 0.0)
  {
    m_view_valid = false;
    m_direction = direction;
    clearGeometry();
    m_lfg_adjacency_b0 = 0;
    m_lfg_adjacency_b1 = 0;
    m_lfg_adjacency_bn0 = 0;
    m_lfg_adjacency_bn1 = 0;

    bool open = false;
    Border *neighbor;
    double s = 0.0; // initialized to silence -Wmaybe-uninitialized warning
    int i = 0;
    for (auto it = lfg->begin(); it != lfg->end(); it++, i++)
    {
      if (i >= lfg_adjacency_i_start)
      {
        if (direction == adore::view::ALaneChangeView::LEFT)
        {
          neighbor = borderSet->getLeftNeighbor(*it);
          if ((*it)->getNeighborDirection() == Border::OPPOSITE_DIRECTION)
          {
            neighbor = 0;
          }
          else if (neighbor != 0 && !borderSet->borderTypeValid(neighbor))
          {
            neighbor = 0;
          }
        }
        else
        {
          neighbor = borderSet->getRightNeighbor(*it);
          if (neighbor != 0 && !borderSet->borderTypeValid(neighbor))
          {
            neighbor = 0;
          }
        }
        if (open)
        {
          // closing found
          if (neighbor == 0 || !m_lfg_adjacency_bn1->m_border->isPredecessorOf(neighbor->m_id))
          {
            m_adjacency_s1 = s;
            open = false;
            // is this the correct adjacency? if it finishes before lower limit, a reset is necessary
            if (m_adjacency_s1 > adjacency_lower_limit)
            {
              break; // break the for loop, adjacency is defined
            }
            else
            {
              // in the following remove previously found opening and replace it with borders of lfv
              m_borderSequence.clear();
              m_borderSequence.insert(m_borderSequence.end(), m_innerBorderSequence.begin(),
                                      m_innerBorderSequence.end());
            }
          }
          // closing not yet found
          else
          {
            m_lfg_adjacency_b1 = *it;
            m_lfg_adjacency_i1 = i;
            BorderBased::Node *node = new BorderBased::Node(neighbor);
            node->setG(s);
            node->setPredecessor(m_lfg_adjacency_bn1);
            m_borderSubSet.emplace(node);
            m_lfg_adjacency_bn1 = node;

            s += neighbor->getLength();
          }
        }
        else
        {
          // find the opening
          if (neighbor != 0)
          {
            bool neighboringLaneExists;
            if (direction == adore::view::ALaneChangeView::LEFT)
            {
              neighboringLaneExists =
                  borderSet->hasLeftNeighbor(neighbor) && neighbor->getNeighborDirection() == Border::SAME_DIRECTION;
            }
            else
            {
              neighboringLaneExists = true;
            }
            if (neighboringLaneExists)
            {
              open = true;
              m_lfg_adjacency_b0 = *it;
              m_lfg_adjacency_b1 = *it;
              m_lfg_adjacency_i0 = i;
              m_lfg_adjacency_i1 = i;

              double n;
              lfg->toRelativeCoordinates(neighbor->m_id.m_first.m_X, neighbor->m_id.m_first.m_Y, m_adjacency_s0, n);
              s = m_adjacency_s0;
              BorderBased::Node *node = new BorderBased::Node(neighbor);
              node->setG(s);
              m_borderSubSet.emplace(node);
              m_lfg_adjacency_bn0 = node;
              m_lfg_adjacency_bn1 = node;

              s += neighbor->getLength();
            }
          }
        }
        // extract the outer border before and during adjacency
        if (open)
        {
          m_borderSequence.push_back(neighbor);
        }
        else
        {
          m_borderSequence.push_back(*it);
        }
      }
      else
      {
        m_borderSequence.push_back(*it);
      }
      m_innerBorderSequence.push_back(*it);
    }
    if (open)
      m_adjacency_s1 = s;
    m_s_viewing_distance = m_adjacency_s1;

    if (m_lfg_adjacency_bn0 != 0 && m_lfg_adjacency_bn1 != 0)
    {
      BASFollowStraight basFollowStraight(m_lfg_adjacency_bn1->m_border, borderSet,
                                          std::max(0.0, preview_distance - s));
      BorderBased::Node *lastNode = m_lfg_adjacency_bn1;
      bool inverted;
      Border *current;
      basFollowStraight.getNextBorder(current, inverted);
      for (basFollowStraight.getNextBorder(current, inverted); current != 0;
           basFollowStraight.getNextBorder(current, inverted))
      {
        m_borderSequence.push_back(current);
        m_innerBorderSequence.push_back(current);

        BorderBased::Node *node = new BorderBased::Node(current);
        node->setG(lastNode->g() + lastNode->m_border->getLength());
        node->setPredecessor(lastNode);
        m_borderSubSet.emplace(node);
        lastNode = node;
        m_s_viewing_distance = node->g() + node->m_border->getLength();
      }

      BorderGraph graph(borderSet);
      std::vector<BorderBased::Node *> open_list;
      open_list.push_back(m_lfg_adjacency_bn0);
      std::unordered_set<Border *> closed_set;

      while (open_list.size() > 0)
      {
        BorderBased::Node *parent = open_list.back();
        open_list.pop_back();
        closed_set.emplace(parent->m_border);
        auto expansion = graph.getPredecessors(parent, false, false);
        while (expansion.hasMore())
        {
          BorderBased::Node *child = new BorderBased::Node(expansion.getNext());
          if (closed_set.find(child->m_border) == closed_set.end() &&
              m_borderSubSet.find(child) == m_borderSubSet.end() &&
              child->m_border->isContinuousPredecessorOf(parent->m_border))
          {
            // remember border for the selection of relevant traffic participants
            m_borderSubSet.emplace(child);
            // explore graph against driving direction, up to review_distance depth
            if (m_adjacency_s0 - child->g() < review_distance)
            {
              open_list.push_back(child);
            }
          }
          else
          {
            delete child;
          }
        }
      }

      BorderAccumulator leftBA, rightBA, centerBorderBA;
      if (direction == adore::view::ALaneChangeView::LEFT)
      {
        BASNeighbor basNeighbor(&m_borderSequence, borderSet, BASNeighbor::LEFT);
        leftBA.append(&basNeighbor);

        for (auto it = m_borderSequence.begin(); it != m_borderSequence.end(); it++)
        {
          centerBorderBA.append(*it, false);
        }

        for (auto it = m_innerBorderSequence.begin(); it != m_innerBorderSequence.end(); it++)
        {
          rightBA.append(*it, false);
        }
      }
      else
      {
        BASNeighbor basNeighbor(&m_innerBorderSequence, borderSet, BASNeighbor::LEFT);
        leftBA.append(&basNeighbor);

        BASNeighbor basNeighbor2(&m_borderSequence, borderSet, BASNeighbor::LEFT);
        centerBorderBA.append(&basNeighbor2);

        for (auto it = m_borderSequence.begin(); it != m_borderSequence.end(); it++)
        {
          rightBA.append(*it, false);
        }
      }

      leftBA.defineFunction(m_leftBorder_fct);
      rightBA.defineFunction(m_rightBorder_fct);
      centerBorderBA.defineFunction(m_centerBorder_fct);
      double max_lane_change_distance = 10.0;

      adore::mad::defineDistanceMap2d(&m_leftBorderDistance_fct, 1, lfg->getCenterline(), lfg->getCenterlineNormal(),
                                      &m_leftBorder_fct, max_lane_change_distance, lfg->getOffsetOfLeftBorderFct());
      adore::mad::defineDistanceMap2d(&m_rightBorderDistance_fct, 1, lfg->getCenterline(), lfg->getCenterlineNormal(),
                                      &m_rightBorder_fct, max_lane_change_distance, lfg->getOffsetOfRightBorderFct());
      if (direction == adore::view::ALaneChangeView::LEFT)
      {
        adore::mad::defineDistanceMap2d(&m_centerBorderDistance_fct, 1, lfg->getCenterline(), lfg->getCenterlineNormal(),
                                        &m_centerBorder_fct, max_lane_change_distance, lfg->getOffsetOfRightBorderFct());
      }
      else
      {
        adore::mad::defineDistanceMap2d(&m_centerBorderDistance_fct, 1, lfg->getCenterline(), lfg->getCenterlineNormal(),
                                        &m_centerBorder_fct, max_lane_change_distance, lfg->getOffsetOfLeftBorderFct());
      }

      bool width_open = false;
      m_s_lane_width_open = 0.0;
      m_s_lane_width_closed = 10000.0;
      auto border =
          (direction == adore::view::ALaneChangeView::LEFT) ? &m_leftBorderDistance_fct : &m_rightBorderDistance_fct;
      auto data = m_centerBorderDistance_fct.getData();
      for (int i = 0; i < data.nc(); i++)
      {
        double d = -(std::abs)(data(1, i)) + (std::abs)(border->fi(data(0, i), 0)) - m_vehicle_width;
        if (width_open)
        {
          if (d < 0)
          {
            m_s_lane_width_closed = data(0, i);
            break;
          }
        }
        else
        {
          if (d > 0)
          {
            m_s_lane_width_open = data(0, i);
            width_open = true;
          }
        }
      }

      m_view_valid = (m_adjacency_s1 - m_adjacency_s0) > 0.0 && (m_s_lane_width_closed - m_s_lane_width_open) > 0.0;
    }
  }

  /**
  * @brief Get the sequence of inner Borders
  * 
  * @return BAContainer* 
  */
  BorderSubSet *getInnerBorders()
  {
    return &m_innerBorderSequence;
  }
  /**
  * @brief Get the sequence of outer Borders
  * 
  * @return BAContainer* 
  */
  BorderSubSet *getOuterBorders()
  {
    return &m_borderSequence;
  }
  /**
  * @brief Get the sequence of inner Borders
  * 
  * @return BAContainer* 
  */
  BorderSubSet *getLeftBorders()
  {
    if(m_direction == adore::view::ALaneChangeView::direction::LEFT)
    return &m_borderSequence;
    return &m_innerBorderSequence;
  }
  /**
  * @brief Get the sequence of inner Borders
  * 
  * @return BAContainer* 
  */
  BorderSubSet *getRightBorders()
  {
    if(m_direction == adore::view::ALaneChangeView::direction::RIGHT)
    return &m_borderSequence;
    return &m_innerBorderSequence;
  }


  /**
   * @brief Get the viewing distance
   * 
   * @return double viewing distance
   */
  double getViewingDistance() const
  {
    return m_s_viewing_distance;
  }
  /**
   * @brief Get the direction of the LaneChangeGeometry
   * 
   * @return adore::view::ALaneChangeView::direction direction of the LaneChangeGeometry
   */
  adore::view::ALaneChangeView::direction getLCDirection() const
  {
    return m_direction;
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
   * @brief Get the offset of the left border at a certain position
   * 
   * @param s s-coordinate of the position
   * @return double offset of the left border at the given position
   */
  double getOffsetOfLeftBorder(double s)
  {
    s=adore::mad::bound(m_leftBorderDistance_fct.limitLo(),m_leftBorderDistance_fct.limitLo()+s,m_leftBorderDistance_fct.limitHi());
    return m_leftBorderDistance_fct(s);
  }
  /**
   * @brief Get the offset of the right border at a certain position
   * 
   * @param s s-coordinate of the position
   * @return double offset of the right border at the given position
   */
  double getOffsetOfRightBorder(double s)
  {
    s=adore::mad::bound(m_rightBorderDistance_fct.limitLo(),m_rightBorderDistance_fct.limitLo()+s,m_rightBorderDistance_fct.limitHi());
    return m_rightBorderDistance_fct(s);
  }
  /**
   * @brief Get the offset of the center border at a certain position
   * 
   * @param s s-coordinate of the position
   * @return double offset of the center border at the given position
   */
  double getOffsetOfCenterBorder(double s)
  {
    return m_centerBorderDistance_fct(s);
  }
  /**
   * @brief Get the s-coordinate where the lane starts to be in direct adjacency to the LaneFollowingGeometry whithout a barrier
   * 
   * @return double s-coordinate where the direct adjacency starts
   */
  double getProgressOfGateOpen() const
  {
    return m_adjacency_s0;
  }
  /**
   * @brief Get the s-coordinate where the lane ends to be in direct adjacency to the LaneFollowingGeometry whithout a barrier
   * 
   * @return double s-coordinate where the direct adjacency ends
   */
  double getProgressOfGateClosed() const
  {
    return m_adjacency_s1;
  }
};
} // namespace BorderBased
} // namespace env
} // namespace adore