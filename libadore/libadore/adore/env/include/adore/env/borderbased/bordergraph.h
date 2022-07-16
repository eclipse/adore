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

#pragma once
#include <vector>
#include <unordered_map>
#include <queue>
#include <adore/env/borderbased/borderset.h>

namespace adore
{
namespace env
{
namespace BorderBased
{
struct Node
{
public:
  Border* m_border;
  Node* m_predecessor;
  Node* m_successor;
  double m_g;
  double m_h;

public:
  Node(const Node& value)
    : m_border(value.m_border)
    , m_predecessor(value.m_predecessor)
    , m_successor(value.m_successor)
    , m_g(value.m_g)
    , m_h(value.m_h)
  {
  }
  Node() : m_border(0), m_predecessor(0), m_successor(0), m_g(0), m_h(0)
  {
  }
  Node(Border* border) : m_border(border), m_predecessor(0), m_successor(0), m_g(0), m_h(0)
  {
  }
  void setBorder(Border* border)
  {
    m_border = border;
  }
  void setG(double g)
  {
    m_g = g;
  }
  void setH(double h)
  {
    m_h = h;
  }
  void setSuccessor(Node* successor)
  {
    m_successor = successor;
  }
  void setPredecessor(Node* predecessor)
  {
    m_predecessor = predecessor;
  }
  double g() const
  {
    return m_g;
  }
  double h() const
  {
    return m_h;
  }
  double f() const
  {
    return m_g + m_h;
  }
  bool operator<(const Node& other) const
  {
    return this->f() < other.f();
  }
  bool operator>(const Node& other) const
  {
    return this->f() > other.f();
  }
};
struct NodeHasher
{
  BorderIDHasher bh;
  std::size_t operator()(const Node& b) const
  {
    return bh.operator()(b.m_border->m_id);
  }
  std::size_t operator()(const Node* b) const
  {
    return bh.operator()(b->m_border->m_id);
  }
};
/**
 *	ABorderGraphCost - abstract class, which defines a template for a cost metric between two borders
 */
class ABorderGraphCost
{
public:
  virtual ~ABorderGraphCost()
  {
  }
  virtual void assign(Node* n) = 0;
  virtual double getCostGuard(){return 1.0e10;}
  virtual bool isCostBounded(double value){return -1.0e9<value && value<1.0e9;}
};

class SimpleBorderGraphCost : public ABorderGraphCost
{
public:
  virtual void assign(Node* n) override
  {
    if (n->m_predecessor != 0)
    {
      if (n->m_border->isSuccessorOf(n->m_predecessor->m_border->m_id))
      {
        n->setG(n->m_predecessor->g() + n->m_predecessor->m_border->getLength());
      }
      else
      {
        n->setG(n->m_predecessor->g() +
                std::min(n->m_border->m_id.m_first.distance(n->m_predecessor->m_border->m_id.m_first),
                         n->m_border->m_id.m_first.distance(n->m_predecessor->m_border->m_id.m_last)));
      }
    }
    else
    {
      if (n->m_successor != 0)
      {
        if (n->m_border->isPredecessorOf(n->m_successor->m_border->m_id))
        {
          n->setG(n->m_successor->g() - n->m_border->getLength());
        }
        else
        {
          n->setG(n->m_successor->g() -
                  std::min(n->m_border->m_id.m_first.distance(n->m_successor->m_border->m_id.m_first),
                           n->m_border->m_id.m_first.distance(n->m_successor->m_border->m_id.m_last)));
        }
      }
    }
    n->setH(0.0);
  }
};

class BorderGraphCostWithLaneChanges : public ABorderGraphCost
{
private:
  double m_laneChangePenalty;

public:
  BorderGraphCostWithLaneChanges(double laneChangePenalty = 100.0) : m_laneChangePenalty(laneChangePenalty)
  {
  }
  void setLaneChangePenalty(double value)
  {
    m_laneChangePenalty = value;
  }
  virtual void assign(Node* n) override
  {
    n->setH(0.0);
    if (n->m_predecessor != 0)
    {
      n->setG(getCostGuard());
      if (n->m_border->isContinuousSuccessorOf(n->m_predecessor->m_border))
      {
        n->setG(n->m_predecessor->g() + n->m_predecessor->m_border->getLength());
      }
      else
      {
        if (n->m_border->isLaneChangeNeighborOf(n->m_predecessor->m_border))
        {
          n->setG(n->m_predecessor->g() + m_laneChangePenalty);
        }
      }
    }
    else
    {
      if (n->m_successor != 0)
      {
        n->setG(-getCostGuard());
        if (n->m_border->isContinuousPredecessorOf(n->m_successor->m_border))
        {
          n->setG(n->m_successor->g() - n->m_border->getLength());
        }
        else
        {
          if (n->m_border->isLaneChangeNeighborOf(n->m_successor->m_border))
          {
            n->setG(n->m_successor->g() - m_laneChangePenalty);
          }
        }
      }
    }
  }
};

/**
 * A topological graph structure living on top of a geometric Border representation.
 * This class implements forward and backward search for drivable routes along Border objects.
 */
class BorderGraph
{
private:
  BorderSet* m_borderSet;
  ABorderGraphCost* m_cost;

public:
  typedef std::unordered_map<adore::env::BorderBased::BorderID, double, adore::env::BorderBased::BorderIDHasher>
      BorderCostMap;

  BorderGraph()
  {
  }
  BorderGraph(BorderSet* borderSet) : m_borderSet(borderSet)
  {
    m_cost = new SimpleBorderGraphCost();
  }
  BorderGraph(BorderSet* borderSet, ABorderGraphCost* cost) : m_borderSet(borderSet), m_cost(cost)
  {
  }
  virtual ~BorderGraph()
  {
    delete m_cost;
  }
  void init(BorderSet* borderSet)
  {
    m_borderSet = borderSet;
    m_cost = new SimpleBorderGraphCost();
  }
  void init(BorderSet* borderSet, ABorderGraphCost* cost)
  {
    m_borderSet = borderSet;
    m_cost = cost;
  }

  void djikstra(adore::env::BorderBased::Node* startNode, BorderCostMap* closedList)
  {
    std::priority_queue<adore::env::BorderBased::Node, std::vector<adore::env::BorderBased::Node>,
                        std::less<adore::env::BorderBased::Node>>
        openList;
    openList.push(*startNode);
    startNode->setG(0);

    while (!openList.empty())
    {
      auto closingNode = openList.top();
      openList.pop();
      if (closedList->find(closingNode.m_border->m_id) == closedList->end())
      {
        closedList->insert(
            std::pair<adore::env::BorderBased::BorderID, double>(closingNode.m_border->m_id, closingNode.g()));

        auto expansion = getPredecessors(&closingNode);
        while (expansion.hasMore())
        {
          auto newNode = expansion.getNext();

          if (closedList->find(newNode.m_border->m_id) == closedList->end())
          {
            if (newNode.m_border->m_type == adore::env::BorderBased::BorderType::DRIVING)
            {
              if(m_cost->isCostBounded(newNode.g()))
              {
                openList.push(newNode);
              }
            }
          }
        }
      }
    }
  }

  class Expansion
  {
  private:
    itCoordinate2Border m_it;
    Node* m_n;
    ABorderGraphCost* m_cost;
    std::vector<Border*> m_connectedBorders;
    bool m_search_forward;

  public:
    Expansion(Node* n, Border* left, Border* right, itCoordinate2Border& it, ABorderGraphCost* cost,
              bool search_forward, bool allow_direction_inversion = false)
      : m_it(it), m_n(n), m_cost(cost), m_search_forward(search_forward)
    {
      if (left != 0 && (allow_direction_inversion || n->m_border->getNeighborDirection() == Border::SAME_DIRECTION))
      {
        m_connectedBorders.push_back(left);
      }
      if (right != 0 && (allow_direction_inversion || right->getNeighborDirection() == Border::SAME_DIRECTION))
      {
        m_connectedBorders.push_back(right);
      }
      for (; it.first != it.second; it.first++)
      {
        Border* k = it.first->second;
        if(m_search_forward)
        {
          if(m_n->m_border->isContinuousPredecessorOf(k))
          {
            m_connectedBorders.push_back(k);
          }
        } 
        else
        {
          if (m_n->m_border->isContinuousSuccessorOf(k))
          {
            m_connectedBorders.push_back(k);
          }
        }
      }
    }
    bool hasMore()
    {
      return !m_connectedBorders.empty();
    }
    Node getNext()
    {
      Node k;
      k.setBorder(m_connectedBorders.back());
      m_connectedBorders.pop_back();
      if (m_search_forward)
        k.setPredecessor(m_n);
      else
        k.setSuccessor(m_n);
      m_cost->assign(&k);
      return k;
    }
  };
  /**
   *	getPredecessors - returns an Expansion object with backwards search direction, which produces all
   *neighbors/predecessors of a border
   */
  inline Expansion getPredecessors(Node* n, bool left_neighbors = true, bool right_neighbors = true,
                                   bool allow_direction_inversion = false)
  {
    auto predecessors = m_borderSet->getPredecessors(n->m_border);
    return Expansion(n, left_neighbors ? m_borderSet->getLeftNeighbor(n->m_border) : 0,
                     right_neighbors ? m_borderSet->getRightNeighbor(n->m_border) : 0, predecessors, m_cost, false,
                     allow_direction_inversion);
  }
  /**
   *	getSuccessors - returns an Expansion object with forward search direction, which produces all neighbors/successors
   *of a border
   */
  inline Expansion getSuccessors(Node* n, bool left_neighbors = true, bool right_neighbors = true,
                                 bool allow_direction_inversion = false)
  {
    auto successors = m_borderSet->getSuccessors(n->m_border);
    return Expansion(n, left_neighbors ? m_borderSet->getLeftNeighbor(n->m_border) : 0,
                     right_neighbors ? m_borderSet->getRightNeighbor(n->m_border) : 0, successors, m_cost, false,
                     allow_direction_inversion);
  }
};
}  // namespace BorderBased
}  // namespace env
}  // namespace adore