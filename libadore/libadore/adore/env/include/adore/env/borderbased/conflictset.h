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

#pragma once
#include <vector>
#include <adore/view/conflictset.h>
#include <adore/env/borderbased/border.h>
#include <adore/env/borderbased/lanefollowinggeometry.h>

namespace adore
{
namespace env
{
namespace BorderBased
{
/**
 * ConflictZone - area of overlapping lanes, where the lane containing the conflict zone has lower priority than other lanes.
 * A conflict zone may not be entered by AV at the same time as higher-priority cross traffic will enter the zone.
 * A conflict zone is an abstraction for conflicts occuring at lane merges, roundabouts, pedestrian/zebra crossings, crossings with yield sign, etc.
 */
class ConflictZone : public adore::view::ConflictZone
{
private:
  std::vector<Border *> pathOfCrossTraffic_; // vector which contains the borders that lead to the conflict zone
  std::vector<Coordinate> coordinates_;      // vecotr of coordinates of the intersection points
  double firstBorderDistToExit_;                      // distance from the end of first Border in pathOfCrossTraffic_ to the end of the conflict zone
  double firstBorderDistToEntr_;                  // distance from the end of first Border in pathOfCrossTraffic_ to the beginning of the conflict zone
  bool distancesCalculated_;                 // flag whether exitDistance_ and entryDistance_ are already calculated

public:
  ConflictZone()
  {
    distancesCalculated_ = false;
  }
  void setPathOfCrossTraffic(std::vector<Border *> poct)
  {
    pathOfCrossTraffic_.clear();
    pathOfCrossTraffic_ = poct;
  }
  std::vector<Border *> *getPathOfCrossTraffic()
  {
    return &pathOfCrossTraffic_;
  }
  void setCoordinates(std::vector<Coordinate> c)
  {
    coordinates_ = c;
  }
  void calcDistances(BorderSet *bs)
  {
    // exit distance
    firstBorderDistToExit_ = std::numeric_limits<double>::min();
    double s, n;
    std::map<double, Border *> possibleEntrances;
    for (auto &c : coordinates_)
    {
      firstBorderDistToExit_ = std::max(firstBorderDistToExit_, (*pathOfCrossTraffic_.begin())->m_path->getClosestParameter(c.m_X, c.m_Y, 1, 2, n));
    }
    firstBorderDistToExit_ = (*pathOfCrossTraffic_.begin())->m_path->limitHi() - firstBorderDistToExit_;

    // entry distance
    firstBorderDistToEntr_ = std::numeric_limits<double>::min();

    for (auto &c : coordinates_)
    {
      for (auto &b : pathOfCrossTraffic_)
      {
        b->m_path->getPositionOfPoint(c.m_X, c.m_Y, 1, 2, s, n);
        if (s < 0 || s > 1)
        {
          continue;
        }
        possibleEntrances.insert(std::make_pair(b->m_path->getClosestParameter(c.m_X, c.m_Y, 1, 2, n), b));
        break;
      }
    }
    std::vector<double> dist;
    double smax;
    for (auto &p : possibleEntrances)
    {
      auto cpPath = pathOfCrossTraffic_;
      dist.clear();
      bs->getDistancesBetweenBordersAlongSuccessors(p.second, *cpPath.begin(), &cpPath, dist);
      if (!dist.empty())
      {
        smax = *std::max_element(dist.begin(), dist.end());
      }
      else
      {
        smax = 0;
      }
      if (*pathOfCrossTraffic_.begin() != p.second)
      {
        smax += p.second->m_path->limitHi();
      }
      firstBorderDistToEntr_ = std::max(firstBorderDistToEntr_, (*pathOfCrossTraffic_.begin())->m_path->limitHi() + smax - p.first);
    }
    distancesCalculated_ = true;
  }
  double getFirstBorderExitDistance() // todo rename this
  {
    return distancesCalculated_ ? firstBorderDistToExit_ : 0.0;
  }
  double getFirstBorderEntranceDistance() // todo rename this
  {
    return distancesCalculated_ ? firstBorderDistToEntr_ : 0.0;
  }
};
class ConflictArea : public adore::view::ConflictArea
{
private:
  std::vector<ConflictZone *> cz_; // vector that contains ConflictZone objects

public:
  std::vector<ConflictZone *> *getBorderBasedConflictZones() // todo add getConflictZones() override for vector<view::conflictzones>
  {
    return &cz_;
  }
  virtual std::vector<view::ConflictZone *> getConflictZones() const override // todo add getConflictZones() override for vector<view::conflictzones>
  {
    std::vector<view::ConflictZone *> acz;
    acz.assign(cz_.begin(), cz_.end());
    return acz;
  }
  void destroy()
  {
    for (auto p: cz_)
    {
      delete p;
    }
    cz_.clear();
  }
  void addConflictZone(ConflictZone *cz)
  {
    cz_.push_back(cz);
  }
  virtual unsigned int getNumberOfConflictZones() const override
  {
    return cz_.size();
  }
  /**
  * getStartProgress - returns progress at which conflict zone starts
  */
  virtual double getStartProgress() const override
  {
    double res = std::numeric_limits<double>::max();
    for (auto &cz : cz_)
    {
      res = std::min(res, cz->getStartProgress());
    }
    return res;
  }
  /**
  * getEndProgress - returns progress at which conflict zone ends. 
  */
  virtual double getEndProgress() const override
  {
    double res = std::numeric_limits<double>::min();
    for (auto &cz : cz_)
    {
      res = std::max(res, cz->getEndProgress());
    }
    return res;
  }
  /**
   * @brief 
   * 
   * @return true if all conflict zones within area have end progress
   * @return false 
   */
  virtual bool hasEndProgress() const override
  {
    bool hep = true;
    for (auto &cz : cz_)
    {
      hep = hep && cz->hasEndProgress();
    }
    return hep;
  }
  virtual double getWaitingPosition() const override
  {
    double res = std::numeric_limits<double>::max();
    for (auto &cz : cz_)
    {
      res = std::min(res, cz->getWaitingPosition());
    }
    return res;
  }
};

class ConflictSet : public adore::view::ConflictSet
{
private:
  LaneFollowingGeometry<20, 200> *lfg_; //lane following geometry
  std::vector<ConflictArea *> ca_;      // vector which contains all known conflict areas for a lane
  std::vector<Coordinate> corner_points_vec;                        // vector of corner points of the overlaps, e. g. for plotting
  std::vector<BorderOverlapSet> overlap_sets;                       // vector of grouped border overlaps
  std::vector<BorderOverlap> overlapping_borders;                   // vector of border overlaps
  std::vector<Border *> right_borders_of_conf_lanes;                // vector of right borders
  bool valid_;                                                      // flag indicates whether conflictset is valid

public:
  ConflictSet(LaneFollowingGeometry<20, 200> *lfg) { lfg_ = lfg; }
  virtual std::vector<view::ConflictArea *> getConflictAreas() const override
  {
    std::vector<view::ConflictArea *> aca;
    aca.assign(ca_.begin(), ca_.end());
    return aca;
  }
  virtual bool isValid() const override
  {
    return valid_;
  }
  void update(BorderSet *borderSet, PrecedenceSet *precedenceSet, adore::env::traffic::EgoLaneTraffic *elt, BAContainer *rightBorders, BAContainer *leftBorders)
  {
    valid_ = false;
    double evaluation_distance_from_intersection_point = 100.0;
    // reset the ConflictSet
    for (auto p : ca_)
    {
      p->destroy();
      delete p;
    }
    ca_.clear();
    // get borders that overlap with the ego lane
    overlapping_borders.clear();
    borderSet->getOverlappingBorders(
        *rightBorders, overlapping_borders);

    /*
     * Iterate through BorderOverlaps. Determine path of potentially conflicting vehicles
     */
    // TODO investigate if not using those variables is a bug or if they can be fully removed
    // double s_max_all = std::numeric_limits<double>::min(); // -Wunused-variable
    // double s_min_all = std::numeric_limits<double>::max(); // -Wunused-variable
    corner_points_vec.clear();
    right_borders_of_conf_lanes.clear();
    overlap_sets.clear();
    for (auto overlap = overlapping_borders.begin();
         overlap != overlapping_borders.end(); ++overlap)
    {
      right_borders_of_conf_lanes.push_back(overlap->m_second_right);
      //auto corner_points = overlap->getCornerPoints();
      auto cp = overlap->getCornerPointVector();
      for (auto asdf = cp->begin(); asdf != cp->end(); ++asdf)
      {
        corner_points_vec.push_back(
            Coordinate(asdf->m_X, asdf->m_Y, asdf->m_Z));
      }
    }
    for (auto overlap = overlapping_borders.begin();
         overlap != overlapping_borders.end(); ++overlap)
    {
      std::vector<Border *> path_of_conflicting_veh;
      double length = 0;
      borderSet->getAllPredecessorsUpToDistance(
          overlap->m_second_right,
          evaluation_distance_from_intersection_point - length,
          path_of_conflicting_veh, true, true);
      // filter paths that are equivalent to lane following path
      auto all_lf_borders = *rightBorders;
      bool skip_this_path = false;
      for (auto cborder : path_of_conflicting_veh)
      {
        auto split_neighbor_check_it =
            std::find(all_lf_borders.begin(), all_lf_borders.end(), cborder);
        if (split_neighbor_check_it != all_lf_borders.end())
        {
          skip_this_path = true;
          break;
        }
      }
      if (skip_this_path)
        continue;

      /**
       * group border overlaps and build border overlap sets
       */
      bool new_overlap_set = true; // form a new overlapset if the overlap can't be added to existing overlapset
      for (auto &ols : overlap_sets)
      {
        if (std::find(ols.conflictingBorders_.begin(),
                      ols.conflictingBorders_.end(), overlap->m_second_right) !=
            ols.conflictingBorders_.end())
        {
          // overlap is added to overlapset that contains the the first conflicting border of the overlap
          ols.borderoverlaps_.push_back(&(*overlap));
          new_overlap_set = false;
          break;
        }
        if (std::find_first_of(path_of_conflicting_veh.begin(),
                               path_of_conflicting_veh.end(),
                               ols.conflictingBorders_.begin(),
                               ols.conflictingBorders_.end()) !=
            path_of_conflicting_veh.end())
        {
          // overlap is added to overlapset that contains the any conflicting border of the overlap
          bool update_conflicting_path = false;
          ols.borderoverlaps_.push_back(&(*overlap));
          for (auto cborder : path_of_conflicting_veh)
          {
            for (auto oborder : right_borders_of_conf_lanes)
            {
              if (cborder == oborder && std::find(ols.conflictingBorders_.begin(), ols.conflictingBorders_.end(), cborder) == ols.conflictingBorders_.end())
              {
                // update the conflicting path and conflicting borders of overlap set
                ols.conflictingBorders_.push_back(cborder);
                update_conflicting_path = true;
              }
            }
          }
          if(update_conflicting_path)
          {
            ols.conflictingPath_ = path_of_conflicting_veh;
          }
          new_overlap_set = false;
          break;
        }
      }
      if (new_overlap_set)
      {

        BorderOverlapSet bos;
        bos.borderoverlaps_.push_back(&(*overlap));
        // add conjunction of boderoverlaps' path and
        // right_borders_of_conf_lanes
        
        for (auto cborder : path_of_conflicting_veh)
        {
          for (auto oborder : right_borders_of_conf_lanes)
          {
            if (cborder == oborder)
              bos.conflictingBorders_.push_back(cborder);
          }
        }
        bos.conflictingPath_ = path_of_conflicting_veh;
        overlap_sets.push_back(bos);
      }
    }
    /** 
     * filter overlap sets that result from intersections between parralell lines
    */
    for (auto ols = overlap_sets.begin(); ols != overlap_sets.end();)
    {
      bool delete_this_ols = false;
      bool left_left_intersection = false;
      bool right_left_intersection = false;
      bool left_right_intersection = false;
      bool right_right_intersection = false;
      for (auto &ol : ols->borderoverlaps_)
      {
        left_left_intersection = left_left_intersection || ol->m_has_intersection_left_left;
        right_left_intersection = right_left_intersection || ol->m_has_intersection_right_left;
        left_right_intersection = left_right_intersection || ol->m_has_intersection_left_right;
        right_right_intersection = right_right_intersection || ol->m_has_intersection_right_right;
      }
      if (left_left_intersection + right_left_intersection + left_right_intersection + right_right_intersection == 1)
      {
        double n = 0;
        double n_new = 0;
        double s = 0;
        if (left_left_intersection)
        {
          for (auto &ol : ols->borderoverlaps_)
          {
            auto c = ol->m_second_left->m_path->f(ol->m_second_left->m_path->limitHi());
            lfg_->toRelativeCoordinates(c(0), c(1), s, n_new);
            n_new = lfg_->getOffsetOfLeftBorder(s) - n_new;
            n = std::max(n, n_new);
            c = ol->m_second_left->m_path->f(0);
            lfg_->toRelativeCoordinates(c(0), c(1), s, n_new);
            n_new = lfg_->getOffsetOfLeftBorder(s) - n_new;
            n = std::max(n, n_new);
          }
        }
        if (left_right_intersection)
        {
          for (auto &ol : ols->borderoverlaps_)
          {
            auto c = ol->m_second_right->m_path->f(ol->m_second_right->m_path->limitHi());
            lfg_->toRelativeCoordinates(c(0), c(1), s, n_new);
            n_new = lfg_->getOffsetOfLeftBorder(s) - n_new;
            n = std::max(n, n_new);
            c = ol->m_second_right->m_path->f(0);
            lfg_->toRelativeCoordinates(c(0), c(1), s, n_new);
            n_new = lfg_->getOffsetOfLeftBorder(s) - n_new;
            n = std::max(n, n_new);
          }
        }
        if (right_left_intersection)
        {
          for (auto &ol : ols->borderoverlaps_)
          {
            auto c = ol->m_second_left->m_path->f(ol->m_second_left->m_path->limitHi());
            lfg_->toRelativeCoordinates(c(0), c(1), s, n_new);
            n_new = -lfg_->getOffsetOfRightBorder(s) + n_new;
            n = std::max(n, n_new);
            c = ol->m_second_left->m_path->f(0);
            lfg_->toRelativeCoordinates(c(0), c(1), s, n_new);
            n_new = -lfg_->getOffsetOfRightBorder(s) + n_new;
            n = std::max(n, n_new);
          }
        }
        if (right_right_intersection)
        {
          for (auto &ol : ols->borderoverlaps_)
          {
            auto c = ol->m_second_right->m_path->f(ol->m_second_right->m_path->limitHi());
            lfg_->toRelativeCoordinates(c(0), c(1), s, n_new);
            n_new = -lfg_->getOffsetOfRightBorder(s) + n_new;
            n = std::max(n, n_new);
            c = ol->m_second_right->m_path->f(0);
            lfg_->toRelativeCoordinates(c(0), c(1), s, n_new);
            n_new = -lfg_->getOffsetOfRightBorder(s) + n_new;
            n = std::max(n, n_new);
          }
        }
        if (n < 0.2)
        {
          //std::cout << "erase conflicting lane"<<std::endl;
          delete_this_ols = true;
        }
      }
      if (delete_this_ols)
      {
        ols = overlap_sets.erase(ols);
      }
      else
      {
        ++ols;
      }
    }
    // form a conflict zone for each borderoverlapset
    assembleConflictZones(precedenceSet, borderSet, rightBorders, leftBorders);
    mapParticipantsToConflictZones(elt, borderSet);
    valid_ = true;
  }
  void assembleConflictZones(PrecedenceSet *precedenceSet, BorderSet *borderSet, BAContainer *rightBorders, BAContainer *leftBorders)
  {
    std::vector<Coordinate> coordinates;
    std::multimap<double, ConflictZone *> conflictzones;
    double s, n;
    // look for segments of unary priority rules for ego vehicle
    std::vector<std::pair<double, double>> egoHasPriority; // pairs in vector represent segments in s coordinates in which ego vehicle has priority over all other
    auto rules = precedenceSet->getAllRules();
    for (auto rule= rules->begin();rule!=rules->end();++rule)
    {
      auto pr = rule->second;
      if (!pr->unary_)
        continue;
      auto highFrom = pr->high_.from_;
      auto highFromSet = borderSet->getBordersAtPoint(highFrom(0), highFrom(1));
      if (std::find_first_of(highFromSet.begin(), highFromSet.end(), rightBorders->begin(), rightBorders->end()) != highFromSet.end())
      {
        auto highTo = pr->high_.to_;
        auto highToSet = borderSet->getBordersAtPoint(highTo(0), highTo(1));
        if (std::find_first_of(highToSet.begin(), highToSet.end(), rightBorders->begin(), rightBorders->end()) != highToSet.end())
        {
          double s_from,s_to,n;
          lfg_->toRelativeCoordinates(highFrom(0),highFrom(1),s_from,n);
          lfg_->toRelativeCoordinates(highTo(0),highTo(1),s_to,n);
          egoHasPriority.push_back(std::make_pair(s_from,s_to));
        }
      }
    }


      for (auto &os : overlap_sets)
      {
        coordinates.clear();
        os.getCoordinates(coordinates);
        // determine s_min, s_max
        double s_min = std::numeric_limits<double>::max();
        double s_max = std::numeric_limits<double>::min();
        double x_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::min();
        double y_min = std::numeric_limits<double>::max();
        double y_max = std::numeric_limits<double>::min();
        for (auto c : coordinates)
        {
          lfg_->toRelativeCoordinates(c.m_X, c.m_Y, s, n);
          s_min = std::min(s_min, s);
          s_max = std::max(s_max, s);
          x_min = std::min(x_min, c.m_X);
          x_max = std::max(x_max, c.m_X);
          y_min = std::min(y_min, c.m_Y);
          y_max = std::max(y_max, c.m_Y);
        }
        env::BorderBased::ConflictZone *cz = new env::BorderBased::ConflictZone();
        cz->setStartProgress(s_min);
        cz->setEndProgress(s_max);
        cz->setWaitingPosition(s_min);
        cz->setCoordinates(coordinates);
        cz->setPathOfCrossTraffic(os.conflictingPath_);
        // check whether cz has end
        
        double x_end = 0.5*((*os.conflictingPath_.begin())->m_id.m_last.m_X + (*os.conflictingPath_.begin())->m_left->m_last.m_X); 
        double y_end = 0.5*((*os.conflictingPath_.begin())->m_id.m_last.m_Y + (*os.conflictingPath_.begin())->m_left->m_last.m_Y); 
        auto end_border=borderSet->getBordersAtPoint(x_end,y_end);
        if (std::find_first_of(end_border.begin(),end_border.end(),rightBorders->begin(),rightBorders->end())!=end_border.end())
        {
          cz->setHasEndProgress(false);
        }
        else
        {
          cz->setHasEndProgress(true);
        }
        for (auto &prioritysegment:egoHasPriority)
        {
          if (s_min > prioritysegment.first && s_max < prioritysegment.second)
          {
            cz->setPriority(false); // conflict zone is within a segment in which ego has right of way
            break;
          }
        }
        if (cz->hasPriority())
        {
          auto ruleitpair = precedenceSet->getRulesInRegion(x_min, y_min, x_max, y_max);
          for (; ruleitpair.first != ruleitpair.second; ++ruleitpair.first)
          {
            auto pr = (*ruleitpair.first).second;
            if (pr->unary_)
              continue;
            auto from = pr->low_.from_;
            auto fromSet = borderSet->getBordersAtPoint(from(0), from(1));
            if (std::find_first_of(fromSet.begin(),
                                   fromSet.end(),
                                   os.conflictingPath_.begin(),
                                   os.conflictingPath_.end()) != fromSet.end())
            {
              std::cout << "  found from point in conflicting path"<<std::endl;
              // check whether ego lane is high route
              auto highFrom = pr->high_.from_;
              auto highFromSet = borderSet->getBordersAtPoint(highFrom(0), highFrom(1));
              if (std::find_first_of(highFromSet.begin(), highFromSet.end(), rightBorders->begin(), rightBorders->end()) != highFromSet.end())
              {
                auto highTo = pr->high_.to_;
                auto highToSet = borderSet->getBordersAtPoint(highTo(0), highTo(1));
                if (std::find_first_of(highToSet.begin(), highToSet.end(), rightBorders->begin(), rightBorders->end()) != highToSet.end())
                {
                  cz->setPriority(false);
                  break;
                }
              }
            }
            // pr->low_.to_; // TODO commented out to fix -Wunused-value, bring back in when needed
            //TODO check to_-Point of priority route with successors of first border in cz->getPathOfCrossTraffic_
          }
        }
        conflictzones.insert(std::make_pair(s_min, cz));
      }
      // add conflictzones to conflict area
      for (auto cz = conflictzones.begin(); cz != conflictzones.end(); ++cz)
      {
        //if (!ca_.empty())
        //std::cout << "cz.first="<<cz->first<< "  caback="<<ca_.back()->getEndProgress()<<std::endl;
        if (ca_.empty() || cz->first - ca_.back()->getEndProgress() > 5)
        {
          ConflictArea *ca = new ConflictArea();
          ca->addConflictZone(cz->second);
          ca_.push_back(ca);
          continue;
        }
        ca_.back()->addConflictZone(cz->second);
      }
    }
    void mapParticipantsToConflictZones(adore::env::traffic::EgoLaneTraffic * elt, BorderSet * borderSet)
    {
      // get the traffic participants
      auto trafficMap = elt->getTrafficMap();
      for (auto &ca : ca_)
      {
        auto czs = ca->getBorderBasedConflictZones();
        for (adore::env::BorderBased::ConflictZone *&cz : *czs)
        {
          auto cp = cz->getPathOfCrossTraffic();
          double length = 0.0;
          for (auto &b : *cp)
          {
            length += b->m_path->limitHi();
          }
          cz->calcDistances(borderSet);
          auto queue = cz->getCrossTraffic();
          queue->clear();
          std::unordered_set<env::traffic::Participant::TTrackingID> idset;
          for (auto &b : *cp)
          {
            for (auto it = trafficMap->getBorderToParticipant().find(b->m_id);
                 it != trafficMap->getBorderToParticipant().end();
                 it++)
            {
              if (!it->second.first.m_isReferenceInside)
                continue;
              std::vector<double> distances;
              auto cpcopy = *cp;
              borderSet->getDistancesBetweenBordersAlongSuccessors(
                  borderSet->getBorder(it->first), *cpcopy.begin(), &cpcopy, distances);
              double smin = 0;
              if (!distances.empty())
              {
                smin = *std::min_element(distances.begin(), distances.end());
              }
              auto progress = borderSet->getBorder(it->first)->m_path->limitHi() - it->second.first.s + smin + (*cp->begin())->m_path->limitHi(); 
              if (borderSet->getBorder(it->first) == *cp->begin())
                progress -= (*cp->begin())->m_path->limitHi();
              env::traffic::Participant::TTrackingID id = it->second.second;
              if (idset.find(id) != idset.end())
                continue; //don't duplicate
              auto it2 = trafficMap->getTrackingIDToParticipant().find(id);
              if (it2 == trafficMap->getTrackingIDToParticipant().end())
                continue;
              idset.emplace(id);
              const env::traffic::Participant &par = it2->second;
              //@TODO: apply prediction/classification module to determine whether participant is moving
              // with traffic flow or should be rated as cross traffic. Also determine potential exit time
              // if participant is changing lanes, enter a prediction of exit time
              // check neighboring lanes for entering vehicles

              // TODO investigate if the following unused variables are a bug
              // auto &center = par.center_;
              // double s, n; // -Wunused-variable 
              adore::view::TrafficObject object;
              object.setCurrentProgress(length - progress);
              // note: vehicle length is not considered for entrance and exit progress
              object.setEntranceProgress(length - cz->getFirstBorderEntranceDistance());
              object.setExitProgress(length - cz->getFirstBorderExitDistance());
              object.setObservationTime(par.observation_time_);
              object.setExitTime(par.observation_time_ + 1.0e5);
              object.setTrackingID(par.trackingID_);
              object.setLength(par.length_);
              object.setCurrentAcceleration(par.getAx());
              object.setCurrentSpeed(par.getVx());
              queue->push_back(object);
            }
          }
        }
      }
    }


    std::vector<Coordinate> *getCornerPoints() { return &corner_points_vec; }
    std::vector<BorderOverlapSet> *getOverlapSet() { return &overlap_sets; }
    std::vector<Border *> *get_right_borders_of_conf_lanes()
    {
      return &right_borders_of_conf_lanes;
    }
    void getBordersToPrint(std::vector<Border *> & result, std::vector<int> & ids)
    {
      int id = 0;
      for (auto &ca : ca_)
      {
        auto czs = ca->getBorderBasedConflictZones();
        for (auto &cz : *czs)
        {
          auto ctp = cz->getPathOfCrossTraffic();
          bool first = true;
          for (auto &b : *ctp)
          {
            result.push_back(b);
            if (first)
              ids.push_back(id);
            ++id;
            first = false;
          }
          // result.insert(result.begin(),ctp->begin(),ctp->end())
        }
      }
    }
    /*
    void printCSinfo()
    {
      std::cout << std::endl
                << "Conflict set information:" << std::endl
                << "There is/are " << ca_.size() << " conflict area(s)." << std::endl;
      for (auto &ca : ca_)
      {
        double start;
        double end;
        start = ca->getStartProgress();
        end = ca->getEndProgress();
        std::cout << "  conflict area with " << ca->getNumberOfConflictZones() << " conflictzones." << std::endl;
        std::cout << "    s_start = " << start << "  s_end = " << end << std::endl;
        auto czs = ca->getBorderBasedConflictZones();
        std::cout << "    num of cz " << czs->size() << std::endl;
        int asdf = 0;

        for (auto &cz : *czs)
        {
          auto ct = cz->getCrossTraffic();
          std::cout << "    cz: cross traffic = " << ct->size() << " priority" << cz->hasPriority()<<std::endl;
          if (ct->size() != 0)
          {
            for (auto &i : *ct)
            {

              std::cout << "      entryProgress = " << i.getEntranceProgress() << " exitProgress = " << i.getExitProgress() << " currentProgress = " << i.getCurrentProgress() << std::endl;
            }
          }
        }
      }
    }*/
  };
} // namespace BorderBased
} // namespace BorderBased
} // namespace env