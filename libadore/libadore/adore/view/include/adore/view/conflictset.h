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
#include "trafficobject.h"
#include "limitline.h"

namespace adore
{
namespace view
{
/**
         * ConflictZone - area of overlapping lanes, where the lane containing the conflict zone has lower priority than other lanes.
         * A conflict zone may not be entered by AV at the same time as higher-priority cross traffic will enter the zone.
         * A conflict zone is an abstraction for conflicts occuring at lane merges, roundabouts, pedestrian/zebra crossings, crossings with yield sign, etc.
         */
class ConflictZone
{
private:
    double startProgress_;      /**< Progress along ALane at which ConflictZone starts*/
    double endProgress_;        /**< Progress along ALane at which ConflictZone ends*/
    bool hasEndProgress_;       /**< Determines whether the ConflictZone has an immediate end. Conflict zone at an X crossing ends after crossing the conflicting lane. Conflict zone at a Y crossing (merging of two lanes) extends indefinitely.*/
    double waitingPosition_;    /**< Offset from ConflictZone at which to wait for access*/
    bool hasPriority_;          /**< true, if traffic objects in crossTraffic_ have right of way*/
    TrafficQueue crossTraffic_; /**< TrafficQueue of objects approaching and leaving conflict zone. TrafficObject with currentProgress<entranceProgress approaches conflict zone. TrafficObject with entranceProgress<currentProgress<exitProgress is located inside conflict zone. TrafficOjbect with exitProgress<currentProgress has passed along conflict zone*/
    LimitLine limitLine_;       /**< LimitLine associated with ConflictZone.*/
public:
    ConflictZone()
    {
        startProgress_ = std::numeric_limits<double>::max();
        startProgress_ = std::numeric_limits<double>::min();
        hasEndProgress_ = false;
        hasPriority_ = true;
    }
    /**
    * getStartProgress - returns progress at which conflict zone starts
    */
    double getStartProgress() const
    {
        return startProgress_;
    }
    /**
    * getEndProgress - returns progress at which conflict zone ends. 
    */
    double getEndProgress() const
    {
        return endProgress_;
    }
    /**
    * hasEndProgress - returns false, if the conflict zone is unbounded, e.g. in case of a lane merge
    */
    bool hasEndProgress() const
    {
        return hasEndProgress_;
    }
    /**
    * getWaitingPosition - returns progress where AV has to wait, if it cannot enter conflict zone.
    */
    double getWaitingPosition() const
    {
        return waitingPosition_;
    }
    /**
     * hasPriority - returns true, if objects in traffic que have priority
     */
    bool hasPriority() const
    {
        return hasPriority_;
    }
    /*
    * getCrossTraffic - returns a set of traffic objects approaching the conflict zone with higher priority.
    * The AV has to yield to cross traffic, before entering a conflict zone.
    * Cross traffic is ordered by EntranceTime.
    */
    TrafficQueue *getCrossTraffic()
    {
        return &crossTraffic_;
    }
    /**
    *  getLimitLine - returns the associated limit line information.
    */
    const LimitLine &getLimitLine() const
    {
        return limitLine_;
    }

public:
    void setStartProgress(double value)
    {
        startProgress_ = value;
    }
    void setEndProgress(double value)
    {
        endProgress_ = value;
    }
    void setHasEndProgress(bool value)
    {
        hasEndProgress_ = value;
    }
    void setWaitingPosition(double value)
    {
        waitingPosition_ = value;
    }
    /**
     * setPriority - sets the hasPriority_ member
     * @param value true, if objects in crossTraffic_ have priority over ego vehicle
     */
    void setPriority(bool value)
    {
        hasPriority_ = value;
    }
    void setCrossTraffic(TrafficQueue &trafficqueue)
    {
        crossTraffic_ = trafficqueue;
    }
    void setLimitLine(LimitLine &limitline)
    {
        limitLine_ = limitline;
    }
};
class ConflictArea
{
public:

    ConflictArea() = default;
    // TODO investigate if calling delete on derived classes will cause memory leaks
    virtual ~ConflictArea() = default; // -Wdelete-non-virtual-dtor

    virtual unsigned int getNumberOfConflictZones() const = 0;
    
    /**
    * getStartProgress - returns progress at which conflict zone starts
    */
            
    virtual double getStartProgress() const=0;
    
    /**
    * getEndProgress - returns progress at which conflict zone ends. 
    */
    virtual double getEndProgress() const=0;
   
    /**
    * hasEndProgress - returns false, if the conflict zone is unbounded, e.g. in case of a lane merge
    */
    virtual bool hasEndProgress() const=0;
    
    /**
    * getWaitingPosition - returns progress where AV has to wait, if it cannot enter conflict zone.
    */
    virtual double getWaitingPosition() const=0;
    
    /**
     * getConflictZones - return vector of view::ConflictZone objects
     * 
     */
    virtual std::vector<ConflictZone*> getConflictZones() const = 0;
};
/**
         *  ConflictSet - a set of conflict zones, ordered by progress along a lane
         */

class ConflictSet
{
public:
    /**
    * isValid - return true if representation of lane is valid
    */
    virtual bool isValid() const = 0;
    /**
     * getConflictAreas - return vector of view::ConflictArea objects
     * 
     */
    virtual std::vector<ConflictArea*> getConflictAreas() const = 0; // TODO 
};
} // namespace view
} // namespace adore
