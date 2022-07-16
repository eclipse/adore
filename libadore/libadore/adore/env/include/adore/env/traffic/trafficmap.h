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
 *    Daniel Heß - initial implementation and API
 ********************************************************************************/


#pragma once

#include <unordered_map>
#include <adore/env/borderbased/borderset.h>
#include <adore/env/borderbased/lanematchingstrategy.h>
#include <adore/env/afactory.h>
#include "participant.h"


namespace adore
{
    namespace env
    {
        namespace traffic
        {
            /**
             * Class to represent traffic data
             * 
             */
            class TrafficMap
            {
            public:
                typedef std::unordered_multimap<adore::env::BorderBased::BorderID,std::pair<adore::env::BorderBased::BorderPositioning<4>,adore::env::traffic::Participant::TTrackingID>,adore::env::BorderBased::BorderIDHasher> TBorderToParticipant;
                typedef std::unordered_multimap<adore::env::traffic::Participant::TTrackingID,std::pair<adore::env::BorderBased::BorderID,adore::env::BorderBased::BorderPositioning<4>>> TParticipantToBorder;
                typedef std::unordered_multimap<adore::env::traffic::Participant::TTrackingID,adore::env::traffic::Participant> TTrackingIDToParticipant;
            private:
                TBorderToParticipant borderToParticipant_; /**< Borders mapped to participants */
                TParticipantToBorder participantToBorder_; /**< Participants mapped to borders */
                TTrackingIDToParticipant trackingIDToParticipant_; /**< TrackingIDs mapped to participants */
                TParticipantSet participantSet_; /**< set of participants */
                adore::env::BorderBased::BorderSet* borderSet_; /**< set of borders */
                adore::env::AFactory::TParticipantSetReader* tpsetReader_; /**< reader of traffic participants */
            public:
                /**
                 * @brief Get the set of traffic participants
                 * 
                 * @return TParticipantSet&
                 */
                TParticipantSet& getTrafficParticipantSet()
                {
                    return participantSet_;
                }
                /**
                 * @brief Get the border set
                 * 
                 * @return adore::env::BorderBased::BorderSet*
                 */
                adore::env::BorderBased::BorderSet* getBorderSet()
                {
                    return borderSet_;
                }
                /**
                 * @brief Get the border to participant map
                 * 
                 * @return const TBorderToParticipant&
                 */
                const TBorderToParticipant& getBorderToParticipant()const
                {
                    return borderToParticipant_;
                }
                /**
                 * @brief Get the participant to border map
                 * 
                 * @return const TParticipantToBorder&
                 */
                const TParticipantToBorder& getParticipantToBorder()const
                {
                    return participantToBorder_;
                }
                /**
                 * @brief Get the tracking id to participant map
                 * 
                 * @return const TTrackingIDToParticipant&
                 */
                const TTrackingIDToParticipant& getTrackingIDToParticipant()const
                {
                    return trackingIDToParticipant_;
                }
                /**
                 * @brief Construct a new TrafficMap object
                 * 
                 * @param borderSet set of borders
                 */
                TrafficMap(adore::env::BorderBased::BorderSet* borderSet)
                {
                    tpsetReader_ = EnvFactoryInstance::get()->getTrafficParticipantSetReader();
                    borderSet_ = borderSet;
                }
                /**
                 * @brief Construct a new TrafficMap object
                 * 
                 * @param borderSet set of borders
                 * @param factory 
                 */
                TrafficMap(adore::env::BorderBased::BorderSet* borderSet,adore::env::AFactory* factory)
                {
                    if(factory!=nullptr)
                    {
                        tpsetReader_ = factory->getTrafficParticipantSetReader();
                    }
                    else
                    {
                        tpsetReader_ = nullptr;
                    }
                    borderSet_ = borderSet;
                }
                /**
                 * @brief Update the TrafficMap
                 * 
                 */
                void update()
                {
                    if(tpsetReader_->hasUpdate())
                    {
                        borderToParticipant_.clear();
                        participantToBorder_.clear();
                        trackingIDToParticipant_.clear();
                        tpsetReader_->getData(participantSet_);
                        for(auto& participant:participantSet_)
                        {
                            trackingIDToParticipant_.insert(std::make_pair(participant.trackingID_,participant));
                        }
                        matchBorders();
                    }
                    //implement variant where individual participants are re-matched
                }
                /**
                 * @brief Match traffic to borders
                 * 
                 */
                void matchBorders()
                {
                    double X[4];double Y[4];
                    double ca;
                    double sa;
                    double Xref;
                    double Yref;
                    double a;
                    double b;
                    double w2;
                    double xmin,xmax,ymin,ymax;

                    for(auto& participant:participantSet_)
                    {
                        ca = std::cos(participant.yaw_);
                        sa = std::sin(participant.yaw_);
                        a = participant.length_*0.5;
                        b = -participant.length_*0.5;
                        w2 = participant.width_*0.5;
                        Xref = participant.center_(0);
                        Yref = participant.center_(1);
                        X[0] = Xref+ca*b      -sa*(-w2);
                        Y[0] = Yref+sa*b      +ca*(-w2);
                        X[1] = Xref+ca*a      -sa*(-w2);
                        Y[1] = Yref+sa*a      +ca*(-w2);
                        X[2] = Xref+ca*a      -sa*(+w2);
                        Y[2] = Yref+sa*a      +ca*(+w2);
                        X[3] = Xref+ca*b      -sa*(+w2);
                        Y[3] = Yref+sa*b      +ca*(+w2);
                        xmin = std::min(std::min(X[0],X[1]),std::min(X[2],X[3]));
                        xmax = std::max(std::min(X[0],X[1]),std::max(X[2],X[3]));
                        ymin = std::min(std::min(Y[0],Y[1]),std::min(Y[2],Y[3]));
                        ymax = std::max(std::min(Y[0],Y[1]),std::max(Y[2],Y[3]));
                        adore::env::BorderBased::Border::boost_box pbox(adore::env::BorderBased::Coordinate::boost_point(xmin,ymin),
                                                                      adore::env::BorderBased::Coordinate::boost_point(xmax,ymax));

                        double offset = 5.0;
                        for(auto bit = borderSet_->getBordersInRegion(xmin-offset,xmax+offset,ymin-offset,ymax+offset);
                            bit.first!=bit.second;
                            bit.first++)
                        {
                            auto border = bit.first->second;
                            auto left = borderSet_->getLeftNeighbor(border);
                            if(left!=0)
                            {
                                if(boost::geometry::intersects(border->getBoostBox(left),pbox))
                                {
                                    adore::env::BorderBased::BorderPositioning<4> positioning(border,left,Xref,Yref,X,Y);
                                    if(positioning.anyInside())
                                    {
                                        borderToParticipant_.insert(std::make_pair(border->m_id,std::make_pair(positioning,participant.trackingID_)));
                                        participantToBorder_.insert(std::make_pair(participant.trackingID_,std::make_pair(border->m_id,positioning)));
                                    }
                                }
                            }
                        }
                    }
                }
            };
        }
    }
}