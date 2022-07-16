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
 *   Daniel Heß - initial implementation
 ********************************************************************************/

#pragma once

#include <adore/env/afactory.h>
#include <adore/sim/afactory.h>
#include <adore/params/afactory.h>
#include <unordered_map>


namespace adore
{
    namespace apps
    {
        /**
         * @brief A class, which monitors distance between vehicle and goal
         */
        class GoalAttainmentMonitor
        {   
            private:
            adore::env::AFactory::TNavigationGoalReader* goal_reader_;
            adore::env::AFactory::TVehicleMotionStateReader* position_reader_;
            adore::env::AFactory::TPropositionWriter* proposition_writer_;
            public:
            GoalAttainmentMonitor(
                    adore::env::AFactory::TNavigationGoalReader* goal_reader,
                    adore::env::AFactory::TVehicleMotionStateReader* position_reader,
                    adore::env::AFactory::TPropositionWriter* proposition_writer
                    ):
                    goal_reader_(goal_reader),
                    position_reader_(position_reader),
                    proposition_writer_(proposition_writer)
            {}
            void run()
            {
                if(goal_reader_->hasData() && position_reader_->hasData())
                {
                    adore::fun::NavigationGoal nav_goal;
                    adore::env::VehicleMotionState9d pos;
                    goal_reader_->getData(nav_goal);
                    position_reader_->getData(pos);
                    double dx = nav_goal.target_.x_ - pos.getX();
                    double dy = nav_goal.target_.y_ - pos.getY();
                    double dz = nav_goal.target_.z_ - pos.getZ();
                    double d = 50.0;//m
                    adore::env::Proposition p("NEAR_GOAL",dx*dx+dy*dy+dz*dz<d*d);
                    proposition_writer_->write(p);
                }
            }
        };
        /**
         * @brief A class, which monitors for collisions between ego and other objects
         */
        class CollisionMonitor
        {   
            private:
            adore::sim::AFactory::TParticipantFeed* participant_feed_;
            adore::env::AFactory::TPropositionWriter* proposition_writer_;
            std::unordered_map<adore::env::traffic::Participant::TTrackingID,adore::env::traffic::Participant> participants_;
            adore::env::traffic::Participant::TTrackingID egoID_;
            public:
            CollisionMonitor(
                        adore::sim::AFactory::TParticipantFeed* participant_feed,
                        adore::env::AFactory::TPropositionWriter* proposition_writer,
                        adore::env::traffic::Participant::TTrackingID egoID
                    ):
                    participant_feed_(participant_feed),
                    proposition_writer_(proposition_writer),
                    egoID_(egoID)
            {}
            void run()
            {
                bool changed = false;
                bool in_collision = false;
                while(participant_feed_->hasNext())
                {
                    adore::env::traffic::Participant p;
                    participant_feed_->getNext(p);
                    participants_.emplace(p.getTrackingID(), p).first->second = p;
                    changed = true;
                }
                if(changed)
                {
                    auto ego_it = participants_.find(egoID_);
                    if(ego_it!=participants_.end())
                    {
                        auto& ego = ego_it->second;
                        auto& egopos = ego.getCenter();
                        const double Re = std::sqrt(ego.getWidth()*ego.getWidth()+ego.getLength()*ego.getLength())*0.5;
                        for(auto it:participants_)
                        {
                            if(it.first!=egoID_)
                            {
                                auto& other = it.second;
                                auto& otherpos = other.getCenter();
                                const double Ro = std::sqrt(other.getWidth()*other.getWidth()+other.getLength()*other.getLength())*0.5;
                                auto d = otherpos-egopos;
                                if(std::sqrt(d(0)*d(0)+d(1)*d(1)+d(2)*d(2))-Re-Ro<0.0)
                                {
                                    //@TODO: separating axis test
                                    in_collision = true;
                                    break;
                                }
                            }
                        }
                        adore::env::Proposition p("IN_COLLISION",in_collision);
                        proposition_writer_->write(p);
                    }
                }
            }
        };

        class Monitor0
        {
            private:
            adore::sim::AFactory::TParticipantFeed* participant_feed_;
            adore::env::AFactory::TPropositionWriter* proposition_writer_;
            adore::env::AFactory::TNavigationGoalReader* goal_reader_;
            adore::env::AFactory::TVehicleMotionStateReader* position_reader_;
            GoalAttainmentMonitor* goalAttainmentMonitor_;
            CollisionMonitor* collisionMonitor_;
            public:
            Monitor0(
                adore::env::AFactory* envFactory,
                adore::sim::AFactory* simFactory,
                int simulationID
            )
            {
                participant_feed_ = simFactory->getParticipantFeed();
                proposition_writer_ = envFactory->getPropositionWriter();
                goal_reader_ = envFactory->getNavigationGoalReader();
                position_reader_ = envFactory->getVehicleMotionStateReader();
                goalAttainmentMonitor_ = new GoalAttainmentMonitor(goal_reader_,position_reader_,proposition_writer_);
                collisionMonitor_ = new CollisionMonitor(participant_feed_,proposition_writer_,simulationID);
            }
            virtual ~Monitor0()
            {
                delete goalAttainmentMonitor_;
                delete collisionMonitor_;
            }
            void run()
            {
                goalAttainmentMonitor_->run();
                collisionMonitor_->run();
            }
        };
    }
}