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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#pragma once

#include <map>
#include <string>
#include <vector>
#include <string_view>
#include <memory>
#include <iostream>
#include <functional>
#include <adore/sim/afactory.h>
#include <adore/sim/schedulernotification.h>
#include <chrono>
#include <thread>

namespace adore
{
    namespace sim
    {

        template <typename RegistreeInfo, typename TimeKeyType>
        /**
 * @brief Scheduler is a class which provides functionality for stepped simulation
 * 
 */
        class Scheduler
        {
            using ScheduleMap = std::multimap<TimeKeyType, RegistreeInfo>;

        private:
            adore::sim::AFactory *simFactory_;
            adore::sim::AFactory::TSchedulerNotificationFeed *notificationFeed_;
            adore::sim::AFactory::TSimulationTimeWriter *simulationTimeWriter_;
            adore::sim::AFactory::TClockTimeWriter *clockTimeWriter_;
            bool pause_;       // true, if scheduling is paused
            int minRegisters_; // number of registers that is needed to start the scheduling
            bool autostart_;   // true, if scheduling should start without keyboard input
            bool started_;     // ture, if scheduling has started
            std::pair<uint32_t, uint32_t> lastWallTime_;
            std::pair<uint32_t, uint32_t> lastRosTime_;
            ScheduleMap *schedule_;
            TimeKeyType now_;

            /**
     * Search for the registree info in schedule_ and replace the associated time key
     * 
     * If the registree info is not found, it is added to the schedule_
     * 
     * @param ri registree info
     * @param tk time key
     */
            void updateSchedule(RegistreeInfo ri, TimeKeyType tk)
            {
                for (auto it = schedule_->begin(); it != schedule_->end(); ++it)
                {
                    if (it->second == ri)
                    {
                        auto nh = schedule_->extract(it);
                        nh.key() = tk;
                        schedule_->insert(std::move(nh));
                        return;
                    }
                }
                schedule_->insert(std::make_pair(tk, ri));
                std::cout << ri << " has registered." << std::endl;
                write();
                --minRegisters_;
            }

            Scheduler(adore::sim::AFactory *simFactory, int minRegisters, bool autostart)
            {
                schedule_ = new ScheduleMap();
                simFactory_ = simFactory;
                notificationFeed_ = simFactory_->getSchedulerNotificationFeed();
                std::function<void()> fcn(std::bind(&Scheduler::update, this));
                notificationFeed_->setCallback(fcn);
                simulationTimeWriter_ = simFactory_->getSimulationTimeWriter();
                clockTimeWriter_ = simFactory_->getClockTimeWriter();
                now_ = std::make_pair(0, 0);
                pause_ = false;
                autostart_ = autostart;
                started_ = false;
                minRegisters_ = minRegisters;
                lastWallTime_ = std::make_pair(ros::WallTime::now().sec, ros::WallTime::now().nsec);
                lastRosTime_ = std::make_pair(ros::Time::now().sec, ros::Time::now().nsec);
            }

        public:
            static Scheduler *getInstance(adore::sim::AFactory *simFactory, int minRegisters, bool autostart)
            {
                return new Scheduler(simFactory, minRegisters, autostart);
            }
            void init()
            {
                std::cout << std::endl
                          << "Press s to start" << std::endl;
            }
            /**
     * Process the message feed
     * 
     * Read the notification feed and update the schedule map
     */
            void processMessageFeed()
            {
                while (notificationFeed_->hasNext())
                {
                    SchedulerNotification notification;
                    notificationFeed_->getNext(notification);
                    updateSchedule(notification.getID(), notification.getUpperTimeLimit_pair());
                }
            }
            inline void setNewTime(bool incrementalIncrease = false)
            {
                if (!schedule_->empty())
                {
                    auto upper_time_limit = schedule_->begin()->first;
                    /**
                     * incrementalIncrease must be used in order to trigger initial timer event in ros based python nodes
                     */
                    if(incrementalIncrease)
                    {
                        auto d = (upper_time_limit.second - now_.second) / 1000;
                        while (upper_time_limit.second>now_.second)
                        {
                            now_.second += d;
                            write();
                        }
                    }
                    if (now_ < upper_time_limit)
                    {
                        bool print = false;
                        if (upper_time_limit.first > now_.first && upper_time_limit.first % 5 == 0)
                            print = true;
                        now_ = upper_time_limit;
                        if (print)
                            printTime();
                        write();
                    }
                }
                else
                {
                    now_.first = 0;
                    now_.second = 0;
                    write();
                }
            }
            void togglePause()
            {
                pause_ = !pause_;
                std::cout << (pause_ ? "pause at " : "resume at ");
                printTime();
                if (!pause_)
                    setNewTime();
            }
            inline void write()
            {
                simulationTimeWriter_->write((double)now_.first + ((double)now_.second) * 1e-9);
                clockTimeWriter_->write(now_);
            }
            void printTime()
            {
                std::pair<uint32_t, uint32_t> newWallTime = std::make_pair(ros::WallTime::now().sec, ros::WallTime::now().nsec);
                double speedfactor = ((double)now_.first + ((double)now_.second) * 1e-9 - (double)lastRosTime_.first - ((double)lastRosTime_.second) * 1e-9) /
                                     ((double)newWallTime.first + ((double)newWallTime.second) * 1e-9 - (double)lastWallTime_.first - ((double)lastWallTime_.second) * 1e-9);
                std::cout << "time = " << now_.first << "." << std::setfill('0') << std::setw(9) << now_.second << "; rel. simulation speed = " << speedfactor << std::endl;
                lastWallTime_ = newWallTime;
                lastRosTime_ = now_;
            }
            void update()
            {
                processMessageFeed();
                if (!pause_ && started_)
                {
                    setNewTime();
                    return;
                }
                if (!pause_ && minRegisters_ < 1 && autostart_)
                {
                    std::cout << "scheduling has automatically started" << std::endl;
                    started_ = true;
                    setNewTime();
                    return;
                }
            }
            void printInfo()
            {
                std::cout << "\nscheduling info at ";
                printTime();
                for (auto i = schedule_->begin(); i != schedule_->end(); ++i)
                {
                    std::cout << "  max time for id " << i->second << " is " << i->first.first << "." << std::setfill('0') << std::setw(9) << i->first.second << std::endl;
                }
            }
            void start()
            {
                std::cout << "scheduling started" << std::endl;
                started_ = true;
                setNewTime(true);
            }
        };
    } // namespace sim
} // namespace adore
