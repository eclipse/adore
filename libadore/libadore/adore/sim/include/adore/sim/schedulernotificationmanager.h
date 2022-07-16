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
#include <adore/sim/afactory.h>

namespace adore
{
  namespace sim
  {
    class SchedulerNotificationManager
    {
    private:
      AFactory::TSchedulerNotificationWriter *writer_;
      SchedulerNotification sn_;
      uint32_t duration_; // in nano seconds
      bool pause_;

    public:
      SchedulerNotificationManager(AFactory *sim_factory, unsigned int id, uint32_t duration, bool reg = true)
          : duration_(duration)
      {
        writer_ = sim_factory->getSchedulerNotificationWriter();
        while (!writer_->getNumberOfSubscribers())
        {
          std::cout << "wait for the scheduler ..." << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        sn_.setID(id);
        std::cout << "scheduler id : " << id << std::endl;
        pause_ = false;
        if (reg)
          registerAtScheduler();
      }
      void registerAtScheduler()
      {
        //sn_.setUpperTimeLimit(0, 0);
        sn_.setUpperTimeLimit(0, duration_);
        writer_->write(sn_);
      }
      /*
      void startScheduler()
      {
        sn_.setUpperTimeLimit(0, duration_);
        writer_->write(sn_);
      }
      */
      void notifyScheduler(uint32_t sec, uint32_t nsec)
      {
        sn_.setUpperTimeLimit(sec, nsec + duration_);
        if (!pause_)
        {
          writer_->write(sn_);
        }
      }
      void pause()
      {
        pause_ = true;
      }
      void resume()
      {
        pause_ = false;
        writer_->write(sn_);
      }
    };
  } // namespace sim
} // namespace adore