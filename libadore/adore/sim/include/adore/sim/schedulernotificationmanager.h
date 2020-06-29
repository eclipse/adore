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
        AFactory::TSchedulerNotificationWriter* writer_;
        SchedulerNotification sn_;
        uint32_t duration_; // in nano seconds
        public:
        SchedulerNotificationManager(AFactory* sim_factory, unsigned int id, uint32_t duration, bool reg = true)
        : duration_(duration)
        {
            writer_ = sim_factory->getSchedulerNotificationWriter();
            sn_.setID(id);
            if (reg) registerAtScheduler();
        }
        void registerAtScheduler()
        {
            sn_.setUpperTimeLimit(0,0);
            writer_->write(sn_);
        }
        void notifyScheduler(uint32_t sec, uint32_t nsec)
        {
            sn_.setUpperTimeLimit(sec,nsec+duration_);
            writer_->write(sn_);
        }
    };
  }
}