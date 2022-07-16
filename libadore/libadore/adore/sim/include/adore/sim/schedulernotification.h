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
#include <cstdint>

namespace adore
{
  namespace sim
  {
    struct SchedulerNotification
    {
        private:
        uint32_t upper_time_limit_sec;
        uint32_t upper_time_limit_nsec;
        unsigned int identifier;
        public:
        uint32_t getUpperTimeLimit_sec() const
        {
            return upper_time_limit_sec;
        }
        uint32_t getUpperTimeLimit_nsec() const
        {
            return upper_time_limit_nsec;
        }
        std::pair<uint32_t,uint32_t> getUpperTimeLimit_pair()
        {
            return std::make_pair(upper_time_limit_sec,upper_time_limit_nsec);
        }
        void setUpperTimeLimit (uint32_t sec, uint32_t nsec)
        {
            while (nsec >= 1e9)
            {
                nsec-=1e9;
                sec+=1;
            }
            upper_time_limit_sec = sec;
            upper_time_limit_nsec = nsec;
        }
        void setZero(unsigned int id)
        {
            identifier = id;
            upper_time_limit_sec = 0;
            upper_time_limit_nsec = 0;
        }
        unsigned int getID() const
        {
            return identifier;
        }
        void setID(unsigned int id)
        {
            identifier = id;
        }
    };
  }
}