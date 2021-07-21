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
#include <string>
#include <unordered_map>

namespace adore
{
    namespace env
    {
        /**
         * @brief A logical proposition, with a possible timeout for the information.
         */
        struct Proposition
        {
            unsigned char order_;   /**< order of calculus, (zero/first), or enum value for others if used */
            std::string term_;      /**< logical term, e.g. GOAL_REACHED in 0-order */
            bool value_;            /**< boolean value of term, e.g. GOAL_REACHED=true */
            double timeout_;        /**< UTC time, after which value shall be considered undefined */
            bool has_timeout_;      /**< if false, timeout shall be considered infinite */
            Proposition(){}
            Proposition(std::string term,bool value)
                :order_(0),term_(term),value_(value),timeout_(0.0),has_timeout_(false){}
        };

        /**
         * @brief A data structure managing logical propositions of order 0
         */
        class PropositionSet0
        {
            private:
            std::unordered_map<std::string,Proposition> map_;   /**< a map from proposition term (string) to proposition struct */
            double t_;                                          /**< the current time, allows to manage information timeout */
            public:
            PropositionSet0():t_(0.0){}
            /**
             * @brief update the current time in order to make use of propositions which are defined for a limited time
             */
            void setTime(double t)
            {
                t_ = t;
            }
            /**
             * @brief insert or update a proposition
             */            
            void add(const Proposition& p)
            {
                map_.emplace(p.term_, p).first->second = p;
            }
            /**
             * @brief returns the boolean value of a term, if known. otherwise default_value is returned
             */
            bool getValue(const std::string& term,bool default_value)const
            {
                auto it = map_.find(term);
                if(it==map_.end())
                {
                    return default_value;
                }
                else if(it->second.has_timeout_ && it->second.timeout_<t_)
                {
                    return default_value;
                }
                else
                {
                    return it->second.value_;
                }                
            }
            /**
             * @brief returns true if value of a term is known.
             */
            bool hasValue(const std::string& term)const
            {
                auto it = map_.find(term);
                if(it==map_.end())
                {
                    return false;
                }
                else if(it->second.has_timeout_ && it->second.timeout_<t_)
                {
                    return false;
                }
                else
                {
                    return true;
                }                
            }

        };


    }
}