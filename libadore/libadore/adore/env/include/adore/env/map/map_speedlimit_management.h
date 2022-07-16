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
 * Thomas Lobig
 ********************************************************************************/

#pragma once

#include "speedlimit.h"
#include "adore/view/alane.h"
#include "adore/env/afactory.h"
#include "adore/mad/com_patterns.h"
#include <adore/mad/linearfunctiontypedefs.h>
#include <adore/mad/linearconstraintset.h>

namespace adore
{
    namespace env
    {
        /**
         * @brief automatically manage speed limit information based on current vehicle position
         *
         */
        class SpeedLimitManagement
        {
          private:

            adore::env::AFactory::TSpeedLimitFeed* speedlimit_feed_;
            std::unordered_map<adore::env::TSpeedLimitID, adore::env::SpeedLimit> known_speed_limits;
            double default_speed_limit_;

            //TODO: this function needs to be part of the alane class and should not be here
            /**
             * @brief helper function to determine if a eucledian (x,y) point is on the lane or outside, set s of the point
             * 
             * @param x 
             * @param y 
             * @param s 
             * @return true 
             * @return false 
             */
            inline bool const isOnLane(
                adore::mad::function_type_xyz const & centerSmoothed_fct,
                adore::mad::function_type_scalar const & leftOffset_fct,
                adore::mad::function_type_scalar const & rightOffset_fct,
                double const &x, double const &y, double &s) 
            {
                double offset = 0.0;
                s = centerSmoothed_fct.getClosestParameter(x, y, 1, 2, offset);

                if (    s < rightOffset_fct.limitLo()
                     || s > rightOffset_fct.limitHi()
                     || s < leftOffset_fct.limitLo()
                     || s > leftOffset_fct.limitHi() )
                {
                    // if s is outside the limits of the distance functions, assume it's not on lane
                    return false;
                }
                else
                {
                    return rightOffset_fct(s) < offset && leftOffset_fct(s) > offset;
                }
            }

          public:
            SpeedLimitManagement()
            {
                speedlimit_feed_ = adore::env::EnvFactoryInstance::get()->getSpeedLimitFeed();
                default_speed_limit_ = 0.0;
            }
            void setDefaultValue(double value){default_speed_limit_ = value;}
            void update(double const & egoX, double const & egoY){
                // read updates from the speed limit feed
                adore::env::TSpeedLimitBundle bundle;
                while (speedlimit_feed_->hasNext())
                {
                    adore::env::SpeedLimit limit_info;
                    speedlimit_feed_->getNext(limit_info);
                    bundle.push_back(limit_info);
                }

                // if there are any updates, insert them to the known speed limits
                for ( auto limit : bundle)
                {
                    known_speed_limits.insert_or_assign(limit.id,limit);
                }

                // see if any speed limit infos are completely outside the radius of interest
                // TODO utility function available?
                std::set<adore::env::TSpeedLimitID> speed_limit_ids_outside_radius;
                double visibilty_radius = adore::params::ParamsFactoryInstance::get()->getMapProvider()->getVisibiltyRadius();
                double carX = egoX;
                double carY = egoY;
                for (auto [id,item] : known_speed_limits)
                {

                    double abs1 = std::abs(carX - item.startX);
                    double abs2 = std::abs(carY - item.startY);
                    double abs3 = std::abs(carX - item.stopX);
                    double abs4 = std::abs(carY - item.stopY);

                    if (((abs1*abs1 + abs2*abs2) > visibilty_radius*visibilty_radius) and ((abs3*abs3 + abs4*abs4) > visibilty_radius*visibilty_radius))
                    {
                        speed_limit_ids_outside_radius.insert(id);
                    }
                }

                if (speed_limit_ids_outside_radius.size() > 0)
                {
                    for ( auto it = known_speed_limits.begin(); it != known_speed_limits.end();)
                    {
                        if (speed_limit_ids_outside_radius.find(it->first) != speed_limit_ids_outside_radius.end())
                        {
                            it = known_speed_limits.erase(it);
                        }
                        else
                        {
                            it++;
                        }
                        
                    }
                }
            }

            void const getFunction(
                    adore::mad::function_type_xyz & centerSmoothed_fct,
                    adore::mad::function_type_scalar & leftOffset_fct,
                    adore::mad::function_type_scalar & rightOffset_fct,
                    adore::mad::function_type_scalar & speedLimitOutput_fct
                    ) 
                       
                {
                    adore::mad::LinearConstraintSet constraint_set;
                    constraint_set.insert(centerSmoothed_fct.limitLo(),default_speed_limit_,
                                            centerSmoothed_fct.limitHi(),default_speed_limit_);
                    

                    for (auto const & [id,item] : known_speed_limits)
                    {
                        double startS;
                        double stopS;
                        if (isOnLane(centerSmoothed_fct,leftOffset_fct,rightOffset_fct,item.startX,item.startY,startS)
                        && isOnLane(centerSmoothed_fct,leftOffset_fct,rightOffset_fct,item.stopX,item.stopY,stopS))
                        {
                            if (startS != stopS)
                            {
                                constraint_set.insert(startS,item.value,stopS,item.value);
                            }
                        }
                    }

                    // TODO currently this does not allow to e.g. drive at 70 kmph if base speed is 50 kmph
                    constraint_set.bound(&(speedLimitOutput_fct),adore::mad::LinearConstraintSet::Direction::LOWER);
                }
        };
    }  // namespace env
}  // namespace adore