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
 *   Thomas Lobig - initial implementation
 ********************************************************************************/

#pragma once
#include <adore/params/afactory.h>
#include <adore/env/map/speedlimit.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/env/afactory.h>
#include <adore/mad/utility/csv_reader.h>
#include <adore/mad/com_patterns.h>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>

#include "if_plotlab/plot_shape.h"

namespace adore
{
    namespace apps
    {
        /**
         * @brief A node to gather speed limit data and provide it e.g. for laneview computation
         * 
         */
        class SpeedLimitProvider
        {
            //TODO vector only for quick prototyping, should be efficient geospatial data structure
            // using SpeedLimitBundle=adore::env::TSpeedLimitBundle;
            adore::env::TSpeedLimitBundle limits;
            std::string file_watch_path_;
            adore::env::AFactory::TVehicleMotionStateReader* motion_state_reader_;
            adore::env::AFactory::TSpeedLimitWriter* speed_limit_writer_;

            public:

            SpeedLimitProvider(std::string file_watch_path) : file_watch_path_(file_watch_path)
            {
                motion_state_reader_ = adore::env::EnvFactoryInstance::get()->getVehicleMotionStateReader();
                speed_limit_writer_ = adore::env::EnvFactoryInstance::get()->getSpeedLimitWriter();
                auto read_result = adore::mad::CsvReader::get_data<double>(file_watch_path);

                if (read_result.size() % 5 == 0)
                {
                    int index = 0;
                    int id = 0;
                    adore::env::SpeedLimit datapoint;
                    while(index < read_result.size())
                    {
                        datapoint.value  = read_result[index++];
                        datapoint.startX = read_result[index++];
                        datapoint.startY = read_result[index++];
                        datapoint.stopX  = read_result[index++];
                        datapoint.stopY  = read_result[index++];
                        datapoint.id = id++;
                        limits.push_back(datapoint);
                    }
                }
                else
                {
                    std::cerr << "ERROR speedlimit csv might be corrupted, the count of numbers is not divisible by 5";
                }
            }

            void run()
            {
                adore::env::VehicleMotionState9d motion_state;
                
                motion_state_reader_->getData(motion_state);
                double visibilty_radius = adore::params::ParamsFactoryInstance::get()->getMapProvider()->getVisibiltyRadius();

                double x = motion_state.getX();
                double y = motion_state.getY();


                // TODO might think about caching which speed limit data points were already sent and only send new ones and
                // maybe resend other less frequently, similar to how the map provider handles borders
                // adore::env::TSpeedLimitBundle msg;

                for (auto item : limits)
                {
                    double abs1 = std::abs(x - item.startX);
                    double abs2 = std::abs(y - item.startY);
                    double abs3 = std::abs(x - item.stopX);
                    double abs4 = std::abs(y - item.stopY);

                    if (((abs1*abs1 + abs2*abs2) < visibilty_radius*visibilty_radius) or ((abs3*abs3 + abs4*abs4) < visibilty_radius*visibilty_radius))
                    {
                        if (speed_limit_writer_->canWriteMore())
                        {
                            speed_limit_writer_->write(item);
                        }
                        else
                        {
                            std::cout << "Speed_limit_feed was full, data lost\n";
                        }
                        
                    }
                }
                // if (msg.size() > 0)
                // {
                    
                //     speed_limit_writer_->write(msg);
                // }
            }
        };
    }
}