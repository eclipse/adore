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

#include <adore/env/afactory.h>
#include <adore_if_ros_msg/OccupancyCylinderPredictionSet.h>


namespace adore
{
    namespace if_ROS
    {
        struct OccupancyConverter
        {
            /**
             * Conversion of adore::env::OccupancyCylinderPredictionSet to adore_if_ros_msg::OccupancyCylinderPredicitonSet message
             */
            adore_if_ros_msg::OccupancyCylinderPredictionSet operator()(const adore::env::OccupancyCylinderPredictionSet& set)
            {
                adore_if_ros_msg::OccupancyCylinderPredictionSet msg;
                for(auto& prediction: set)
                {
                    adore_if_ros_msg::OccupancyCylinderPrediction msgi;
                    msgi.trackingID = prediction.trackingID_;
                    msgi.branchID = prediction.branchID_;
                    msgi.predecessorID = prediction.predecessorID_;
                    msgi.confidence = prediction.confidence_;
                    for(const auto& pair: prediction.occupancy_.getLevel(0))
                    {
                        const auto& cylinder = pair.second;
                        adore_if_ros_msg::OccupancyCylinder msgij;
                        msgij.rxy = cylinder.rxy_;
                        msgij.x = cylinder.x_;
                        msgij.y = cylinder.y_;
                        msgij.t0 = cylinder.t0_;
                        msgij.t1 = cylinder.t1_;
                        msgij.z0 = cylinder.z0_;
                        msgij.z1 = cylinder.z1_;
                        msgi.occupancy.push_back(msgij);
                    }
                    msg.data.push_back(msgi);
                }

                return msg;   
            }
            /**
             * Conversion of adore_if_ros_msg::OccupancyCylinderPredictionSet message to adore::env::OccupancyCylinderPredictionSet
             */
            void operator()(adore_if_ros_msg::OccupancyCylinderPredictionSetConstPtr msg,adore::env::OccupancyCylinderPredictionSet* set)
            {
                set->clear();
                for(auto& msgi:msg->data)
                {
                    adore::env::OccupancyCylinderPrediction prediction;
                    prediction.trackingID_ = msgi.trackingID;
                    prediction.branchID_ = msgi.branchID;
                    prediction.predecessorID_ = msgi.predecessorID;
                    prediction.confidence_ = msgi.confidence;
                    for(auto& msgij:msgi.occupancy)
                    {
                        adore::mad::OccupancyCylinder cylinder;
                        cylinder.rxy_ = msgij.rxy;
                        cylinder.x_ = msgij.x;
                        cylinder.y_ = msgij.y;
                        cylinder.t0_ = msgij.t0;
                        cylinder.t1_ = msgij.t1;
                        cylinder.z0_ = msgij.z0;
                        cylinder.z1_ = msgij.z1;
                        prediction.occupancy_.insert(cylinder);
                    }
                    set->push_back(prediction);
                }
            }
        };
    }
}
