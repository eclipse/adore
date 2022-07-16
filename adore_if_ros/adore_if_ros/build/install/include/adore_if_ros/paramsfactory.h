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
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/

#pragma once

#include <ros/ros.h>
#include <adore/params/afactory.h>
#include <adore_if_ros/params_factory/p_vehicle.h>
#include <adore_if_ros/params_factory/p_trajectory_tracking.h>
#include <adore_if_ros/params_factory/p_emergency_operation.h>
#include <adore_if_ros/params_factory/p_map_provider.h>
#include <adore_if_ros/params_factory/p_trajectory_generation.h>
#include <adore_if_ros/params_factory/p_longitudinal_planner.h>
#include <adore_if_ros/params_factory/p_lateral_planner.h>
#include <adore_if_ros/params_factory/p_local_road_map.h>
#include <adore_if_ros/params_factory/p_lane_change_view.h>
#include <adore_if_ros/params_factory/p_lane_following_view.h>
#include <adore_if_ros/params_factory/p_tactical_planner.h>
#include <adore_if_ros/params_factory/p_function_management.h>
#include <adore_if_ros/params_factory/p_navigation.h>
#include <adore_if_ros/params_factory/p_cooperation.h>
#include <adore_if_ros/params_factory/p_traffic_light_sim.h>
#include <adore_if_ros/params_factory/p_sensor_model.h>
#include <adore_if_ros/params_factory/p_checkpoints.h>
#include <adore_if_ros/params_factory/p_mission_control.h>
#include <adore_if_ros/params_factory/p_prediction.h>
#include <adore_if_ros/params_factory/p_odometrymodel.h>
#include <adore_if_ros/params_factory/p_localizationmodel.h>

namespace adore
{
  namespace if_ROS
  {
    /**
     *  ROS implementation for transmission of parameters.
     */
    class PARAMS_Factory:public adore::params::AFactory
    {
      private:
        ros::NodeHandle n_;
        std::string prefix_;

      public:
        PARAMS_Factory(ros::NodeHandle n,std::string prefix):n_(n)
        {
          prefix_ = prefix + "PARAMS/";
        }
        virtual adore::params::APVehicle* getVehicle()const override
        {
          return new adore::if_ROS::params::PVehicle(n_,prefix_);          
        }
        virtual adore::params::APTrajectoryGeneration* getTrajectoryGeneration()const override
        {
            return new adore::if_ROS::params::PTrajectoryGeneration(n_,prefix_);       
        }
        virtual adore::params::APTrajectoryTracking* getTrajectoryTracking()const override
        {
          return new adore::if_ROS::params::PTrajectoryTracking(n_,prefix_);          
        }
        virtual adore::params::APTacticalPlanner* getTacticalPlanner()const override
        {
          return new adore::if_ROS::params::PTacticalPlanner(n_,prefix_);          
        }
        virtual adore::params::APMapProvider* getMapProvider()const override
        {
          return new adore::if_ROS::params::PMapProvider(n_,prefix_);          
        } 
        virtual adore::params::APNavigation* getNavigation()const override
        {
          return new adore::if_ROS::params::PNavigation(n_,prefix_);          
        }
        virtual adore::params::APCooperation* getCooperation()const override
        {
          return new adore::if_ROS::params::PCooperation(n_,prefix_);          
        }
        virtual adore::params::APEmergencyOperation* getEmergencyOperation()const override
        {
          return new adore::if_ROS::params::PEmergencyOperation(n_,prefix_);          
        }
        virtual adore::params::APFunctionManagement* getFunctionmanagement()const override
        {
          return new adore::if_ROS::params::PFunctionManagement(n_,prefix_);          
        }
        virtual adore::params::APLaneChangeView* getLaneChangeView()const override
        {
          return new adore::if_ROS::params::PLaneChangeView(n_,prefix_);          
        }
        virtual adore::params::APLaneFollowingView* getLaneFollowingView() const override
        {
          return new adore::if_ROS::params::PLaneFollowingView(n_,prefix_);          
        }
        virtual adore::params::APLocalRoadMap* getLocalRoadMap() const override
        {
          return new adore::if_ROS::params::PLocalRoadMap(n_,prefix_);  
        }
			  virtual adore::params::APLongitudinalPlanner* getLongitudinalPlanner() const override
        {
          return new adore::if_ROS::params::PLongitudinalPlanner(n_,prefix_);          
        }
			  virtual adore::params::APLateralPlanner* getLateralPlanner() const override
        {
          return new adore::if_ROS::params::PLateralPlanner(n_,prefix_);          
        }      
        virtual adore::params::APTrafficLightSim* getTrafficLightSim()const override
        {
          return new adore::if_ROS::params::PTrafficLightSim(n_,prefix_);          
        } 
        virtual adore::params::APSensorModel* getSensorModel() const override
        {
          return new adore::if_ROS::params::PSensorModel(n_,prefix_);
        }
        virtual adore::params::APCheckpoints* getCheckpoints() const override
        {
          return new adore::if_ROS::params::PCheckpoints(n_,prefix_);
        }
        virtual adore::params::APMissionControl* getMissionControl() const override
        {
          return new adore::if_ROS::params::PMissionControl(n_,prefix_);
        }
        virtual adore::params::APPrediction* getPrediction() const override
        {
          return new adore::if_ROS::params::PPrediction(n_,prefix_);
        }
        virtual adore::params::APOdometryModel* getOdometryModel() const override
        {
          return new adore::if_ROS::params::POdometryModel(n_,prefix_);
        }
        virtual adore::params::APLocalizationModel* getLocalizationModel()const override
        {
          return new adore::if_ROS::params::PLocalizationModel(n_,prefix_);
        }
    };
  }
}