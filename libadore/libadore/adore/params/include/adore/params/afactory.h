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

#include <adore/params/ap_vehicle.h>
#include <adore/params/ap_trajectory_generation.h>
#include <adore/params/ap_trajectory_tracking.h>
#include <adore/params/ap_tactical_planner.h>
#include <adore/params/ap_map_provider.h>
#include <adore/params/ap_navigation.h>
#include <adore/params/ap_cooperation.h>
#include <adore/params/ap_emergency_operation.h>
#include <adore/params/ap_function_management.h>
#include <adore/params/ap_lane_change_view.h>
#include <adore/params/ap_lane_following_view.h>
#include <adore/params/ap_local_road_map.h>
#include <adore/params/ap_longitudinal_planner.h>
#include <adore/params/ap_lateral_planner.h>
#include <adore/params/ap_traffic_light_sim.h>
#include <adore/params/ap_sensor_model.h>
#include <adore/params/ap_checkpoints.h>
#include <adore/params/ap_mission_control.h>
#include <adore/params/ap_prediction.h>
#include <adore/params/ap_odometrymodel.h>
#include <adore/params/ap_localizationmodel.h>
#include <iostream>

namespace adore
{
	/**
	 * @brief this namespace contains functionality related to providing parameters to configure runtime behaviour
	 * 
	 */
	namespace params
	{

		/**
		 * @brief abstract factory for adore::params classes
		 * 
		 */
		class AFactory
		{

		public:
			virtual APVehicle* getVehicle()const =0;
			virtual APTrajectoryGeneration* getTrajectoryGeneration()const =0;
			virtual APTrajectoryTracking* getTrajectoryTracking()const =0;
			virtual APTacticalPlanner* getTacticalPlanner()const =0;
			virtual APMapProvider* getMapProvider()const =0; 
			virtual APTrafficLightSim* getTrafficLightSim()const =0; 
			virtual APNavigation* getNavigation()const =0;
			virtual APCooperation* getCooperation()const =0;
			virtual APEmergencyOperation* getEmergencyOperation()const =0;
			virtual APFunctionManagement* getFunctionmanagement()const =0;
			virtual APLaneChangeView* getLaneChangeView()const =0;
			virtual APLaneFollowingView* getLaneFollowingView() const =0;
			virtual APLocalRoadMap* getLocalRoadMap() const =0;
			virtual APLongitudinalPlanner* getLongitudinalPlanner() const =0;
			virtual APLateralPlanner* getLateralPlanner() const =0;
			virtual APSensorModel* getSensorModel() const =0;
      virtual APCheckpoints* getCheckpoints() const =0;
      virtual APMissionControl* getMissionControl() const =0;
      virtual APPrediction* getPrediction() const=0;
      virtual APOdometryModel* getOdometryModel() const=0;
      virtual APLocalizationModel* getLocalizationModel() const=0;

		};

		   /**
         * @brief Utility class to simplify factory access
         *
         * init() function should be used once per process to set factories
         */
        class ParamsFactoryInstance final
        {
          private:
            adore::params::AFactory* paramsFactory_ = 0;
 
            /**
             * @brief Function to access singleton instance of the AllFactory using magic static
             *
             * @return AllFactory& reference on the threadsafe singleton instance
             */
            static ParamsFactoryInstance& getInstance()
            {
                static ParamsFactoryInstance instance;
                return instance;
            }

          public:
            static adore::params::AFactory* get()
            {
              if (getInstance().paramsFactory_ == 0)
              {
                std::cerr << " WARNING Accessing singleton paramsFactory instance without ever running init()" 
                      << std::endl;
              }
              return getInstance().paramsFactory_;
            }

            /**
             * @brief Initialize private members of AllFactory
             *
             * This function should ideally run only once per process before the singleton instance is used
             * Makes no guarantees on thread-safety
             *
			       * @param paramsFactory
             */
            static void init(adore::params::AFactory* paramsFactory)
            {
                auto& instance = getInstance();
                instance.paramsFactory_ = paramsFactory;	
            }

          private:
            ParamsFactoryInstance() = default;
            ~ParamsFactoryInstance() = default;

            ParamsFactoryInstance(const ParamsFactoryInstance&) = delete;
            ParamsFactoryInstance& operator=(const ParamsFactoryInstance&) = delete;
            ParamsFactoryInstance(ParamsFactoryInstance&&) = delete;
            ParamsFactoryInstance& operator=(ParamsFactoryInstance&&) = delete;
        };
	}
}