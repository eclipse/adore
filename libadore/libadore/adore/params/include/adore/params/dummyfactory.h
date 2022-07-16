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

#include <adore/params/afactory.h>
#include <adore/params/ap_vehicle_dummy.h>
#include <adore/params/ap_trajectory_generation_dummy.h>
#include <adore/params/ap_trajectory_tracking_dummy.h>
// #include <adore/params/ap_tactical_planner_dummy.h>
#include <adore/params/ap_map_provider_dummy.h>
// #include <adore/params/ap_navigation_dummy.h>
// #include <adore/params/ap_cooperation_dummy.h>
#include <adore/params/ap_emergency_operation_dummy.h>
// #include <adore/params/ap_function_management_dummy.h>
#include <adore/params/ap_lane_change_view_dummy.h>
#include <adore/params/ap_lane_following_view_dummy.h>
#include <adore/params/ap_local_road_map_dummy.h>
#include <adore/params/ap_longitudinal_planner_dummy.h>
#include <adore/params/ap_lateral_planner_dummy.h>
#include <adore/params/ap_traffic_light_sim_dummy.h>




namespace adore
{
	namespace params
	{
		/**
		 * @brief a dummy implementation of the adore::params::afactory for testing purposes
		 * 
		 */
		class DummyFactory: public AFactory
		{

		public:
			virtual APVehicle* getVehicle()const{return new APVehicleDummy();}
			virtual APTrajectoryGeneration* getTrajectoryGeneration()const{return new APTrajectoryGenerationDummy();}
			virtual APTrajectoryTracking* getTrajectoryTracking()const{return new APTrajectoryTrackingDummy();}
			virtual APTacticalPlanner* getTacticalPlanner()const{return nullptr;}
			virtual APMapProvider* getMapProvider()const{return new APMapProviderDummy();} 
			virtual APNavigation* getNavigation()const{return nullptr;}
			virtual APCooperation* getCooperation()const{return nullptr;}
			virtual APEmergencyOperation* getEmergencyOperation()const{return new APEmergencyOperationDummy();}
			virtual APFunctionManagement* getFunctionmanagement()const{return nullptr;}
			virtual APLaneChangeView* getLaneChangeView()const{return new APLaneChangeViewDummy();}
			virtual APLaneFollowingView* getLaneFollowingView()const{return new APLaneFollowingViewDummy();}
			virtual APLocalRoadMap* getLocalRoadMap()const{return new APLocalRoadMapDummy();}
			virtual APLongitudinalPlanner* getLongitudinalPlanner()const{return new APLongitudinalPlannerDummy();}
			virtual APLateralPlanner* getLateralPlanner()const{return new APLateralPlannerDummy();}
			virtual APTrafficLightSim* getTrafficLightSim()const{return new APTrafficLightSimDummy();}

		};
	}
}