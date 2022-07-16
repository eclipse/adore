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


namespace adore
{
	namespace params
	{

		/**
		 * @brief parameter interface for parameters related to prediction
		 * 
		 */
		class APPrediction
		{
		public:
            ///prediction duration for objects that can be matched to road
            virtual double get_roadbased_prediction_duration()const=0;
            ///maximum acceleration for normal behavior for objects that can be matched to road
            virtual double get_roadbased_expected_acc_ub()const=0;
            ///delay after which expected_acc_ub is applied
            virtual double get_roadbased_expected_acc_ub_delay()const=0;
            ///minimum acceleration for normal behavior for objects that can be matched to road
            virtual double get_roadbased_expected_acc_lb()const=0;
            ///maximum velocity for normal behavior for objects that can be matched to road
            virtual double get_roadbased_expected_vel_ub()const=0;
            ///maximum acceleration for worst-case behavior for objects that can be matched to road
            virtual double get_roadbased_worstcase_acc_ub()const=0;
            ///delay after which worstcase_acc_ub is applied
            virtual double get_roadbased_worstcase_acc_ub_delay()const=0;
            ///minimum acceleration for worst-case for objects that can be matched to road
            virtual double get_roadbased_worstcase_acc_lb()const=0;
            ///maximum velocity for worst-case for objects that can be matched to road
            virtual double get_roadbased_worstcase_vel_ub()const=0;
            /// maximum difference between object and road heading for object to be matchable to road
            virtual double get_roadbased_heading_deviation_ub()const=0;
            /// precision of object shape approximation in lateral direction for objects that can be matched to road
            virtual double get_roadbased_lat_precision()const=0;
            /// assumed maximum lateral detection error for objects that can be matched to road (buffer zone)
            virtual double get_roadbased_lat_error()const=0;
            /// assumed maximum longitudinal detectionfor objects that can be matched to road (buffer zone)
            virtual double get_roadbased_lon_error()const=0;
            /// time buffer ahead of an object (objrect predicted to arrive given seconds earlier at a location)
            virtual double get_roadbased_time_headway()const =0;
            /// time buffer behind object (object predicted to leave a location given seconds later)
            virtual double get_roadbased_time_leeway()const =0; 
            /// prediction duration for objects that can not be matched to road
            virtual double get_offroad_prediction_duration()const=0;
            /// maximum acceleration for normal behavior for objects that can not be matched to road
            virtual double get_offroad_expected_acc_ub()const=0;
            /// minimum acceleration for normal behavior for objects that can not be matched to road
            virtual double get_offroad_expected_acc_lb()const=0;
            /// maximum velocity for normal behavior for objects that can not be matched to road
            virtual double get_offroad_expected_vel_ub()const=0;
            /// maximum acceleration for worst-case behavior for objects that can not be matched to road
            virtual double get_offroad_worstcase_acc_ub()const=0;
            ///delay after which worstcase_acc_ub is applied
            virtual double get_offroad_worstcase_acc_ub_delay()const=0;
            /// minimum acceleration for worst-case behavior for objects that can not be matched to road
            virtual double get_offroad_worstcase_acc_lb()const=0;
            /// maximum velocity for worst-case behavior for objects that can not be matched to road
            virtual double get_offroad_worstcase_vel_ub()const=0;
            /// precision of object shape approximation in lateral direction for objects that can not be matched to road
            virtual double get_offroad_lat_precision()const=0;
            /// assumed maximum lateral detection error for objects that can not be matched to road
            virtual double get_offroad_lat_error()const=0;
            /// assumed maximum longitudinal detection error for objects that can not be matched to road
            virtual double get_offroad_lon_error()const=0;
            /// time buffer ahead of an object (objrect predicted to arrive given seconds earlier at a location)
            virtual double get_offroad_time_headway()const =0;
            /// time buffer behind object (object predicted to leave a location given seconds later)
            virtual double get_offroad_time_leeway()const =0; 
            /// distinction between clutter and static traffic objects: how far into road has object to extend to be recognized as traffic?
            virtual double get_area_of_interest_shrink()const =0; 
            /// filtering out all static objects not inside area of effect
            virtual double get_area_of_effect_shrink()const =0; 
            /// filtering of precedence rules for worstcase maneuvers: @return if true activate filtering
            virtual bool get_worstcase_filter_precedence()const=0;
            /// filtering of tcd for worstcase maneuvers: @return if true activate filtering
            virtual bool get_worstcase_filter_tcd()const=0;
            /// returns prediction strategy: 0 width of object, 1 width of road, 2 width of object-> width of road
            virtual int get_setbased_prediction_strategy()const=0;
            /// returns maximum width for a prediction
            virtual double get_prediction_width_ub()const=0;
            /// returns the minimum width for a prediction
            virtual double get_prediction_width_lb()const=0;
		};
	}
}