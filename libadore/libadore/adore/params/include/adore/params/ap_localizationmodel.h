/********************************************************************************
 * Copyright (C) 2017-2021 German Aerospace Center (DLR). 
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
		 * @brief abstract class containing parameters which configure localization state estimation model
		 * 
		 */
		class APLocalizationModel
		{
		public:
			virtual double get_drift_rate_pos()const = 0;/**< returns the drift rate of the position*/
			virtual double get_drift_deviation_pos()const = 0;/**< returns the drift std deviation of the position*/
            virtual double get_jump_deviation_pos()const = 0;/**< returns std deviation of position jumps*/
            virtual double get_jump_threshold_pos()const =0;/**< returns jump threshold, which influences how often position jumps occur*/
            virtual double get_jump_deviation_heading()const = 0;/**< returns std deviation of heading jumps*/
            virtual double get_jump_threshold_heading()const =0;/**< returns jump threshold, which influences how often heading jumps occur*/
		};
	}
}