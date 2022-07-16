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
		 * @brief abstract class for vehicle sensor model parameters
		 * 
		 */
		class APSensorModel
		{

		public:
		  	///maximum sensor range for object detection in a generalized sensor setting
			virtual double get_objectDetectionRange()const =0;
			///time after which object detections are discarded
			virtual double get_objectDiscardAge()const =0;
		};
	}
}