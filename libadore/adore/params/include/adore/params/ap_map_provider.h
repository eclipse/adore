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
namespace adore
{
	namespace params
	{
		/**
		 * @brief abstract class containing parameters to configure aspects of the map provider
		 * 
		 */
		class APMapProvider
		{
		public:
			//visibility radius of the map provider
			virtual double getVisibiltyRadius()const =0;
			virtual bool getActivatePlotting()const =0;
			virtual bool getPlotCompleteMapInLocalView()const =0;
			virtual bool useScenarioManagerMap()const =0;
			virtual int getXODRLoaderPointsPerBorder()const=0;
		};
	}
}