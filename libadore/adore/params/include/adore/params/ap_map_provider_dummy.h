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
#include <adore/params/ap_map_provider.h>
namespace adore
{
    namespace params
    {
            /**
             * @brief a dummy implementation for testing purposes
             * 
             */
            class APMapProviderDummy:public adore::params::APMapProvider
            {
                virtual double getVisibiltyRadius() const override
                {
                    return 200;
                }
                virtual bool getActivatePlotting() const override
                {
                    return true;                
                }
                virtual bool getPlotCompleteMapInLocalView() const override
                {
                    return false;
                }
                virtual bool useScenarioManagerMap() const override
                {
                    return false;
                }
                virtual int getXODRLoaderPointsPerBorder()const override
                {
                    return 128;
                }
            };
    }
}