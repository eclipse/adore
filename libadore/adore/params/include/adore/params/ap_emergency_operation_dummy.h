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
#include <adore/params/ap_emergency_operation.h>
namespace adore
{
	namespace params
	{
		/**
		 * @brief a dummy implementation for testing purposes
		 * 
		 */
    class APEmergencyOperationDummy:public APEmergencyOperation
		{
		public:
			virtual double getKy()const override 
      {
        return 0.05;
      }
			virtual double getKpsi()const override 
      {
        return 0.4;
      }
			virtual double getamin()const override 
      {
        return -3.0;
      }
			virtual double getamax()const override 
      {
        return 2.0;
      }
			virtual double getDeltaMax()const override
      {
        return 1.0;
      }
			virtual double getDeltaMin()const override 
      {
        return -1.0;
      }
		};
	}
}