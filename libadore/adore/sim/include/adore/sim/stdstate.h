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
 *   Matthias Nichting
 ********************************************************************************/

#pragma once
#include <vector>
namespace adore
{
  namespace sim
  {
    struct StdState
    {
    public:
      std::vector<double> values_;
      void set(double values[], unsigned int number_of_elements)
      {
        for (unsigned int i = 0; i < number_of_elements; ++i)
        {
          values_.push_back(values[i]);
        }
      }
    };
  } // namespace sim
} // namespace adore