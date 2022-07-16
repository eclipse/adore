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
#include <adore/mad/occupancycylinder.h>

namespace adore
{
namespace view
{
    struct ConflictPoint
    {
        double s,t0,t1;
        ConflictPoint(double s,double t0,double t1)
            :s(s),t0(t0),t1(t1){}
    };

    class AConflictPointSet
    {
        public:
        virtual int size() const =0;
        virtual ConflictPoint getPoint(int i)const =0;
    };
    
}
}