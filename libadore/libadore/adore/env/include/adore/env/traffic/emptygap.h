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
#include <adore/view/agap.h>

namespace adore
{
namespace env
{
/**
 * @brief defines a gap for testing purposes, which never has lead or chase vehicles.
 */
class EmptyGap:public adore::view::AGap
{
public:
    virtual adore::view::AGap::EGapState getState(double s, double t) override
    {
        return adore::view::AGap::OPEN;
    }
    virtual double getLeadProgress(double t, double guard) override
    {
        return guard;
    }
    virtual double getChaseProgress(double t, double guard) override
    {
        return guard;
    }
};
}
}