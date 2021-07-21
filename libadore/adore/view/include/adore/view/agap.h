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
namespace view
{

/**
 * @brief defines a gap in traffic, into which a merge may be planned
 */
class AGap
{
public:
    enum EGapState
    {
        OPENING,    /**< gap before merge possible */
        OPEN,       /**< gap while merge possible */
        CLOSED      /**< gap after merge possible */
    };
    /**
     * @brief get the state of the gap at a certain time, depending on ego position
     * @param s ego position
     * @param t time
     */
    virtual EGapState getState(double s, double t) =0;

    /**
     * @brief get a prediction of the lead progress at a certain time
     * Lead vehicle is leading the gap. Progress is measured along coordinates of target lane.
     * @param t time
     * @param guard guard value returned if lead does not exist
     */
    virtual double getLeadProgress(double t, double guard) =0;

    /**
     * @brief get a prediction of the chase progress at a certain time
     * Chase vehicle is chasing the gap.  Progress is measured along coordinates of target lane.
     * @param t time
     * @param guard guard value returned if chase does not exist
     */
    virtual double getChaseProgress(double t, double guard) =0;

};

}
}