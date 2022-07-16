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
#include <math.h>
#include <vector>

namespace adore
{
namespace env
{
    /**
     * GapData contains information which allows to rate, select and track a gap in traffic.
     */
    struct GapData
    {
        public:
        double rating;/**< rating of gap: 0 is best, high is bad*/
        bool feasible;/**< can be set to by rating algorithm*/
        double anchor_X;/**< gap anchor X coordinate*/
        double anchor_Y;/**< gap anchor Y coordinate*/
        double anchor_Z;/**< gap anchor Z coordinate*/
        double anchor_dX;/**< gap anchor X velocity component*/
        double anchor_dY;/**< gap anchor Y velocity component*/
        double anchor_dZ;/**< gap anchor Z velocity component*/
        double anchor_vt;/**< gap tangential anchor velocity*/
        double t_obs;/**< observation time*/
        bool lead_exists;/**< true if a lead vehicle exists*/
        bool chase_exists;/**< true if a chase vehicle exists*/
        double s_lead;/**< distance from anchor to lead vehicle rear end along lane at t_obs*/
        double s_chase;/**< distance from anchor to chase vehicle front end at t_obs*/
        double v_lead;/**< lead vehicle velocity*/
        double v_chase;/**< chase vehicle velocity*/
        double s_anchor;/**< s value of anchor, relative to lane view*/
        double s_gate_opening;/**< distance from anchor to gate opening, negative if gate opening is behind*/
        double s_gate_closure;/**< distance from anchor to gate closure, negative if gate closure is behind*/
        GapData()
        :rating(1.0e99),feasible(true),anchor_X(0.0),anchor_Y(0.0),anchor_Z(0.0),anchor_dX(0.0),anchor_dY(0.0),anchor_dZ(0.0),anchor_vt(0.0),
         t_obs(0.0),lead_exists(false),chase_exists(false),s_lead(0.0),s_chase(0.0),
         v_lead(0.0),v_chase(0.0),s_anchor(0.0),s_gate_opening(0.0),s_gate_closure(0.0){}
    };
    typedef std::vector<GapData> GapQueue;
}
}