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
#include <adore/mad/zonotope.h>
#include <adore/mad/alfunction.h>
#include <adore/mad/oderk4.h>


namespace adore
{
	namespace fun
	{
        /**
         * Computation of linearly approximated reachable-set tubes 
         */
        template<int nx,int nu,int nd,int nm,int ntau,int ngen>
        class ZonotopeTube
        {
            public:
            typedef adore::mad::AScalarToN<double,ntau> Ttrajectory;
            typedef adore::mad::Zonotope<double,nx+nu,ngen> TZr;
            typedef adore::mad::Zonotope<double,nd,50> TZd;
            typedef adore::mad::Zonotope<double,nx,50> TZm;

            //required:
            // - reference trajectory: state and feed-forward control (@TODO define constraints and reference for emplanner)
            // - open loop system model f
            // - feedback controller c
            // - disturbance error zonotope
            // - measurement error zonotope

            void compute()
            {
                //compute \bar{x}(t) with ode-solver: closed-loop model wrt reference trajectory, considering \bar{x}(t0) as measured initial state
                // use a rough time discretization, 0.01 
                //loop through all time steps
                    // compute derivatives df/dex, df/deu, df/ded, K=dc/dex
                    // (allow client application to supply derivatives)
                    // compute  matrix exponential // or approximate initial value problem for all generators? (check that in matlab..., what is matthias algorithm?)
            }

        };
    }
}
