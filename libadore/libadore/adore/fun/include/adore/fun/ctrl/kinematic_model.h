/********************************************************************************
 * Copyright (C) 2017-2021 German Aerospace Center (DLR). 
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
#include <adore/mad/aodemodel.h>
#include <adore/params/ap_vehicle.h>
#include <adore/fun/vehicleextendedstate.h>
#include <math.h>

namespace adore
{
    namespace fun
    {
        /**
         * @brief kinematic vehicle model 
         */
        class KinematicModel:public adore::mad::AOdeModel<double>
        {

            public:
            /**
             * @brief compute derivative: planar kinematic model
             * @param x_in input state vector: x_in=(X,Y,Psi,vx,vy,omega,ax); with vx,vy,ax in vehicle fixed coordinates
             * @param x_out output state-derivative vector, same dimensions as x_in
             */
            virtual void f(double t, const adoreMatrix<double, 0, 1>& x_in, adoreMatrix<double, 0, 1>& dx_out) 
            {
                const double s = std::sin(x_in(2));
                const double c = std::cos(x_in(2));
                const double ax = (x_in.nc()>=7)?x_in(6):0.0;
                dx_out(0) = c*x_in(3)-s*x_in(4);
                dx_out(1) = s*x_in(3)+c*x_in(4);
                dx_out(2) = x_in(5);
                dx_out(3) = ax;
                dx_out(4) = 0.0;
                dx_out(5) = 0.0;
                if(x_in.nc()>=7)dx_out(6) = 0.0;
            }        
        };
    }
}