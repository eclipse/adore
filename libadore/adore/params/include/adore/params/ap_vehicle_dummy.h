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
#include <adore/params/ap_vehicle.h>
namespace adore
{
    namespace params
    {
            /**
             * @brief dummy implementation of an abstract vehicle parameter object for testing purposes
             * 
             */
            class APVehicleDummy:public adore::params::APVehicle //@TODO create encapsulation for parameter server
            {
                        virtual std::string get_vehicle_id()const override
                        {
                        return "Dummy Vehicle";
                        }//ID of current vehicle
                        virtual double get_a() const override
                        {
                        return 1.014;
                        }//cog to front axle
                        virtual double get_b() const override
                        {
                        return 1.676;
                        }//rear axle to cog
                        virtual double get_c() const override
                        {
                        return 0.97;
                        }//front axle to front border
                        virtual double get_d() const override
                        {
                        return 1.12;
                        }//rear border to rear axle
                        virtual double get_m() const override
                        {
                        return 1800.0;
                        }//mass
                        virtual double get_mu() const override
                        {
                        return 0.8;
                        }//friction coefficient
                        virtual double get_g() const override
                        {
                        return 9.81;
                        }//gravitational constant
                        virtual double get_h() const override
                        {
                        return 0.5;
                        }//cog height above ground
                        virtual double get_cf() const override
                        {
                        return 10.8;
                        }//front normalized tire stiffness for bicycle model
                        virtual double get_cr() const override
                        {
                        return 17.8;
                        }//rear normalized tire stiffness for bicycle model
                        virtual double get_Iz_m() const override
                        {
                        return 1.57;
                        }//rotational inertia around up axis devided by mass
                        virtual double get_wf() const override
                        {
                        return 1.7;
                        }//track width front
                        virtual double get_wr() const override
                        {
                        return 1.7;
                        }//track width rear
                        virtual double get_bodyWidth() const override
                        {
                        return 1.82;
                        }
                        virtual double get_steeringRatio() const override
                        {
                        return 1.0;
                        }
                        virtual double get_steeringAngleOffsetMeasured() const override
                        {
                        return 0.0;
                        }
                        virtual double get_steeringAngleOffsetCommand() const override
                        {
                        return 0.0;
                        }
                        virtual double get_steeringAngleMax() const override
                        {
                        return 0.7;
                        }
                        virtual double get_steeringAngleMin() const override
                        {
                        return -0.7;
                        }
                        virtual double get_C() const override
                        {
                        return 63000.0;
                        }//unnormalized cornering stiffness
                        virtual double get_brakeBalanceFront() const override
                        {
                        return 0.6;
                        }//returns the percentage of brake force allocated to the front axle, e.g. 0.6 is a typical value
                        virtual double get_accelerationBalanceFront() const override
                        {
                        return 0.4;
                        }//returns the percentage of acceleration force allocated to the front axle, e.g. 1.0 for front drive
                        virtual double get_observationPointForPosition() const override
                        {
                        return 0.0;
                        }
                        virtual double get_observationPointForVelocity() const override
                        {
                        return 0.0;
                        }
                        virtual double get_observationPointForAcceleration() const override
                        {
                        return 0.0;
                        }
                        virtual double get_vehicleFlag() const override
                        {
                        return 0.0;
                        }//returns the flag of currently used vehicle
            };
    }
}