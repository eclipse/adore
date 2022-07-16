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
#include <adore/mad/aodemodel.h>
#include <adore/params/ap_vehicle.h>
#include <adore/fun/vehicleextendedstate.h>

namespace adore
{
    namespace fun
    {
        /**
         * An open loop vehicle bicycle model with linear tire forces.
         * The state vector of the model contains the position of a reference point defined by parameter lambda: 
         * Lambda is the distance from the center of the rear axle (COR) to the reference point.
         * The lateral speed vy (corresponding to slip angle beta in other models) is given also for the reference point.
         * The gear state is applied as a static parameter in the differential equation.
         */
        class VLB_OpenLoop:public adore::mad::AOdeModel<double>
        {
            private:
            //parameters
            double lambda_;
            double a_;
            double b_;
            double L_;
            double h_;
            double g_;
            double mu_;
            double cf_;
            double cr_;
            double Iz_m_;
            double delta_max_;
            double delta_min_;
            VehicleExtendedState::GearState gearState_;

            public:
            /**
             * Constructor
             * @param p vehicle parameters
             */
            VLB_OpenLoop(adore::params::APVehicle* p)
            {
                //parameters
                lambda_ = 0.0;
                updateParameters(p);
            }
            /**
             * Read the parameters again.
             * @param p vehicle parameters
             */
            void updateParameters(adore::params::APVehicle* p)
            {
                a_ = p->get_a();
                b_ = p->get_b();
                L_ = a_+b_;
                h_ = p->get_h();
                g_ = p->get_g();
                mu_ = p->get_mu();
                cf_ = p->get_cf();
                cr_ = p->get_cr();
                Iz_m_ = p->get_Iz_m();
                delta_max_ = p->get_steeringAngleMax();
                delta_min_ = p->get_steeringAngleMin();
                gearState_ = VehicleExtendedState::Drive;
            }
            /**
             * set the position of the reference point
             * @param value the new reference point position
             */
            void setReference(double value)
            {
                lambda_=value;
            }
            /**
             * set the gear state
             * @param value the new gear state
             */
            void setGearState(VehicleExtendedState::GearState value)
            {
                gearState_=value;
            }



            /**
             * f computes the derivative at time t and state x_in. the result is placed in the
             * matrix/vector x_out.
             * R^10: [X,Y,psi,vx,vy,omega, ax, delta, dax, ddelta]
             * @param t time (not used here)
             * @param x_in current state
             * @param dx_out derivative of state, output of the function
             */
            virtual void f(double t, const adoreMatrix<double, 0, 1>& x_in, adoreMatrix<double, 0, 1>& dx_out) 
            {
                //states
                const double PSI = x_in(2);
                const double vx = x_in(3);
                const double vyr = x_in(4)-lambda_*x_in(5);
                // TODO investigate if the following is a bug or if the variable can be fully removed
                // const double vyf = vyr+L_*x_in(5); // fix -Wunused-variable
                const double vy = x_in(4);
                const double omega = x_in(5);
                double ax = x_in(6);
                const double delta = adore::mad::bound(delta_min_,x_in(7),delta_max_);
                const double dax = x_in(8);
                const double ddelta = x_in(9);

                switch(gearState_)
                {
                    case VehicleExtendedState::Drive:
                        if(vx<0.0 && ax<0.0)
                        {
                            ax = -ax;
                        }
                        break;
                    case VehicleExtendedState::Reverse:
                        if(vx>0.0 && ax>0.0)
                        {
                            ax = -ax;
                            ax = -ax;
                        }
                        break;
                    case VehicleExtendedState::Neutral:
                        if  ((vx>0.0 && ax>0.0)
                          || ( vx<0.0 && ax<0.0))
                        {
                            ax = -0.05 * vx;
                        }
                        break;
                    case VehicleExtendedState::Park:
                        if(vx!=0.0)
                        {
                            ax = -5.0 * vx;
                        }
                        break;

                }


                if (vx > 1.0)
                {
                    //slipangles
                    // TODO investigate the following variables are no longer used, usage commented out few lines below, can they be fully removed?
                    // double alpha_f = atan(vyf / vx) - delta; // -Wunused-variable
                    // double alpha_r = atan(vyr / vx);  // -Wunused-variable
                    //forces
                    double Fyf_Fzf, Fyr_Fzr;
                    //Fyf_Fzf = -cos(delta)*mu_*(b_/L_*g_ - h_/L_*0.0)*cf_*alpha_f;
                    // Fyr_Fzr = -mu_*(a_ / L_*g_ + h_ / L_*0.0)*cr_*alpha_r;
                    Fyf_Fzf = -mu_*(b_/L_)*g_*cf_*((vyr+L_*omega)/vx-delta);
                    Fyr_Fzr = -mu_*(a_/L_)*g_*cr_*vyr/vx;
                    //accelerations
                    double ay = Fyf_Fzf + Fyr_Fzr;
                    double domega = (1.0 / Iz_m_) * (a_*Fyf_Fzf - b_*Fyr_Fzr);
                    //derivative - dynamic
                    dx_out(0) = cos(PSI)*vx - sin(PSI)*vy;
                    dx_out(1) = sin(PSI)*vx + cos(PSI)*vy;
                    dx_out(2) = omega;
                    dx_out(3) = ax;
                    dx_out(4) = ay - vx*omega + domega*(lambda_-b_);
                    dx_out(5) = domega;
                    dx_out(6) = dax;
                    dx_out(7) = ddelta;
                    dx_out(8) = 0;
                    dx_out(9) = 0;
                }
                else
                {
                    //derivative - kinematic
                    double kappa = tan(delta) / L_;
                    dx_out(0) = cos(PSI)*vx - sin(PSI)*vy;
                    dx_out(1) = sin(PSI)*vx + cos(PSI)*vy;
                    dx_out(2) = vx*kappa;
                    dx_out(3) = ax;
                    dx_out(4) = -5.0 * (vy - vx*kappa*lambda_);
                    dx_out(5) = -5.0 * (omega - vx*kappa);
                    dx_out(6) = dax;
                    dx_out(7) = ddelta;
                    dx_out(8) = 0;
                    dx_out(9) = 0;
                }
            }        
        };
    }
}