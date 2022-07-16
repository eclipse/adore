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

#include <adore/view/alane.h>
#include <adore/fun/ctrl/vlb_openloop.h>
#include <adore/fun/planarvehiclestate10d.h>
#include <adore/fun/vehiclemotionstate9d.h>
#include <adore/params/ap_vehicle.h>
#include <adore/params/ap_trajectory_generation.h>
#include <adore/mad/fun_essentials.h>
#include <adore/mad/oderk4.h>

namespace adore
{
  namespace fun
  {
    /**
     * Road-relative coordinates.
     * A coordinate frame (s,n) is used, with s the progress along a road and n a lateral offset.
     */
    struct RoadCoordinates
    {
      public:
      double s0,s1,s2,s3;///longitudinal coordinate derivative 0 to 3
      double n0,n1,n2,n3;///lateral coordiante derivative 0 to 3
      bool valid;/**< if false, road coordinates may be out of bounds*/
    };

    /**
     * Converts vehicles state between road relative frame and Euclidean/vehicle frame
     */
    class RoadCoordinateConverter
    {
      private:
      adore::view::ALane* lfg_;
      VLB_OpenLoop model_;
      adoreMatrix<double,0,1> dx_;
      double a_;
      double b_;
      double L_;
      double h_;
      double g_;
      double mu_;
      double cf_;
      double cr_;
      double Iz_m_;
      double rho_;

      private:
      /**
       * Conversion from point-mass model to vehicle models with yaw dynamics.
       * The conversion is achieved by solving an initial value problem for heading and yaw rate.
       */
      class ZeroDynamicsModel: public adore::mad::AOdeModelWithOutput<double>
      {
        private:
            RoadCoordinateConverter* parent_;
            adore::mad::AScalarToN<double,4>* lon_trajectory_;
            adore::mad::AScalarToN<double,4>* lat_trajectory_;
            adoreMatrix<double,0,1> dx_full_;
            double s0_;
        public:
            /**
             * Constructor
             * Parametrized with parent (in order to compute coordinate conversions) and the trajectory for which zd is computed
             */
            ZeroDynamicsModel(RoadCoordinateConverter* parent,
                              adore::mad::AScalarToN<double,4>* lon_trajectory,
                              adore::mad::AScalarToN<double,4>* lat_trajectory,
                              double s0)
                              :parent_(parent),lon_trajectory_(lon_trajectory),lat_trajectory_(lat_trajectory),s0_(s0)
            {
              dx_full_.set_size(10,1);
            }

            /**
             * Differential equation for vehicle's zero dynamics.
             * Ode uses state x = [psi,omega], with #x=2 and output y=[Xr,Yr,psi,vx,vyr,omega,ax,delta,dax,ddelta], with #y=10
             */
            virtual void fh(double t, const adoreMatrix<double, 0, 1>& x_in, adoreMatrix<double, 0, 1>& dx_out,adoreMatrix<double,0,1>& y_out) override
            {
              auto s = lon_trajectory_->f(t);
              auto n = lat_trajectory_->f(t);

              RoadCoordinates r;
              r.s0=s(0) + s0_;r.s1=s(1);r.s2=s(2);r.s3=s(3);
              r.n0=n(0);r.n1=n(1);r.n2=n(2);r.n3=n(3);
              //retrieve full state
              PlanarVehicleState10d x_full;
              parent_->toVehicleState(r,x_in(0),x_in(1),x_full);
              // RoadCoordinates r_test = parent_->toRoadCoordinates(x_full);

              //full state derivative
              parent_->model_.f(t,x_full.data,dx_full_);
              //zero dynamics substate
              dx_out(0) = dx_full_(2);
              dx_out(1) = dx_full_(5);
              //save full state in output
              y_out = x_full.data;
            }
      };     

      public:
      /**
       * Constructor
       * @param lfg lane following view supplies road relative coordinate system
       * @param p vehicle parameters
       * @param tp trajectory generation parameters
       */
      RoadCoordinateConverter(adore::view::ALane* lfg,adore::params::APVehicle* p,adore::params::APTrajectoryGeneration* tp)
        :lfg_(lfg),model_(p)
      {
        updateParameters(p,tp);
      }
      bool isValid()
      {
        return lfg_->isValid();
      }
      /**
       * update and cache parameters
       */
      void updateParameters(adore::params::APVehicle* p,adore::params::APTrajectoryGeneration* tp)
      {
        model_.updateParameters(p);
        dx_.set_size(10,1);
        a_ = p->get_a();
        b_ = p->get_b();
        L_ = a_+b_;
        h_ = p->get_h();
        g_ = p->get_g();
        mu_ = p->get_mu();
        cf_ = p->get_cf();
        cr_ = p->get_cr();
        Iz_m_ = p->get_Iz_m();
        rho_ = tp->get_rho();
      }

      /**
       *  Compute road coordinates from vehicle state.
       *  @param x vehicle state in Euclidean coordinates
       *  @return state in road-relative coordinates 
       */
      RoadCoordinates toRoadCoordinates(const PlanarVehicleState10d& x, bool log=false)
      {
        const double rho = rho_;
        RoadCoordinates r;
        const double cpsi = std::cos(x.getPSI());
        const double spsi = std::sin(x.getPSI());
        model_.f(0.0,x.data,dx_);
        const double vxp = x.getvx();
        const double vyp = x.getvy() + x.getOmega() * rho;
        const double ax = x.getAx();
        const double ay = dx_(4,0) + dx_(5,0) * rho + x.getvx() * x.getOmega();
        if(log)
        {
        std::cout<<"toRoadCoordinates: ay_q="<<ay<<std::endl;
        std::cout<<"toRoadCoordinates: ay_cg="<<ay+dx_(5,0)*(b_-rho)<<std::endl;
        }
        const double px0 = x.getX() + cpsi * rho;
        const double py0 = x.getY() + spsi * rho;
        const double px1 = cpsi * vxp - spsi * vyp;
        const double py1 = spsi * vxp + cpsi * vyp;
        const double px2 = cpsi * ax - spsi * ay;
        const double py2 = spsi * ax + cpsi * ay;
        lfg_->toRelativeCoordinates(px0, py0, r.s0, r.n0);
        r.valid = lfg_->getSMin()<r.s0 && r.s0<lfg_->getSMax();
        const double theta = lfg_->getHeading(r.s0);
        const double ctheta = std::cos(theta);
        const double stheta = std::sin(theta);
        const double kappa0 = lfg_->getCurvature(r.s0,0);
        const double kappa1 = lfg_->getCurvature(r.s0,1);
        r.s1 = ( ctheta * px1 + stheta * py1) / (1.0 - r.n0 * kappa0);
        r.n1 = (-stheta * px1 + ctheta * py1);
        r.s2 = ( ( ctheta * px2 + stheta * py2)  + 2.0 * r.s1 * r.n1 * kappa0 + r.s1 * r.s1 * r.n0 * kappa1 ) / (1.0 - r.n0 * kappa0);
        r.n2 =   (-stheta * px2 + ctheta * py2) - r.s1 * r.s1 * kappa0 * (1.0 - r.n0 * kappa0);
        r.s3 = 0.0;///currently not required
        r.n3 = 0.0;///currently not required
        return r;
      }
      /**
       *  Compute vehicle state from road coordiantes
       *    The zero dynamic state has to be provided in order to allow the euqation to be solved.
       *    In this method, the zero dynamic state is represented by z=[psi,omega=d psi/dt]
       *  @param r the vehicle state in road-relative coordinates
       *  @param psi the yaw angle of the vehicle, when arriving at r
       *  @param omega the yaw rate of the vehicle, when arriving at r
       *  @param x the resulting full vehicle state with position in Euclidean cordinates. Returned by function.
       */
      void toVehicleState(const RoadCoordinates& r,double psi, double omega,PlanarVehicleState10d& x, bool log=false)
      {
        double px0,py0,pz0;
        lfg_->toEucledianCoordinates(r.s0,r.n0,px0,py0,pz0);
        const double theta = lfg_->getHeading(r.s0); 
        const double ctheta = std::cos(theta);
        const double stheta = std::sin(theta);
        const double kappa0 = lfg_->getCurvature(r.s0,0);
        const double kappa1 = lfg_->getCurvature(r.s0,1);
        const double pxr1 = r.s1 * (1.0-r.n0 * kappa0);
        const double pyr1 = r.n1;
        const double pxr2 = r.s2 * (1.0-r.n0 * kappa0) - 2.0 * r.s1 * r.n1 * kappa0 - r.s1 * r.s1 * r.n0 * kappa1;
        const double pyr2 = r.n2 + r.s1 * r.s1 * kappa0 * (1.0 - r.n0 * kappa0);
        const double px1 = ctheta * pxr1 - stheta * pyr1;
        const double py1 = stheta * pxr1 + ctheta * pyr1;
        const double px2 = ctheta * pxr2 - stheta * pyr2;
        const double py2 = stheta * pxr2 + ctheta * pyr2;
        const double cpsi = std::cos(psi);
        const double spsi = std::sin(psi);
        x.setX(px0 - cpsi * rho_);
        x.setY(py0 - spsi * rho_);
        x.setvx(  cpsi * px1 + spsi * py1);
        x.setvy(- spsi * px1 + cpsi * py1 - omega * rho_);
        x.setPSI(psi);
        x.setOmega(omega);
        x.setAx( cpsi * px2 + spsi * py2);
        const double ayq = -spsi * px2 + cpsi * py2;
        if(x.getvx()>0.01)
        {
          //linear tire force model
          const double k0 = -mu_*g_*cf_*b_/L_;
          const double k1 = -mu_*g_*cr_*a_/L_;
          const double fb = k1 * x.getvy()/x.getvx();
          const double fa = (ayq-(1.0-(rho_-b_)*b_/Iz_m_)*fb) / (1.0+(rho_-b_)*a_/Iz_m_);
          if(log)
          {
          std::cout<<"toVehicleState: ay_q="<<ayq<<std::endl;
          std::cout<<"toVehicleState: ay_cg="<<fa+fb<<std::endl;
          }
          const double delta = (x.getvy()+L_*omega)/x.getvx()-fa/k0;
          x.setDelta(delta);
        }
        else
        {
          //no low velocity model for formulation with lateral acceleration
          x.setDelta(0.0);
        }
        x.setDAx(0.0);//not implemented
        x.setDDelta(0.0);//not implemented
      }

      /**
       * Converts a decoupled plan in road coordinates into a 10d vehicle state trajectory.
       * The zero dynamics are initialized with psi0 and omega0 (expected states at the beginning of the plan).
       * @param lon_trajectory_rc the longitudinal plan
       * @param lat_trajectory_rc the lateral plan
       * @param integration_time the time parametrization of the resulting trajectory, also time steps of integration
       * @param s0 progress offset to be used if longitudinal plan starts at 0
       * @param psi0 initial heading of vehicle
       * @param omega0 initial yaw rate of vehicle
       */
      adoreMatrix<double,0,0> toVehicleStateTrajectory(adore::mad::AScalarToN<double,4>* lon_trajectory_rc,
                                                      adore::mad::AScalarToN<double,4>* lat_trajectory_rc,
                                                      const adoreMatrix<double,1,0>& integration_time,
                                                      double s0,double psi0,double omega0)
      {
        ZeroDynamicsModel zd(this,lon_trajectory_rc,lat_trajectory_rc,s0);
				adoreMatrix<double,0,0> Yode;//output
				Yode.set_size(10,integration_time.nc());
        adoreMatrix<double,2,1> x0;//the initial state
        x0(0) = psi0;
        x0(1) = omega0;

				//solve ode
				adore::mad::OdeRK4<double> ode4;
				ode4.solve_with_output(&zd,integration_time,x0,Yode);
        //compute d/dt ax and d/dt delta by finite differences
        for(int j=1;j<Yode.nc();j++)
        {
          int i=j-1;
          double dt = integration_time(j)-integration_time(i);
          Yode(8,i) = (Yode(6,j)-Yode(6,i))/dt;
          Yode(9,i) = (Yode(7,j)-Yode(7,i))/dt;
        }
        return Yode;
      }

    };
  }
}