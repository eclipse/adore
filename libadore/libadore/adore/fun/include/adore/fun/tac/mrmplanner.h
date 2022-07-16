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
#include "lateralplanner.h"
#include "informationsetpostprocessing.h"

namespace adore
{
  namespace fun
  {
     /**
      * MRMPlanner uses a fixed acceleration profile and LateralPlanner to compute a braking trajectory.
      * The fixed acceleration profile consists of two phases with different accelerations
      * and may be parametrized by tstall, the duration of phase 0, astall, the acceleration of phase 0 and amin, the acceleration of phase 1.
      * The lateral motion is constrained to the current lane.
      * If the trajectory with two phases (delayed action) is invalid, a trajectory with immediate deceleration with amin will be attempted.
      */
    template<int K,int P>
    class MRMPlanner:public ANominalPlanner
    {
        public:
        static const int N = 3;//number of states
        static const int R = 1;//number of inputs
        typedef adore::mad::LLinearPiecewiseFunctionM<double,N+R> TPartialPlan;
        typedef InformationSetPostProcessing<4,2> TPostProcessConstraints;
        private:
        LateralPlanner<K,P> lateralPlanner_;
        TPartialPlan longitudinal_plan_;
        TPostProcessConstraints postproc_;
        RoadCoordinateConverter roadCoordinates_;
        adore::params::APTrajectoryGeneration* aptraj_;
        adore::params::APVehicle* apvehicle_;
        double jmax_;/**< maximum absolute longitudinal accelration*/  
        double tstall_;/**< delay, during which astall_ is applied. after tstall_, amin_ is applied. set tstall to 0 to gain immediate braking maneuver*/ 
        double astall_;/**< initial, soft deceleration value*/ 
        double amin_;/**< delayed, hard deceleration value*/  
        bool second_attempt_;/**< if trajectory with tstall_ is invalid and second_attempt_ is true, a trajectory with immediate amin with computed*/ 
        bool longitudinal_plan_valid_;/**< is true, if a valid longitudinal trajectory is available*/
        public:// setters for parameters
        void setJMax(double value){jmax_=value;}
        void setTStall(double value){tstall_=value;}
        void setAStall(double value){astall_=value;}
        void setAMin(double value){amin_=value;}
        void setSecondAttempt(bool value){second_attempt_=value;}

        public:
        MRMPlanner(adore::view::ALane* lfv,
                             adore::params::APLateralPlanner* aplat,
                             adore::params::APVehicle* apvehicle,
                             adore::params::APTrajectoryGeneration* aptrajectory)
                      :lateralPlanner_(lfv,aplat,apvehicle,aptrajectory),
                       roadCoordinates_(lfv,apvehicle,aptrajectory),
                       aptraj_(aptrajectory),apvehicle_(apvehicle)
        {
            jmax_ = 1.0;
            amin_ = -3.0;
            tstall_ = 1.0;
            astall_ = -0.25;
            second_attempt_ = true;
            //initialize the longitudinal plan function [t0,tend]->(s,s',s'',s''')
            longitudinal_plan_.getData().set_size(5,K*P+1);            
            dlib::set_rowm(longitudinal_plan_.getData(),0) = adore::mad::linspace(0.0,lateralPlanner_.getTend(),K*P+1);            
            longitudinal_plan_valid_ = false;
        }

        TPostProcessConstraints::TInformationSet& getInformationSet()
        {
          return postproc_.getInformationSet();
        }

        ///minimum acceleration that can be achieved before inverting jerk to end with v=0 and a=0
        double amin_medium_brake_trapezoidal(double v0, double a0, double jmin)const
        {
          return -std::sqrt(0.5*a0*a0-v0*jmin);
        }
        ///times required for ramp up during short brake
        void t_short_brake_trapezoidal(double v0,double a0,double jmin, double t3)
        {
          t3 = std::sqrt(a0/jmin-2.0*v0/jmin)-a0/jmin;
        }
        ///times required for ramp down and ramp up during medium brake
        void t_medium_brake_trapezoidal(double v0,double a0, double jmin, double& t1, double &t3)const
        {
          const double amin = amin_medium_brake_trapezoidal(v0,a0,jmin);
          t1 = (amin-a0)/jmin;
          t3 = amin/jmin;
        }
        ///times required for ramp down, constant and ramp up during long brake
        void t_long_brake_trapezoidal(double v0,double a0,double amin,double jmin,double& t1, double& t2,double &t3)const
        {
          t1 = (amin-a0)/jmin;
          t3 = amin/jmin;
          const double dv1 = 0.5 * t1 * (amin-a0) + t1 * a0;
          const double dv3 = 0.5 * t3 * amin;
          t2 = -(v0+dv1+dv3)/amin;
        }
        ///
        int brake_case_trapezoidal(double v0,double a0,double amin,double jmin)const
        {
          a0 = std::min(0.0,a0);
          if(-(a0/jmin)*(0.5*a0)>=v0)return 0;             //short brake: reaching v=0 before a0 can be reduced to zero
          if(amin_medium_brake_trapezoidal(v0,a0,jmin)>=amin)return 1; //medium brake: ramp down acceleration until a1>amin, ramp up to zero
          return 2;                                       //long brake: ramp down acceleration until amin, constant acceleration, ramp up to zero
        }
        /// general parameters
        void brake_params_trapezoidal(double v0,double a0,double amin,double jmin,double& a1,double& t1,double& t2,double& t3)
        {
          switch(brake_case_trapezoidal(v0,a0,amin,jmin))
          {
            case 0:
              a1 = a0;
              t1 = 0.0;
              t2 = 0.0;
              t_short_brake_trapezoidal(v0,a0,jmin,t3);
              break;
            case 1:
              a1 = amin_medium_brake_trapezoidal(v0,a0,jmin);
              t_medium_brake_trapezoidal(v0,a0,jmin,t1,t3);
              t2 = 0.0;
              break;
            case 2:
              a1 = amin;
              t_long_brake_trapezoidal(v0,a0,amin,jmin,t1,t2,t3);
              break;
          }
        }
        double a_brake_trapezoidal(double t,double a0,double a1,double jmin,double t1,double t2,double t3)
        {
          if(t<=t1)
          {
            return a0 + t*jmin;
          }
          if(t<=t1+t2)
          {
            return a1;
          }
          if(t<=t1+t2+t3)
          {
            return a1-jmin*(t-t1-t2);
          }
          return 0.0;
        }
        double j_brake_trapezoidal(double t,double jmin,double t1,double t2,double t3)
        {
          if(t<=t1)
          {
            return jmin;
          }
          if(t<=t1+t2)
          {
            return 0.0;
          }
          if(t<=t1+t2+t3)
          {
            return -jmin;
          }
          return 0.0;
        }

        /**
         * compute - try to compute a trajectory according to given constraints and objective
         */
        virtual void compute(const VehicleMotionState9d& initial_state)
        {
          if(!roadCoordinates_.isValid())return; 

          auto rc = roadCoordinates_.toRoadCoordinates(initial_state);
          double s0 = rc.s0;
          double ds0 = rc.s1;
          double dds0 = adore::mad::bound(amin_,rc.s2,0.0);//the current acceleration is set to 0 if a>0 to achieve a quicker reaction 
          double s0_offset = s0;
          double t0_offset = initial_state.getTime();

          //fill the longitudinal plan function [t0,tend]->(s,s',s'',s''')
          //(longitudinal plan only depends on initial conditions)
          double s = s0-s0_offset;
          double ds = ds0;
          double dds = dds0;
          longitudinal_plan_.getData()(1,0) = s;
          longitudinal_plan_.getData()(2,0) = ds;
          longitudinal_plan_.getData()(3,0) = dds;

          double t1,t2,t3,a1;
          brake_params_trapezoidal(ds0,dds0,amin_,-jmax_,a1,t1,t2,t3);
          int version = 1;

          for(int k=1;k<longitudinal_plan_.getData().nc();k++)
          {
            double ti = longitudinal_plan_.getData()(0,k-1);
            double tk = longitudinal_plan_.getData()(0,k);
            double dt = tk-ti;
            double jset;
            if(version==0)
            {
              double aset = ds>0.0?(ti<tstall_?astall_:amin_):0.0;
              jset = adore::mad::bound(-jmax_,(aset-dds)/dt,jmax_);
              s = std::max(s,s+dt*ds+0.5*dt*dt*dds+1.0/6.0*dt*dt*dt*jset);
              ds = std::max(0.0,ds+dds*dt+0.5*dt*dt*jset);
              dds = dds + dt * jset;
            }
            else if(version==1)
            {
              jset = j_brake_trapezoidal(ti,-jmax_,t1,t2,t3);
              s = std::max(s,s+dt*ds+0.5*dt*dt*dds+1.0/6.0*dt*dt*dt*jset);
              ds = std::max(0.0,ds+dds*dt+0.5*dt*dt*jset);
              dds = a_brake_trapezoidal(tk,dds0,a1,-jmax_,t1,t2,t3);
            }
            longitudinal_plan_.getData()(1,k) = s;
            longitudinal_plan_.getData()(2,k) = ds;
            longitudinal_plan_.getData()(3,k) = dds;
            longitudinal_plan_.getData()(4,k-1) = jset;
            longitudinal_plan_.getData()(4,k) = 0.0;
          }
          //postprocess constraints for longitudinal plan
          postproc_.getInformationSet().update(t0_offset,s0_offset,ds0);
          longitudinal_plan_valid_ = postproc_.isLongitudinalPlanValid(t0_offset,s0_offset,&longitudinal_plan_);
          

          if(longitudinal_plan_valid_)
          {
            //compute lateral plan
            lateralPlanner_.compute(initial_state,&longitudinal_plan_,t0_offset,s0_offset);
          }

          if(!hasValidPlan() && second_attempt_)
          {
              second_attempt_ = false;
              double tmp = tstall_;
              tstall_ = 0.0;
              compute(initial_state);
              second_attempt_ = true;
              tstall_ = tmp;
          }
        }

        TPartialPlan& getLongitudinalPlan()
        {
            return longitudinal_plan_;
        }
        TPartialPlan& getLateralPlan()
        {
            return lateralPlanner_.getLateralPlan();
        }
        /**
         * provide reference to lateralPlanner for manipulation of constraints and references
         */
        LateralPlanner<K,P>& getOffsetSolver()
        {
            return lateralPlanner_;
        }
        /**
         *  hasValidPlan - return true, if a trajectory was computed, which satisfies given constraints
         */
        virtual bool hasValidPlan()const 
        {
            return longitudinal_plan_valid_ && lateralPlanner_.hasValidPlan();
        }
        /**
         * getSetPointRequest - return computed trajectory in the form of a SetPointRequest
         */
        virtual const SetPointRequest* getSetPointRequest()const
        {
            return lateralPlanner_.getSetPointRequest();
        }
        /**
         *  getCPUTime - return the time require for trajectory planning in seconds
         */
        virtual double getCPUTime()const 
        {
            return 0.0;
        }
    };
  }
}