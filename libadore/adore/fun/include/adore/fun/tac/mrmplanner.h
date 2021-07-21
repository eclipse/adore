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


        /**
         * compute - try to compute a trajectory according to given constraints and objective
         */
        virtual void compute(const VehicleMotionState9d& initial_state)
        {
          if(!roadCoordinates_.isValid())return; 

          auto rc = roadCoordinates_.toRoadCoordinates(initial_state);
          double s0 = rc.s0;
          double ds0 = rc.s1;
          double dds0 = std::min(0.0,rc.s2);//the current acceleration is set to 0 if a>0 to acchieve a quicker reaction 
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
          for(int k=1;k<longitudinal_plan_.getData().nc();k++)
          {
            double ti = longitudinal_plan_.getData()(0,k-1);
            double tk = longitudinal_plan_.getData()(0,k);
            double dt = tk-ti;
            double aset = ds>0.0?(ti<tstall_?astall_:amin_):0.0;
            double jset = adore::mad::bound(-jmax_,(aset-dds)/dt,jmax_);
            s = std::max(s,s+dt*ds+0.5*dt*dt*dds+1.0/6.0*dt*dt*dt*jset);
            ds = std::max(0.0,ds+dds*dt+0.5*dt*dt*jset);
            dds = dds + dt * jset;
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