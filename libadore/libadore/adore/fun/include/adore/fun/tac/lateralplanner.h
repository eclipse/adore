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
#include "anominalplanner.h"
#include "anominalplannerinformation.h"
#include "roadcoordinates.h"
#include <adore/view/alane.h>
#include <adore/mad/lq_oc_single_shooting.h>
#include <adore/mad/adoremath.h>
#include <adore/params/ap_longitudinal_planner.h>
#include <adore/params/ap_lateral_planner.h>
#include <adore/fun/setpointrequest.h>

namespace adore
{
  namespace fun
  {
    /**
     *  Plans lateral components of trajectories in a road relative coordinate system in 2 steps, using qpOASES.
     *  The longitudinal trajectory is assumed a given.
     *  In step 1 the lateral motion of a point mass is optimized.
     *  In step 2 the detailed vehicle dynamics is recovered by solving an initial value problem for the zero dynamics of the vehile model.
     *  Template parameter K defines number of time steps considered during planning, K*P is an increased number of time steps gained by interpolation.
     *  The optimization problem in step 1 is formulated as linear-quadratic with box constraints for integrator chains with input at the third derivative.
     *  All constraints and references for the optimization problems have to be externally supplied via the NominalPlannerInformationSet interface.
     */
    template<int K,int P>
    class LateralPlanner
    {
      public:
        static const int N = 3;//number of states
        static const int R = 1;//number of inputs
        // static const int K = 20;//number of optimization time steps
        // static const int P = 5;//number of post-interpolation steps
        typedef adore::mad::LQ_OC_single_shooting<N,R,K,P> TOffsetSolver;
        typedef NominalPlannerInformationSet<N+1,2> TInformationSet;
        typedef adore::mad::LLinearPiecewiseFunctionM<double,N+R> TLongitudinalPlan;
        typedef adore::mad::LLinearPiecewiseFunctionM<double,N+R> TLateralPlan;
      protected:
        TInformationSet info_;//reference and constraint defintion
      private://solvers, constraint/reference definitions, initial state
        TOffsetSolver offset_solver_;//solver for lateral planning
        double n0_,dn0_,ddn0_;//lateral initial state
        double T_[K+1];///time steps, incl. 0 at 0
        double T_end_;///end time of plan, defines planning horizon as [0,T_end_]
        RoadCoordinateConverter roadCoordinates_;
        adore::params::APLateralPlanner* aplat_;        
        adore::params::APTrajectoryGeneration* aptraj_;
        adore::params::APVehicle* apvehicle_;
        SetPointRequest spr_;///the result as a set-point request
        bool valid_;

        void init_offset_default_cost()
        {
          //default values for cost function of lateral optimization problem
          offset_solver_.wx()(0) = 6.0;
          offset_solver_.wx()(1) = 1.0;
          offset_solver_.wx()(2) = 1.0;
          offset_solver_.wu()(0) = 1.0;
          offset_solver_.wx_end()(0) = 0.0;
          offset_solver_.wx_end()(1) = 0.0;
          offset_solver_.wx_end()(2) = 0.0;
          offset_solver_.geps()(0) = 1.0e1;
          offset_solver_.geps()(1) = 1.0e1;
          offset_solver_.geps()(2) = 1.0e1;
          offset_solver_.ubeps()(0) = 0.05;
          offset_solver_.ubeps()(1) = 0.05;
          offset_solver_.ubeps()(2) = 0.0;
        }

        void initialize(double Tend)
        {
          T_end_ = Tend;
          double dt_int = Tend/(K*P);
				  adore::mad::linspace(0.0,Tend,T_,K+1);
          adore::mad::define_integrator_chain<double,N>(offset_solver_.Ad(),offset_solver_.Bd(),Tend/K);
          adore::mad::define_integrator_chain<double,N>(offset_solver_.Ad_p(),offset_solver_.Bd_p(),dt_int);


          {
            qpOASES::Options options;
            options.setToMPC();
            options.printLevel = qpOASES::PL_NONE;
            offset_solver_.getQProblem()->setOptions(options);
            offset_solver_.setMaxCPUTime(0.01);//default value
            offset_solver_.setSystemChanged(true);
          }
        }

        /**
         *  prepare_offset_computation - sets up the constraints for the lateral optimization problem
         */
        void prepare_offset_computation(TLongitudinalPlan* longitudinal_plan,double t0_offset,double s0_offset)
        {
          double ti,tj,s,ds;
          s = longitudinal_plan->fi(0.0,0) + s0_offset;
          ds = longitudinal_plan->fi(0.0,1);
          offset_solver_.x0()(0) = n0_;
          offset_solver_.x0()(1) = adore::mad::bound(info_.getLB(1,1,0.0,s,ds),dn0_,info_.getUB(1,1,0.0,s,ds));
          offset_solver_.x0()(2) = adore::mad::bound(info_.getLB(1,2,0.0,s,ds),ddn0_,info_.getUB(1,2,0.0,s,ds));
          // std::cout<<"LateralPlanner::prepare_offset_computation():"<<std::endl<<"s: n(lb, ref, ub) n'(lb, ref, ub) n''(lb, ref, ub)"<<std::endl;
          // std::cout<<"x0: ("<<offset_solver_.x0()(0)<<", "<<offset_solver_.x0()(1)<<", "<<offset_solver_.x0()(2)<<")"<<std::endl;
          for(int i=0;i<K;i++)
          {
            int j=i+1;
            ti = T_[i]+t0_offset;
            tj = T_[j]+t0_offset;
            s = longitudinal_plan->fi(T_[j],0) + s0_offset;
            ds = longitudinal_plan->fi(T_[j],1);

            offset_solver_.lbx()(0,i) = info_.getLB(1,0,tj,s,ds);
            offset_solver_.lbx()(1,i) = info_.getLB(1,1,tj,s,ds);
            offset_solver_.lbx()(2,i) = info_.getLB(1,2,tj,s,ds);
            offset_solver_.lbu_hard()(0,i) = info_.getLB(1,3,tj,s,ds);

            offset_solver_.ubx()(0,i) = info_.getUB(1,0,tj,s,ds);
            offset_solver_.ubx()(1,i) = info_.getUB(1,1,tj,s,ds);
            offset_solver_.ubx()(2,i) = info_.getUB(1,2,tj,s,ds);
            offset_solver_.ubu_hard()(0,i) = info_.getUB(1,3,tj,s,ds);

            info_.getReferenceIfAvailable(1,0,tj,s,ds,offset_solver_.y()(0,i));
            info_.getReferenceIfAvailable(1,1,tj,s,ds,offset_solver_.y()(1,i));
            info_.getReferenceIfAvailable(1,2,tj,s,ds,offset_solver_.y()(2,i));
            info_.getReferenceIfAvailable(1,3,tj,s,ds,offset_solver_.uset()(0,i));
            // std::cout<<s<<": ("<<offset_solver_.lbx()(0,i)<<", "<<offset_solver_.y()(0,i)<<", "<<offset_solver_.ubx()(0,i)<<")";
            // std::cout<<" ("<<offset_solver_.lbx()(1,i)<<", "<<offset_solver_.y()(1,i)<<", "<<offset_solver_.ubx()(1,i)<<")";
            // std::cout<<" ("<<offset_solver_.lbx()(2,i)<<", "<<offset_solver_.y()(2,i)<<", "<<offset_solver_.ubx()(2,i)<<")"<<std::endl;
          }
        }
        bool update_guard(double& target,double value)
        {
          if(target!=value)
          {
            target=value;
            return true;
          }
          return false;
        }
        void update_offset_parameters()
        {
          if(aplat_!=nullptr)
          {
            if(update_guard(offset_solver_.wx()(0),aplat_->getWeightPos())
            || update_guard(offset_solver_.wx()(1),aplat_->getWeightVel())
            || update_guard(offset_solver_.wx()(2),aplat_->getWeightAcc())
            || update_guard(offset_solver_.wu()(0),aplat_->getWeightJerk())
            || update_guard(offset_solver_.wx_end()(0),aplat_->getWeightEndPos())
            || update_guard(offset_solver_.wx_end()(1),aplat_->getWeightEndVel())
            || update_guard(offset_solver_.wx_end()(2),aplat_->getWeightEndAcc())
            || update_guard(offset_solver_.ubeps()(0),aplat_->getSlackPos())
            || update_guard(offset_solver_.ubeps()(1),aplat_->getSlackVel())
            || update_guard(offset_solver_.ubeps()(2),aplat_->getSlackAcc()))
            {
              offset_solver_.setSystemChanged(true);
            }
          }
        }
      public:
        LateralPlanner(adore::view::ALane* lfv,
                             adore::params::APLateralPlanner* aplat,
                             adore::params::APVehicle* apvehicle,
                             adore::params::APTrajectoryGeneration* aptrajectory)
                      :roadCoordinates_(lfv,apvehicle,aptrajectory),
                       aplat_(aplat),aptraj_(aptrajectory),apvehicle_(apvehicle)
        {
          init_offset_default_cost();
          initialize(10.0);
          valid_=false;
        }
        void setPlanningHorizon(double Tend)
        {
          initialize(Tend);
        }
        TInformationSet& getInformationSet()
        {
          return info_;
        }
        TOffsetSolver& getOffsetSolver()
        {
          return offset_solver_;
        } 
        RoadCoordinateConverter& getRoadCoordinateConverter()
        {
          return roadCoordinates_;
        }

        /**
         * compute - try to compute a trajectory according to given constraints and objective
         */
        void compute(const VehicleMotionState9d&  initial_state,TLongitudinalPlan* longitudinal_plan,double t0_offset,double s0_offset)
        {
          if(!roadCoordinates_.isValid())return; 

          auto rc = roadCoordinates_.toRoadCoordinates(initial_state);
          n0_ = rc.n0;
          dn0_ = rc.n1;
          ddn0_ = rc.n2;

          valid_ = false;

          //update constraints and reference
          info_.update(t0_offset,s0_offset,longitudinal_plan->fi(0,1));

          /*
            *  step 1 - lateral planning
            */
          update_offset_parameters();
          prepare_offset_computation(longitudinal_plan,t0_offset,s0_offset);
          offset_solver_.compute();
          offset_solver_.setEndTime(this->T_end_);

          if(offset_solver_.isSolved() && offset_solver_.isFeasible())
          {
            /*
             *  step 2 - full state trajectory
             */
            //find end of trajectory: first standstill
            double t_start = longitudinal_plan->limitLo();
            double t_end = longitudinal_plan->limitHi();
            double t_end_new = t_start + 0.1;
            double t_step = 0.01;
            double v_stop = 0.05;
            double a_stop = 0.001;
            while( (longitudinal_plan->f(t_end_new)(1)>v_stop
            ||      longitudinal_plan->f(t_end_new)(2)>a_stop)
            &&      t_end_new<t_end-t_step )t_end_new+=t_step;

            t_end_new = (std::min)(std::min(T_end_-aptraj_->getZDIntegrationStep(),
                                                  aptraj_->getZDIntegrationLength()),t_end_new);

            auto integration_time = adore::mad::sequence(0.0,
                                      aptraj_->getZDIntegrationStep(),
                                      t_end_new);//integration time

            auto trajectory = roadCoordinates_.toVehicleStateTrajectory(longitudinal_plan,
                                                                        &offset_solver_.result_fun(),
                                                                        integration_time,
                                                                        s0_offset,
                                                                        initial_state.getPSI(),
                                                                        initial_state.getOmega());
            spr_.setPoints.clear();
            spr_.append(integration_time,trajectory,aptraj_->getSetPointCount());
            spr_.setStartTime(t0_offset);
            valid_ = true;
          }
        }
        TLateralPlan& getLateralPlan()
        {
          return offset_solver_.result_fun();
        }


        double getTend() const
        {
          return T_end_;
        }
      
        /**
         *  hasValidPlan - return true, if a trajectory was computed, which satisfies given constraints
         */
        bool hasValidPlan()const 
        {
            return valid_;
        }
        /**
         * getSetPointRequest - return computed trajectory in the form of a SetPointRequest
         */
        const SetPointRequest* getSetPointRequest()const 
        {
            return &spr_;
        }
        /**
         *  getCPUTime - return the time require for trajectory planning in seconds
         */
        double getCPUTime()const 
        {
            return 0.0;
        }
    };
  }
}