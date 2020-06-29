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
#include <adore/view/alanefollowingview.h>
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
     *  Plans trajectories in a road relative coordinate system in 3 steps, using qpOASES.
     *  In step 1 the longitudinal motion of a point mass along the road coordinate s, the progress is optimized.
     *  In step 2 the lateral motion of a point mass is optimized.
     *  In step 3 the detailed vehicle dynamics is recovered by solving an initial value problem for the zero dynamics of the vehile model.
     *  Template parameter K defines number of time steps considered during planning, K*P is an increased number of time steps gained by interpolation.
     *  The optimization problems in step 1 and 2 are formulated as linear-quadratic with box constraints for integrator chains with input at the third derivative.
     *  All constraints and references for the optimization problems have to be externally supplied via the NominalPlannerInformationSet interface.
     */
    template<int K,int P>
    class DecoupledLFLCPlanner:public ANominalPlanner
    {
      public:
        static const int N = 3;//number of states
        static const int R = 1;//number of inputs
        // static const int K = 20;//number of optimization time steps
        // static const int P = 5;//number of post-interpolation steps
        typedef adore::mad::LQ_OC_single_shooting<N,R,K,P> TProgressSolver;
        typedef adore::mad::LQ_OC_single_shooting<N,R,K,P> TOffsetSolver;
        typedef NominalPlannerInformationSet<N+1,2> TInformationSet;
      protected:
        TInformationSet info_;//reference and constraint defintion
      private://solvers, constraint/reference definitions, initial state
        TProgressSolver progress_solver_;//solver for longitudinal planning
        TOffsetSolver offset_solver_;//solver for lateral planning
        double s0,ds0,dds0;//longitudinal initial state 
        double n0,dn0,ddn0;//lateral initial state
        double psi0,omega0;//internal dynamics initial state
        double t0;//observation time of initial state
        double T_[K+1];///time steps, incl. 0 at 0
        double T_end_;///end time of plan, defines planning horizon as [0,T_end_]
        RoadCoordinateConverter roadCoordinates_;
        adore::params::APLongitudinalPlanner* aplon_;
        adore::params::APLateralPlanner* aplat_;        
        adore::params::APTrajectoryGeneration* aptraj_;
        adore::params::APVehicle* apvehicle_;
        SetPointRequest spr_;///the result as a set-point request
        bool valid_;

        void init_progress_default_cost()
        {
          //default values for cost function of longitudinal optimization problem
          progress_solver_.wx()(0) = 2.0;
          progress_solver_.wx()(1) = 2.0;
          progress_solver_.wx()(2) = 5.0;
          progress_solver_.wu()(0) = 1.0;
          progress_solver_.wx_end()(0) = 0.0;
          progress_solver_.wx_end()(1) = 0.0;
          progress_solver_.wx_end()(2) = 0.0;
          progress_solver_.geps()(0) = 1.0e6;
          progress_solver_.geps()(1) = 1.0e6;
          progress_solver_.geps()(2) = 1.0e6;
          progress_solver_.ubeps()(0) = 0.5;//position slack
          progress_solver_.ubeps()(1) = 10.0;//velocity slack: should be admissible to violate speed limit by certain amount, if solution is otherwise infeasible
          progress_solver_.ubeps()(2) = 5.0;//acceleration slack
        }

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
          adore::mad::define_integrator_chain<double,N>(progress_solver_.Ad(),progress_solver_.Bd(),Tend/K);
          adore::mad::define_integrator_chain<double,N>(progress_solver_.Ad_p(),progress_solver_.Bd_p(),dt_int);
          adore::mad::define_integrator_chain<double,N>(offset_solver_.Ad(),offset_solver_.Bd(),Tend/K);
          adore::mad::define_integrator_chain<double,N>(offset_solver_.Ad_p(),offset_solver_.Bd_p(),dt_int);


          //optimization options
          {
            qpOASES::Options options;
            options.setToMPC();
            options.printLevel = qpOASES::PL_NONE;
            progress_solver_.getQProblem()->setOptions(options);
            progress_solver_.setMaxCPUTime(0.01);//default value
            progress_solver_.setSystemChanged(true);
          }
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
         *  prepare_progress_computation - sets up the constraints for the longitudinal optimization problem
         */
        void prepare_progress_computation()
        {
          progress_solver_.x0()(0) = 0.0;//relative start position
          progress_solver_.x0()(1) = adore::mad::bound(info_.getLB(0,1,t0,s0,ds0),ds0,info_.getUB(0,1,t0,s0,ds0));
          progress_solver_.x0()(2) = adore::mad::bound(info_.getLB(0,2,t0,s0,ds0),dds0,info_.getUB(0,2,t0,s0,ds0));

          double s_estimate = s0;
          double ds_estimate = progress_solver_.x0()(1);
          double ti,tj;

          // Forward Pass
          for(int i=0;i<K;i++)
          {
            int j=i+1;
            ti = T_[i] + t0; 
            tj = T_[j] + t0;

            s_estimate += ds_estimate * (tj-ti);
            double s_reference;
            if( info_.getReferenceIfAvailable(0,0,tj,s_estimate,ds_estimate,s_reference))
            {
              s_estimate = (std::min)(s_estimate,s_reference);
            }
            else
            {
              s_reference = s_estimate;
            }
            
            double dds_max = info_.getUB(0,2,tj,s_estimate,ds_estimate);
            ds_estimate += dds_max * (tj-ti);
            double ds_reference;
            if( info_.getReferenceIfAvailable(0,1,tj,s_estimate,ds_estimate,ds_reference))
            {
              ds_estimate = (std::min)(ds_estimate,ds_reference);
            }
            else
            {
              ds_reference = ds_estimate;
            }
            
            progress_solver_.lbx()(0,i) = info_.getLB(0,0,tj,s_estimate,ds_estimate)-s0;//s lower bound
            progress_solver_.lbx()(1,i) = info_.getLB(0,1,tj,s_estimate,ds_estimate);//ds lower bound
            progress_solver_.lbx()(2,i) = info_.getLB(0,2,tj,s_estimate,ds_estimate);//dds lower bound
					  progress_solver_.lbu_hard()(0,i) = info_.getLB(0,3,tj,s_estimate,ds_estimate);//ddds lower bound

            progress_solver_.ubx()(0,i) = info_.getUB(0,0,tj,s_estimate,ds_estimate)-s0;//s upper bound
            progress_solver_.ubx()(1,i) = info_.getUB(0,1,tj,s_estimate,ds_estimate);//ds upper bound
            progress_solver_.ubx()(2,i) = info_.getUB(0,2,tj,s_estimate,ds_estimate);//dds upper bound
					  progress_solver_.ubu_hard()(0,i) = info_.getUB(0,3,tj,s_estimate,ds_estimate);//ddds upper bound

            progress_solver_.y()(0,i) = adore::mad::bound(progress_solver_.lbx()(0,i),
                                                        s_reference-s0,
                                                        progress_solver_.ubx()(0,i));
            progress_solver_.y()(1,i) = adore::mad::bound(progress_solver_.lbx()(1,i),
                                                        ds_reference,
                                                        progress_solver_.ubx()(1,i));
          }

          // Backward Pass
          for( int j=K-1;j>0;j-- )
          {
            int i = j-1;
            //prevent backwards motion in reference:
            progress_solver_.y()(0,i) =  adore::mad::bound(progress_solver_.lbx()(0,i),
                                                std::min(progress_solver_.y()(0,i), progress_solver_.y()(0,j)),
                                                progress_solver_.ubx()(0,i));
          }

        }

        /**
         *  prepare_offset_computation - sets up the constraints for the lateral optimization problem
         */
        void prepare_offset_computation()
        {
          offset_solver_.x0()(0) = n0;
          offset_solver_.x0()(1) = adore::mad::bound(info_.getLB(1,1,t0,s0,ds0),dn0,info_.getUB(1,1,t0,s0,ds0));
          offset_solver_.x0()(2) = adore::mad::bound(info_.getLB(1,2,t0,s0,ds0),ddn0,info_.getUB(1,2,t0,s0,ds0));
          double ti,tj,tj_rel,s,ds;
          for(int i=0;i<K;i++)
          {
            int j=i+1;
            ti = T_[i]+t0;
            tj = T_[j]+t0;
            tj_rel = adore::mad::bound(progress_solver_.result_fun().limitLo(),
                                      tj-t0,
                                      progress_solver_.result_fun().limitHi());
            s = progress_solver_.result_fun().fi(tj_rel,0) + s0;
            ds = progress_solver_.result_fun().fi(tj_rel,1);

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
        void update_progress_parameters()
        {
          if(aplon_!=nullptr)
          {
            if(update_guard(progress_solver_.wx()(0),aplon_->getWeightPos())
            || update_guard(progress_solver_.wx()(1),aplon_->getWeightVel())
            || update_guard(progress_solver_.wx()(2),aplon_->getWeightAcc())
            || update_guard(progress_solver_.wu()(0),aplon_->getWeightJerk())
            || update_guard(progress_solver_.wx_end()(0),aplon_->getWeightEndPos())
            || update_guard(progress_solver_.wx_end()(1),aplon_->getWeightEndVel())
            || update_guard(progress_solver_.wx_end()(2),aplon_->getWeightEndAcc())
            || update_guard(progress_solver_.ubeps()(0),aplon_->getSlackPos())
            || update_guard(progress_solver_.ubeps()(1),aplon_->getSlackVel())
            || update_guard(progress_solver_.ubeps()(2),aplon_->getSlackAcc()))
            {
              progress_solver_.setSystemChanged(true);
            }
          }
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
        DecoupledLFLCPlanner(adore::view::ALaneFollowingView* lfv,
                             adore::params::APLongitudinalPlanner* aplon,
                             adore::params::APLateralPlanner* aplat,
                             adore::params::APVehicle* apvehicle,
                             adore::params::APTrajectoryGeneration* aptrajectory)
                      :roadCoordinates_(lfv,apvehicle,aptrajectory),aplon_(aplon),
                       aplat_(aplat),aptraj_(aptrajectory),apvehicle_(apvehicle)
        {
          init_progress_default_cost();
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
        TProgressSolver& getProgressSolver()
        {
          return progress_solver_;
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
        virtual void compute(const VehicleMotionState9d&  initial_state)override
        {
          if(!roadCoordinates_.isValid())return; 

          auto rc = roadCoordinates_.toRoadCoordinates(initial_state);
          s0 = rc.s0;ds0 = rc.s1;dds0 = rc.s2;
          n0 = rc.n0;dn0 = rc.n1;ddn0 = rc.n2;
          psi0 = initial_state.getPSI();
          omega0 = initial_state.getOmega();
          t0 = initial_state.getTime();

          valid_ = false;

          //update constraints and reference
          info_.update(initial_state.getTime(),s0,ds0);

          /*
            *  step 1 - longitudinal planning
            */
          update_progress_parameters();
          prepare_progress_computation();
          progress_solver_.compute();
          progress_solver_.setEndTime(this->T_end_);
          if(progress_solver_.isSolved() && progress_solver_.isFeasible())
          {
            /*
              *  step 2 - lateral planning
              */
            update_offset_parameters();
            prepare_offset_computation();
            offset_solver_.compute();
            offset_solver_.setEndTime(this->T_end_);

            if(offset_solver_.isSolved() && offset_solver_.isFeasible())
            {
              /*
                *  step 3 - full state trajectory
                */
              auto integration_time = adore::mad::sequence(0.0,
                                        aptraj_->getZDIntegrationStep(),
                                        (std::min)(T_end_-aptraj_->getZDIntegrationStep(),
                                                   aptraj_->getZDIntegrationLength()));//integration time
              auto trajectory = roadCoordinates_.toVehicleStateTrajectory(&progress_solver_.result_fun(),
                                                                          &offset_solver_.result_fun(),
                                                                          integration_time,
                                                                          s0,
                                                                          initial_state.getPSI(),
                                                                          initial_state.getOmega());
              spr_.setPoints.clear();
              spr_.append(integration_time,trajectory,aptraj_->getSetPointCount());
              spr_.setStartTime(initial_state.getTime());
              valid_ = true;
            }
          }
        }
        /**
         *  hasValidPlan - return true, if a trajectory was computed, which satisfies given constraints
         */
        virtual bool hasValidPlan()const override
        {
            return valid_;
        }
        /**
         * getSetPointRequest - return computed trajectory in the form of a SetPointRequest
         */
        virtual const SetPointRequest* getSetPointRequest()const override
        {
            return &spr_;
        }
        /**
         *  getCPUTime - return the time require for trajectory planning in seconds
         */
        virtual double getCPUTime()const override
        {
            return 0.0;
        }
    };
  }
}