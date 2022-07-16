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
#include <adore/view/alimitlineenroute.h>
#include <adore/view/anavigationgoalview.h>
#include <adore/view/aconflictpoint.h>
#include <adore/view/alane.h>
#include <adore/fun/tac/anominalplanner.h>
#include <adore/params/afactory.h>
#include <adore/mad/adoremath.h>
#include <set>

namespace adore
{
  namespace fun
  {
    /**
     * CurvatureSpeedLimit - limit speed during longitudinal planning in order to gain bounded lateral acceleration in curve
     * |ds(t)^2 * kappa(s(t))| < ay_max
     */
    class CurvatureSpeedLimit:public ANominalConstraint
    {
      private:
        adore::view::ALane* lfv_;
        adore::params::APLongitudinalPlanner* plon_;
        double lon_ayub_;
        double vmin_ay_;
      public:
        CurvatureSpeedLimit(adore::view::ALane* lfv,
                            adore::params::APLongitudinalPlanner* plon)
                            :lfv_(lfv),plon_(plon){}

        virtual double getValue(double t,double s,double ds)const override
        {
          double kappa = (std::abs)(lfv_->getCurvature(s,0));
          if(kappa<1e-6)return 10000.0;
          return (std::max)((std::sqrt)(lon_ayub_ / kappa),vmin_ay_);
        }
        virtual void update(double t0,double s0,double ds0) override
        {
          lon_ayub_ = plon_->getAccLatUB();
          vmin_ay_ = plon_->getAccLatUB_minVelocity();
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 1;
        }
    };
    /**
     * CurvatureSpeedLimitSmooth - applies some averaging to CurvatureSpeedLimit
     */
    class CurvatureSpeedLimitSmooth:public ANominalConstraint
    {
      private:
        CurvatureSpeedLimit csl_;
        double dt;
        int km;
        int kp;
      public:
        CurvatureSpeedLimitSmooth(adore::view::ALane* lfv,
                            adore::params::APLongitudinalPlanner* plon)
                            :csl_(lfv,plon)
        {
          dt = 0.5;
          km = 5;
          kp = 15;
        }

        virtual double getValue(double t,double s,double ds)const override
        {
          ds = std::max(0.0,ds);
          double value = csl_.getValue(t,s,ds);
          for(int i=1;i<=kp;i++)value+=csl_.getValue(t,s+ds*dt*i,ds);
          for(int i=1;i<=km;i++)value+=csl_.getValue(t,s-ds*dt*i,ds);
          value = value / (double)(kp+km+1);
          return value;
        }
        virtual void update(double t0,double s0,double ds0) override
        {
          csl_.update(t0,s0,ds0);
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 1;
        }
    };
    /**
     * CurvatureSpeedLimitPredict - applies comfort deceleration curve predictively
     */
    class CurvatureSpeedLimitPredict:public ANominalConstraint
    {
      private:
        CurvatureSpeedLimit csl_;
        double sstep_;
        double smax_;
        double s_curvature_lookahead_;
        double a;
        adore::params::APLongitudinalPlanner* plon_;

      public:
        CurvatureSpeedLimitPredict(adore::view::ALane* lfv,
                                  adore::params::APLongitudinalPlanner* plon)
                            :csl_(lfv,plon),plon_(plon)
        {
          sstep_ = 1.0;
          smax_ = 200.0;
          s_curvature_lookahead_ = 10.0;
        }

        virtual double getValue(double t,double s,double ds)const override
        {
          ds = std::max(0.0,ds);
          const double t_lah = ds/a;
          const double s_lah = (std::min)(smax_,0.5*a*t_lah*t_lah);//breaking distance at current speed
          double vmin = csl_.getValue(t,s,ds);//init with current limit
          for(double si = s+sstep_;si<=s+s_lah;si+=sstep_)
          {
            double v1 = csl_.getValue(t,si,ds);
            for(double sj = si + sstep_;sj<=si+s_curvature_lookahead_;sj+=sstep_)
            {
              v1 = (std::min)(v1,csl_.getValue(t,sj,ds));
            }
            const double v0 = std::sqrt(v1*v1+2.0*(si-s)*a);//initial velocity of breaking curve arriving at si with v1
            vmin = (std::min)(vmin,v0);
          }
          return std::max(0.0,vmin);
        }
        virtual void update(double t0,double s0,double ds0) override
        {
          a = -plon_->getComfortAccLB();
          csl_.update(t0,s0,ds0);
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 1;
        }
    };
    /**
     * DontDriveBackwards - constrain speed to be positive
     */
    class DontDriveBackwards:public ANominalConstraint
    {
      public:
        DontDriveBackwards(){}
        virtual double getValue(double t,double s,double ds)const override
        {
          return 0.0;
        }
        virtual void update(double t0,double s0,double ds0) override {}
        virtual ConstraintDirection getDirection() override
        {
          return LB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 1;
        }
    };
    /**
     * LaneWidthSpeedLimitLFV - constrain speed according to available space on lane
     */
    class LaneWidthSpeedLimitLFV:public ANominalConstraint
    {
      private:
        adore::view::ALane* lfv_;
        const adore::params::APLongitudinalPlanner* plon_;
        double min_width_stop_;
        double min_width_slow_;
        double min_width_slow_speed_;
        double min_width_fast_;
        double min_width_fast_speed_;
      public:
        LaneWidthSpeedLimitLFV(adore::view::ALane* lfv,
                               const adore::params::APLongitudinalPlanner* plon)
                            :lfv_(lfv),plon_(plon){}

        virtual double getValue(double t,double s,double ds)const override
        {
          double d = lfv_->getOffsetOfLeftBorder(s)-lfv_->getOffsetOfRightBorder(s);
          if(d>min_width_fast_)
          {
            return 10000.0;
          }
          else if(d>min_width_slow_)
          {
            return min_width_slow_speed_
            + (d-min_width_slow_)
              /(min_width_fast_-min_width_slow_)
              *(min_width_fast_speed_-min_width_slow_speed_);
          }
          else
          {
            return std::max(0.0,(d-min_width_stop_)
                                /(min_width_slow_-min_width_stop_)
                                *min_width_slow_speed_);
          }
          
        }
        virtual void update(double t0,double s0,double ds0) override
        {
          min_width_stop_ = plon_->getMinWidthStop();
          min_width_slow_ = plon_->getMinWidthSlow();
          min_width_slow_speed_ = plon_->getMinWidthSlowSpeed();
          min_width_fast_ = plon_->getMinWidthFast();
          min_width_fast_speed_ = plon_->getMinWidthFastSpeed();
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 1;
        }
    };
    /**
     * StopAtBottleneckLFV - create position upper bound, if lane contains a bottleneck too narrow to navigate 
     */
    class StopAtBottleneckLFV:public ANominalConstraint
    {
      private:
        adore::view::ALane* lfv_;
        const adore::params::APLongitudinalPlanner* plon_;
        const adore::params::APVehicle* pv_;
        const adore::params::APTrajectoryGeneration* pgen_;
        const adore::params::APTacticalPlanner* ptac_;
        double distance_;
        double ds_sample_;
      public:
        StopAtBottleneckLFV(adore::view::ALane* lfv, 
                            const adore::params::APLongitudinalPlanner* plon,
                            const adore::params::APVehicle* pv,
                            const adore::params::APTrajectoryGeneration* pgen,
                            const adore::params::APTacticalPlanner* ptac)
                            :lfv_(lfv),plon_(plon),pv_(pv),pgen_(pgen),ptac_(ptac)
        {
          ds_sample_ = 1.0;
        }
        virtual double getValue(double t,double s,double ds)const override
        {
          return distance_;
        }
        virtual void update(double t0,double s0,double ds0) override 
        {
          distance_ = lfv_->getSMax()
                            - pv_->get_a()-pv_->get_b()-pv_->get_c()
                            + pgen_->get_rho();
          double dmin = plon_->getMinWidthStop();
          for(double s = s0;s<lfv_->getSMax();s+=ds_sample_)
          {
            double d = lfv_->getOffsetOfLeftBorder(s)-lfv_->getOffsetOfRightBorder(s);
            if(d<=dmin)
            {
              distance_ = s - pv_->get_a()-pv_->get_b()-pv_->get_c()
                            + pgen_->get_rho()-ptac_->getFrontSGap();
              break;
            }
          }
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        { 
          return 0;
        }
    };


    /**
     * BreakAtHorizon - plan to come to a standstill at the end of the lane following view
     */
    class BreakAtHorizon:public ANominalConstraint
    {
      private:
        adore::view::ALane* lfv_;
        adore::params::APVehicle* pvehicle_;
        adore::params::APTrajectoryGeneration* pgen_;
        double distance_;
      public:
        BreakAtHorizon(adore::view::ALane* lfv,adore::params::APVehicle* pvehicle,adore::params::APTrajectoryGeneration* pgen):
          lfv_(lfv),pvehicle_(pvehicle),pgen_(pgen){}
        virtual double getValue(double t,double s,double ds)const override
        {
          return distance_;
        }
        virtual void update(double t0,double s0,double ds0) override 
        {
          distance_ = lfv_->getSMax()-(pvehicle_->get_a()+pvehicle_->get_b()+pvehicle_->get_c()-pgen_->get_rho());
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    };



    /**
     *  LateralAccelerationConstraint - limit lateral acceleration during lateral planning
     *  limitations are set by maximum curvature (steering angle limitation) and maximum lateral acceleration compensating lateral acceleration of road-coordinates
     */
    class LateralAccelerationConstraint:public ANominalConstraint
    {
      private:
        adore::view::ALane* lfv_;
        adore::params::APLateralPlanner* p_;
        double ay_ub_;
        double ay_lb_;
        double curvature_ub_;
        double curvature_lb_;
        ConstraintDirection d_;
      public:
        LateralAccelerationConstraint(adore::view::ALane* lfv,
                                      adore::params::APLateralPlanner* p,
                                      ConstraintDirection d)
                            :lfv_(lfv),p_(p),d_(d){}

        virtual double getValue(double t,double s,double ds)const override
        {
          double kappa = lfv_->getCurvature(s,0);
          ds = (std::max)(0.0,ds);
          double ayset = -kappa*ds*ds; 
          return ayset + (d_==LB?(std::max)(curvature_lb_*ds*ds,ay_lb_)
                               :(std::min)(curvature_ub_*ds*ds,ay_ub_));
        }
        virtual void update(double t0,double s0,double ds0) override
        {
          ay_lb_ = p_->getAccLB();
          ay_ub_ = p_->getAccUB();
          curvature_ub_ = p_->getCurvatureUB();
          curvature_lb_ = p_->getCurvatureLB();
        }
        virtual ConstraintDirection getDirection() override
        {
          return d_;
        }
        virtual int getDimension() override
        {
          return 1;
        }
        virtual int getDerivative() override
        {
          return 2;
        }
    };

    /**
     *   HeadingConstraint - constrain the deviation from heading of the road during lateral planning
     */
    class HeadingConstraint:public ANominalConstraint
    {
      private:
        adore::params::APLateralPlanner* p_;
        double heading_ub_;
        double heading_lb_;
        ConstraintDirection d_;
      public:
        HeadingConstraint(adore::params::APLateralPlanner* p,
                          ConstraintDirection d)
                            :p_(p),d_(d){}

        virtual double getValue(double t,double s,double ds)const override
        {
          ds = std::max(0.001,ds);
          return ds * (std::tan)(d_==LB?heading_lb_:heading_ub_);
        }
        virtual void update(double t0,double s0,double ds0) override
        {
          heading_lb_ = p_->getRelativeHeadingLB();
          heading_ub_ = p_->getRelativeHeadingUB();
        }
        virtual ConstraintDirection getDirection() override
        {
          return d_;
        }
        virtual int getDimension() override
        {
          return 1;
        }
        virtual int getDerivative() override
        {
          return 1;
        }
    };

    /**
     *  LFVSpeedLimit - maintain speed limit of lane following view during longitudinal planning
     *  HeD20210915: relaxation for speedlimit: the initial speed may be violating speed limit: 
     *               manual control, other planners with different constraints, switching of 
     *               non-constant traffic control systems.
     *               To avoid infeasible problem, speed limit is relaxed with initial condition
     *               and breaking curve.  
     */
    class LFVSpeedLimit:public ANominalConstraint
    {
      private:
        adore::view::ALane* lfv_;
        adore::params::APTacticalPlanner* p_;
        adore::params::APLongitudinalPlanner* p_long_;
        double globalSpeedLimit_;
        double amin_;
        double t0_;
        double ds0_;
      public:
        LFVSpeedLimit(adore::view::ALane* lfv,adore::params::APTacticalPlanner* p):lfv_(lfv),p_(p)
        {
          globalSpeedLimit_ = 5.0;
          amin_ = -1.0;
          t0_ = 0.0;
          p_long_ = adore::params::ParamsFactoryInstance::get()->getLongitudinalPlanner();
        }

        virtual double getValue(double t,double s,double ds)const override
        {
          double v_initially_constrained = ds0_ + amin_*(t-t0_);
          double v_limit = std::min(lfv_->getSpeedLimit(s),globalSpeedLimit_);
          return std::max(v_initially_constrained,v_limit);
        }

        virtual void update(double t0,double s0,double ds0) override
        {
          globalSpeedLimit_ = p_->getGlobalSpeedLimit();
          amin_ = p_long_->getComfortAccLB();
          t0_ = t0;
          ds0_ = ds0;
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 1;
        }
    };


    /**
     * DistanceToLaneEndConstraint - use this as constraint to avoid moving too close to end of a lane
     */
    class DistanceToLaneEndConstraint:public ANominalConstraint
    {
      private:
        adore::view::ALane* lfv_;
        adore::params::APVehicle* pvehicle_;
        adore::params::APTrajectoryGeneration* pgen_;
        adore::params::APTacticalPlanner* ptac_;
        double stopDistance_;
        double distance_;
      public:
        DistanceToLaneEndConstraint(adore::view::ALane* lfv,adore::params::APVehicle* pvehicle,adore::params::APTrajectoryGeneration* pgen,
          adore::params::APTacticalPlanner* ptac):
          lfv_(lfv),pvehicle_(pvehicle),pgen_(pgen), ptac_(ptac) {
          }

        void update(double t0,double s0,double ds0) override 
        {
          stopDistance_ = ptac_->getHorizonStopReferenceDistance();
          distance_ = lfv_->getSMax()-(pvehicle_->get_a()+pvehicle_->get_b()+pvehicle_->get_c()-pgen_->get_rho()) - stopDistance_;
        }
        virtual double getValue(double t,double s,double ds)const override
        {
          return distance_;
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    };

    

    /**
     *  LateralAccelerationReference - use this reference for lateral planning in order to avoid road curvature
     */
    class LateralAccelerationReference:public ANominalReference
    {
      private:
        adore::view::ALane* lfv_;
      public:
        LateralAccelerationReference(adore::view::ALane* lfv):lfv_(lfv){}
      public:
        virtual bool getValueIfAvailable(double t, double s, double ds,double & ref)const override
        {
          ds = std::max(0.0,ds);
          double kappa = lfv_->getCurvature(s,0);
          ref = -kappa*ds*ds;            
          return true;
        }
        virtual void update(double t0,double s0,double ds0)override{}
        virtual int getDimension() override
        {
          return 1;
        }
        virtual int getDerivative() override
        {
          return 2;
        }
    };    

    /**
     *  LateralJerkReference - use this reference for lateral planning in order to avoid road curvature
     */
    class LateralJerkReference:public ANominalReference
    {
      private:
        adore::view::ALane* lfv_;
      public:
        LateralJerkReference(adore::view::ALane* lfv):lfv_(lfv){}
      public:
        virtual bool getValueIfAvailable(double t, double s, double ds,double & ref)const override
        {
          ds = std::max(0.0,ds);
          double dkappads = lfv_->getCurvature(s,1);
          ref = -dkappads*ds*ds*ds;
          return true;
        }
        virtual void update(double t0,double s0,double ds0)override{}
        virtual int getDimension() override
        {
          return 1;
        }
        virtual int getDerivative() override
        {
          return 3;
        }
    }; 

    /**
     *  NominalReferenceSpeed - use this reference for unconstrained nominal planning - maximize speed
     */
    class NominalReferenceSpeed:public ANominalReference
    {
      private:
        adore::view::ALane* lfv_;
        LFVSpeedLimit lFVSpeedLimit_;
        CurvatureSpeedLimit curvatureSpeedLimit_;
        adore::params::APLongitudinalPlanner* plon_;
        double constraintClearance_;
        double speed_scale_;/**< speed_scale_ allows to drive (cautiosly) at a certain percentage of the maximum speed*/
      public:
        NominalReferenceSpeed(adore::view::ALane* lfv,adore::params::APLongitudinalPlanner* plon,adore::params::APTacticalPlanner* ptac)
              :lfv_(lfv),lFVSpeedLimit_(lfv,ptac),curvatureSpeedLimit_(lfv,plon),plon_(plon)
              {
                speed_scale_ = 1.0;
              }
      public:
        virtual bool getValueIfAvailable(double t, double s, double ds,double & ref)const override
        {
          //set reference to upper bound - constraint clearance parameter
          ref = (std::max)(0.0,(std::min)(lFVSpeedLimit_.getValue(t,s,ds),
                                           curvatureSpeedLimit_.getValue(t,s,ds))
                                           -constraintClearance_) * speed_scale_;
          return true;
        }
        void setSpeedScale(double value)
        {
          speed_scale_ = value;
        }
        virtual void update(double t0,double s0,double ds0)override
        {
          constraintClearance_ = plon_->getConstraintClearanceVel();
          lFVSpeedLimit_.update(t0,s0,ds0);
          curvatureSpeedLimit_.update(t0,s0,ds0);
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 1;
        }
    }; 

    /**
     *  FollowCenterlineReference - use this reference to follow the coordinate system of the lane following view
     */
    class FollowCenterlineReference:public ANominalReference
    {
      private:
        adore::view::ALane* lfv_;
        adore::params::APLateralPlanner* p_lat_;
        adore::params::APVehicle* p_veh_;
        double d_;///lateral grid scale 
        double dmax_;///hard safety distance
        double w_;///width of vehicle
        double i_grid_;//grid step index
      public:
        FollowCenterlineReference(adore::view::ALane* lfv,double i_grid = 0.0):lfv_(lfv)
        {
          p_lat_ = adore::params::ParamsFactoryInstance::get()->getLateralPlanner();
          p_veh_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
          d_ = 0.2;
          dmax_ = 0;
          w_ = 1.8;
          i_grid_ = i_grid;//You don't know nothing, Jon Snow!
        }
        virtual bool getValueIfAvailable(double t, double s, double ds,double & ref)const override
        {
          const double dl = lfv_->getOffsetOfLeftBorder(s)-dmax_-w_*0.5;
          const double dr = lfv_->getOffsetOfRightBorder(s)+dmax_+w_*0.5;
          const double dc = (dl+dr)*0.5;
          ref = dc + i_grid_ * d_;
          if(dl<ref && ref<dr)ref = dc;//constraints invalid, use center
          else if(dl<ref)ref = dl;
          else if(ref<dr)ref = dr;
          return true;
        }
        virtual void update(double t0,double s0,double ds0)override
        {
          d_ = p_lat_->getLateralGridScale();
          dmax_ = p_lat_->getHardSafetyDistanceToLaneBoundary();
          w_ = p_veh_->get_bodyWidth();
        }
        virtual int getDimension() override
        {
          return 1;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    }; 
    
    /**
     *  LateralOffsetConstraintLF - provides the left=UB or right=LB bound for lateral planning wrt the lane following view.
     */
    class LateralOffsetConstraintLF:public ANominalConstraint
    {
      private:
        adore::view::ALane* lfv_;
        adore::params::APVehicle* pv_;
        adore::params::APLateralPlanner* plat_;
        double width_;
        double hard_safety_distance_;
        double soft_safety_distance_;
        double min_control_space_;
        double delay_s_;
        double delay_n_;
        double s0_;
        ConstraintDirection d_;
      public:
        LateralOffsetConstraintLF(adore::view::ALane* lfv,
                                  adore::params::APVehicle* pv,
                                  adore::params::APLateralPlanner* plat,
                                  ConstraintDirection d):lfv_(lfv),pv_(pv),plat_(plat),d_(d)
        {
        }


        double getValue(double t,double s,double ds)const override
        {
          double dl = lfv_->getOffsetOfLeftBorder(s);
          double dr = lfv_->getOffsetOfRightBorder(s);
          double dc = (dl+dr)*0.5;
          if(s-s0_<delay_s_)
          {
            dl += delay_n_;
            dr -= delay_n_;
          }
          double d = dl-dr;

          //TODO: next section was added for debugging
          {
            if(d_==LB)
            {
              return dr + hard_safety_distance_ + width_*0.5;
            }
            else
            {
              return dl - hard_safety_distance_ - width_*0.5;
            }
          }

          //not enough room to enforce hard safety distance->singular solution
          if(d < 2.0*hard_safety_distance_ + width_)
          {
            return dc+((d_==LB)?-0.01:+0.01);
          }
          //not enough room to enforce min_control_space and hard_safety_distance -> enforce hard_safety_distance
          else if(d < 2.0*soft_safety_distance_ + width_ + min_control_space_)
          {
            if(d_==LB)
            {
              return dr + hard_safety_distance_ + width_*0.5;
            }
            else
            {
              return dl - hard_safety_distance_ - width_*0.5;
            }
          }
          //enough room to enforce soft_safety_distance and min_control_space
          else
          {
            if(d_==LB)
            {
              return dr + soft_safety_distance_ + width_*0.5;
            }
            else
            {
              return dl - soft_safety_distance_ - width_*0.5;
            }            
          }
        }
        virtual void update(double t0,double s0,double ds0) override
        {
          width_ = pv_->get_bodyWidth();
          hard_safety_distance_ = plat_->getHardSafetyDistanceToLaneBoundary();
          soft_safety_distance_ = plat_->getSoftSafetyDistanceToLaneBoundary();
          min_control_space_ = plat_->getMinimumLateralControlSpace();
          delay_s_ = plat_->getMergeConstraintDelay() * (std::max)(ds0,3.0);
          delay_n_ = width_;
          s0_ = s0;
        }
        virtual ConstraintDirection getDirection() override
        {
          return d_;
        }
        virtual int getDimension() override
        {
          return 1;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    };

    /**
     * LongitudinalAccelerationConstraint - static bounds on longitudinal acceleration
     */
    class LongitudinalAccelerationConstraint:public ANominalConstraint
    {
      private:
        adore::params::APLongitudinalPlanner* plon_;
        double ax_ub_;
        double ax_lb_;
        double ax_ub_slow_;
        double v_ax_ub_slow_;
        double ds0_;
        double t0_;
        ANominalConstraint::ConstraintDirection d_;
      public:
        LongitudinalAccelerationConstraint( adore::params::APLongitudinalPlanner* plon,
                                            ANominalConstraint::ConstraintDirection d)
                            :plon_(plon),d_(d){}

        virtual double getValue(double t,double s,double ds)const override
        {
          if(d_==ANominalConstraint::LB)
          {
            return ax_lb_;
          }
          else
          {
            double daxdds = std::max(0.0,(ax_ub_slow_-ax_ub_)/std::max(0.1,v_ax_ub_slow_));
            return ax_ub_slow_ - std::max(0.0,std::min(v_ax_ub_slow_,ds)) * daxdds;
          }
        }
        virtual void update(double t0,double s0,double ds0) override
        {
          ax_ub_ = plon_->getAccUB();
          ax_lb_ = plon_->getAccLB();
          ax_ub_slow_ = plon_->getAccUBSlow();
          v_ax_ub_slow_ = plon_->getVAccUBSlow();
          t0_ = t0;
          ds0_ = ds0;
        }
        virtual ConstraintDirection getDirection() override
        {
          return d_;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 2;
        }
    };

    /**
     * A constraint, which upper bounds the position of the ego vehicle according to the nearest preceding vehicle
     */
    class FollowPrecedingVehicle:public ANominalConstraint
    {
      private:
        adore::view::ALane* lane_;
        const adore::params::APTacticalPlanner* ptac_;
        const adore::params::APVehicle* pveh_;
        const adore::params::APTrajectoryGeneration* pgen_;
        //@TODO: prediction strategy
        double s_front_t0_;/**< constraint position at t0*/
        double v_front_t0_;/**< constraint velocity at t0*/
        double t0_;/**< observation time*/
      public:
        FollowPrecedingVehicle(adore::view::ALane* lane,
                                const adore::params::APVehicle* pveh,
                                const adore::params::APTacticalPlanner* ptac,
                                const adore::params::APTrajectoryGeneration* pgen)
                      :lane_(lane),ptac_(ptac),pveh_(pveh),pgen_(pgen){}
        virtual double getValue(double t,double s,double ds)const override
        {
          return s_front_t0_ + v_front_t0_ * (t-t0_);
        }
        virtual void update(double t0,double s0,double ds0) override 
        {
          double to_front = pveh_->get_a() + pveh_->get_b() + pveh_->get_c() - pgen_->get_rho();
          const adore::view::TrafficQueue& q = lane_->getOnLaneTraffic();
          s_front_t0_ = 1.0e6;//initialize unbounded
          v_front_t0_ = 0.0;
          t0_ = t0;
          for(auto& object:q)
          {
            double object_v0 = object.getCurrentSpeed();
            double delay = t0_ - object.getObservationTime();
            double object_s0 = object.getCurrentProgress() - object.getLength()*0.5 - to_front + delay * object_v0;
            if(object_s0>s0)
            {
              double buffer = object_v0 * ptac_->getFrontTimeGap() + ptac_->getFrontSGap();
              s_front_t0_ = object_s0 - buffer ;
              v_front_t0_ = object_v0;
              break;
            }
          }
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    };

    /**
     * A constraint, which upper bounds the position of the ego vehicle according to the nearest preceding vehicle
     */
    class FollowPrecedingVehicle_BreakingCurve:public ANominalConstraint
    {
      private:
        adore::view::ALane* lane_;
        const adore::params::APTacticalPlanner* ptac_;
        const adore::params::APVehicle* pveh_;
        const adore::params::APTrajectoryGeneration* pgen_;
        //@TODO: prediction strategy
        double s_front_t0_;/**< constraint position at t0*/
        double v_front_t0_;/**< constraint velocity at t0*/
        double t0_;/**< observation time*/
        double amin_;/**< minimum assumed acceleration for preceding vehicle*/
      public:
        FollowPrecedingVehicle_BreakingCurve(adore::view::ALane* lane,
                                const adore::params::APVehicle* pveh,
                                const adore::params::APTacticalPlanner* ptac,
                                const adore::params::APTrajectoryGeneration* pgen)
                      :lane_(lane),ptac_(ptac),pveh_(pveh),pgen_(pgen),amin_(-1.0){}
        virtual double getValue(double t,double s,double ds)const override
        {
          double t_standstill = std::max(v_front_t0_,0.0) / -amin_ + t0_;
          t = std::min(t,t_standstill);//predict no further then until standstill
          return s_front_t0_ + v_front_t0_ * (t-t0_) + 0.5*amin_*(t-t0_)*(t-t0_);
        }
        virtual void update(double t0,double s0,double ds0) override 
        {
          amin_ = ptac_->getAssumedNominalAccelerationMinimum();
          double to_front = pveh_->get_a() + pveh_->get_b() + pveh_->get_c() - pgen_->get_rho();
          const adore::view::TrafficQueue& q = lane_->getOnLaneTraffic();
          s_front_t0_ = 1.0e6;//initialize unbounded
          v_front_t0_ = 0.0;
          t0_ = t0;
          for(auto& object:q)
          {
            double object_v0 = object.getCurrentSpeed();
            double delay = t0_ - object.getObservationTime();
            double object_s0 = object.getCurrentProgress() - object.getLength()*0.5 - to_front + delay * object_v0;
            if(object_s0>s0)
            {
              //in contrast to other constraint variant, the time gap is removed here, as the breaking curve is directly considered
              double buffer = ptac_->getFrontSGap();
              s_front_t0_ = object_s0 - buffer ;
              v_front_t0_ = object_v0;
              break;
            }
          }
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    };

    /**
     * A constraint, which upper bounds the position of the ego vehicle according to the nearest preceding vehicle
     * with a constant distance only. Braking curve and time gap are not considered.
     */
    class LowerBoundSGapToPrecedingVehicle:public ANominalConstraint
    {
      private:
        adore::view::ALane* lane_;
        const adore::params::APTacticalPlanner* ptac_;
        const adore::params::APVehicle* pveh_;
        const adore::params::APTrajectoryGeneration* pgen_;
        //@TODO: prediction strategy
        double s_front_t0_;/**< constraint position at t0*/
        double v_front_t0_;/**< constraint velocity at t0*/
        double t0_;/**< observation time*/
        double amin_;/**< minimum assumed acceleration for preceding vehicle*/
      public:
        LowerBoundSGapToPrecedingVehicle(adore::view::ALane* lane,
                                const adore::params::APVehicle* pveh,
                                const adore::params::APTacticalPlanner* ptac,
                                const adore::params::APTrajectoryGeneration* pgen)
                      :lane_(lane),ptac_(ptac),pveh_(pveh),pgen_(pgen),amin_(-1.0){}
        virtual double getValue(double t,double s,double ds)const override
        {
          return s_front_t0_ + v_front_t0_ * (t-t0_);
        }
        virtual void update(double t0,double s0,double ds0) override 
        {
          double to_front = pveh_->get_a() + pveh_->get_b() + pveh_->get_c() - pgen_->get_rho();
          const adore::view::TrafficQueue& q = lane_->getOnLaneTraffic();
          s_front_t0_ = 1.0e6;//initialize unbounded
          v_front_t0_ = 0.0;
          t0_ = t0;
          for(auto& object:q)
          {
            double object_v0 = object.getCurrentSpeed();
            double delay = t0_ - object.getObservationTime();
            double object_s0 = object.getCurrentProgress() - object.getLength()*0.5 - to_front + delay * object_v0;
            if(object_s0>s0)
            {
              //in contrast to other constraint variant, the time gap is removed here, as the breaking curve is directly considered
              double buffer = ptac_->getLowerBoundFrontSGapForLF();
              s_front_t0_ = object_s0 - buffer ;
              v_front_t0_ = object_v0;
              break;
            }
          }
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    };

    /**
     * A constraint, which upper bounds the position of the ego vehicle according to the next limit line
     */
    class AdhereToNextLimitLine:public ANominalConstraint
    {
      private:
         adore::view::ALimitLineEnRoute* nextLL_;
        const adore::params::APTacticalPlanner* ptac_;
        const adore::params::APVehicle* pveh_;
        const adore::params::APTrajectoryGeneration* pgen_;
        const adore::params::APLongitudinalPlanner* plon_;
        double s_max_;
        std::set<adore::view::LimitLine::EState> drivable_states_;
        std::set<adore::view::LimitLine::EState> clearance_states_;
        
      public:
        AdhereToNextLimitLine( adore::view::ALimitLineEnRoute* nextLL)
                      :nextLL_(nextLL)
        {
          ptac_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
          pveh_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
          pgen_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
          plon_ = adore::params::ParamsFactoryInstance::get()->getLongitudinalPlanner();
          drivable_states_.insert(adore::view::LimitLine::permissive_Movement_Allowed);
          drivable_states_.insert(adore::view::LimitLine::protected_Movement_Allowed);
          drivable_states_.insert(adore::view::LimitLine::caution_Conflicting_Traffic);
          drivable_states_.insert(adore::view::LimitLine::dark);//HeD20220407: Usage in DE-BS-Steinriedendamm/Forstrstr intersection: Train crossing pre-signal is "dark", when no trains approaching. AV has to cross pre-signal in that case.
          clearance_states_.insert(adore::view::LimitLine::permissive_clearance);
          clearance_states_.insert(adore::view::LimitLine::protected_clearance);
        }
        virtual double getValue(double t,double s,double ds)const override
        {
          return s_max_;
        }
        virtual void update(double t0,double s0,double ds0) override 
        {
          s_max_ = 1.0e6;//initialize unconstrained
          if(nextLL_!=nullptr && nextLL_->hasLimitLine(s0))
          {
            auto ll = nextLL_->getLimitLine(t0,s0);
            double front_length = pveh_->get_a() + pveh_->get_b() + pveh_->get_c() - pgen_->get_rho();
            double s_stop = ll.getProgress()-front_length;
            double distance = s_stop -s0;
            double v = ds0>1.0?ds0:0.0;
            bool braking_possible = true;
            if(plon_->stopAtRedLights_always_before())
            {
              //if red light is before rear-bumper: stop
              braking_possible = (ll.getProgress()>s0-pgen_->get_rho()-pveh_->get_d());
            }
            else
            {
              //prevent braking, if impossible acceleration required
              if(v>0.1)
              {
                double a_stop = -0.5 * v*v / std::max(distance + plon_->stopAtRedLights_max_connection_length(),0.0);
                braking_possible = a_stop>plon_->getAccLB();
              }
            }
            //if the current state is drivable
            if(drivable_states_.find(ll.getCurrentState())!=drivable_states_.end())
            {
              return;//leave smax at maximum value
            }
            //if the red-light is so far behind, that stopping is supressed, continue
            if(-distance>plon_->stopAtRedLights_max_connection_length())
            {
              return;//leave smax at maximum value
            }
            if(clearance_states_.find(ll.getCurrentState())!=clearance_states_.end()
               && !braking_possible)
            {
              return;//leave smax at maximum value
            }
            s_max_ = s_stop;
          }
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    };
    /**
     * A constraint, which upper bounds the position of the ego vehicle according to the next goalpoint
     */
    class StopAtNextGoalPoint:public ANominalConstraint
    {
      private:
        const adore::view::ANavigationGoalView* next_;
        const adore::params::APVehicle* pveh_;
        const adore::params::APTrajectoryGeneration* pgen_;
        double s_max_;
      public:
        StopAtNextGoalPoint(const adore::view::ANavigationGoalView* next)
                      :next_(next)
        {
          pveh_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
          pgen_ = adore::params::ParamsFactoryInstance::get()->getTrajectoryGeneration();
        }
        virtual double getValue(double t,double s,double ds)const override
        {
          return s_max_;
        }
        virtual void update(double t0,double s0,double ds0) override 
        {
          s_max_ = 1.0e6;//initialize unconstrained
          if(next_!=nullptr
          && next_->isNextGoalPointFinal()
          && next_->isNextGoalPointInView())
          {
            double front_length = pveh_->get_a() + pveh_->get_b() + pveh_->get_c() - pgen_->get_rho();
            s_max_ = next_->getProgress() - front_length;
          }
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    };

    /**
     * A constraint, which upper bounds the position of the ego vehicle according to the next conlictpoint
     */
    class StopAtNextConflictPoint:public ANominalConstraint
    {
      private:
        const adore::view::AConflictPointSet* conflicts_;
        double distance_to_point_;
        double smax_;
        adore::params::APVehicle* pvehicle_;
        adore::params::APLongitudinalPlanner* plon_;

      public:
        void setView(adore::view::AConflictPointSet* conflicts)
        {
          conflicts_ = conflicts;
        }
        StopAtNextConflictPoint(const adore::view::AConflictPointSet* conflicts)
                      :conflicts_(conflicts)
        {
          distance_to_point_ = 5.0;
          pvehicle_ =  adore::params::ParamsFactoryInstance::get()->getVehicle();
          plon_ = adore::params::ParamsFactoryInstance::get()->getLongitudinalPlanner();
        }
        virtual double getValue(double t,double s,double ds)const override
        {
          return smax_;
        }
        virtual void update(double t0,double s0,double ds0) override 
        {
          distance_to_point_ = plon_->getStopDistanceToConflictPoint();
          smax_ = 1.0e6;//initialize unconstrained
          if(conflicts_!=nullptr && conflicts_->size()>0)
          {
            auto p = conflicts_->getPoint(0);
            smax_ = p.s-distance_to_point_-pvehicle_->get_a()-pvehicle_->get_b()-pvehicle_->get_c();
          }
        }
        virtual ConstraintDirection getDirection() override
        {
          return UB;
        }
        virtual int getDimension() override
        {
          return 0;
        }
        virtual int getDerivative() override
        {
          return 0;
        }
    };

  }
}