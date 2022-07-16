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
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/


#pragma once
#include <adore/view/alanechangeview.h>
#include <adore/fun/tac/anominalplanner.h>
#include <adore/params/ap_longitudinal_planner.h>
#include <adore/params/ap_lateral_planner.h>
#include <adore/mad/adoremath.h>
#include <adore/view/agap.h>

namespace adore
{
namespace fun
{

    enum LaneChangePhase
    {
      preTransition,
      transition,
      postTransition,
      undefined
    };
    /**
     *  LateralOffsetConstraintLC - provides the left=UB or right=LB bound for lateral planning wrt the lane change view.
     */
    class LateralOffsetConstraintLC:public ANominalConstraint
    {
      private:
        adore::view::ALaneChangeView *lcv_;
        adore::params::APVehicle* pv_;
        adore::params::APLateralPlanner* plat_;
        double width_;
        double hard_safety_distance_;
        double soft_safety_distance_;
        double min_control_space_;
        ConstraintDirection direction_;
        adore::view::AGap* gap_;
      public:
        LateralOffsetConstraintLC(adore::view::ALaneChangeView* lcv,
                                  adore::params::APVehicle* pv,
                                  adore::params::APLateralPlanner* plat,
                                  ConstraintDirection direction)
          :lcv_(lcv),pv_(pv),plat_(plat),direction_(direction),gap_(nullptr){}

        void setGap(adore::view::AGap* gap){gap_=gap;}

        virtual double getValue(double t,double s,double ds)const override
        {
          double dl,dr,dc,d;
          if(!lcv_->getSourceLane()->isValid())return 0;
          if(gap_==nullptr ||!lcv_->getTargetLane()->isValid())
          {
            dl = lcv_->getSourceLane()->getOffsetOfLeftBorder(s);
            dr = lcv_->getSourceLane()->getOffsetOfRightBorder(s);
          }
          else
          {
            auto state = gap_->getState(s,t);
            state = adore::view::AGap::OPEN;
            if( s < lcv_->getProgressOfGateOpen()) state = adore::view::AGap::OPENING;
            if( s > lcv_->getProgressOfGateClosed()) state = adore::view::AGap::CLOSED;
            switch( state )
            {
              case adore::view::AGap::OPENING:
              {
                if(lcv_->getLCDirection()==adore::view::ALaneChangeView::LEFT)
                {
                  dl = lcv_->getOffsetOfDestinationOuterBorder(s);
                  dr = lcv_->getOffsetOfStartOuterBorder(s);
                }
                else
                {
                  dl = lcv_->getOffsetOfStartOuterBorder(s);                  
                  dr = lcv_->getOffsetOfDestinationOuterBorder(s);
                }
              }
              break;
              case adore::view::AGap::OPEN:
              {
                if(lcv_->getLCDirection()==adore::view::ALaneChangeView::LEFT)
                {
                  dl = lcv_->getOffsetOfDestinationOuterBorder(s);
                  dr = lcv_->getOffsetOfStartOuterBorder(s);
                }
                else
                {
                  dl = lcv_->getOffsetOfStartOuterBorder(s);
                  dr = lcv_->getOffsetOfDestinationOuterBorder(s);                  
                }
              }
              break;
              case adore::view::AGap::CLOSED:
              {
                dl = lcv_->getTargetLane()->getOffsetOfLeftBorder(s);
                dr = lcv_->getTargetLane()->getOffsetOfRightBorder(s);
              }
              break;
            }
          }
          dc = (dl+dr)*0.5;//center
          d = dl-dr;
          //TODO: this part has been inserted for debugging
          {
            if(direction_==LB)
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
            return dc;
          }
          //not enough room to enforce min_control_space and hard_safety_distance -> enforce hard_safety_distance
          else if(d < 2.0*soft_safety_distance_ + width_ + min_control_space_)
          {
            if(direction_==LB)
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
            if(direction_==LB)
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

          //for lane change: operate only with hard safety distance
          // soft_safety_distance_ = hard_safety_distance_;
        }
        virtual ConstraintDirection getDirection() override
        {
          return direction_;
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
     *  LaneChangePhaseEstimator - provides estimation for current LaneChangePhase
     * to do: re-arrange code structure. this is no ANominalReference
     *        -> shift this functionality to the Gap itself?
     */
    class LaneChangePhaseEstimator:public ANominalReference
    {
      private:
        adore::view::ALaneChangeView* lcv_;
        adore::params::APVehicle* pveh_;
        adore::params::APLateralPlanner* plat_;
        adore::params::APTrajectoryGeneration* pgen_;
        adore::params::APTacticalPlanner* ptac_;
        adore::view::AGap* gap_;
        double width_;
        double s_front_t0_;
        double v_front_t0_;
        double sgap_;
        double t0_;
      public:
        LaneChangePhaseEstimator(adore::view::ALaneChangeView* lcv,adore::params::APVehicle* pv,adore::params::APLateralPlanner* plat,adore::params::APTrajectoryGeneration* pgen, adore::params::APTacticalPlanner* ptac)
          :lcv_(lcv),pveh_(pv),plat_(plat),pgen_(pgen),ptac_(ptac),gap_(nullptr),sgap_(0.0){}
        void setGap(adore::view::AGap* gap){gap_=gap;}

        LaneChangePhase getLaneChangePhase(double t, double s)
        {
          if (gap_!=nullptr && lcv_->getSourceLane()->isValid() && lcv_->getTargetLane()->isValid())
          {
            switch (gap_->getState(s,t)) 
            {
            case adore::view::AGap::OPENING:
                return LaneChangePhase::preTransition;
                break;
            case adore::view::AGap::CLOSED:
                return LaneChangePhase::postTransition;
                break;
            case adore::view::AGap::OPEN:
                if (s> s_front_t0_ + (t-t0_) * v_front_t0_)
                  return LaneChangePhase::postTransition;
                return LaneChangePhase::transition;
                break;
            default:
                throw std::runtime_error("unknown gap state in LaneChangePhaseExtimator");
                break;
            }
          }
          return LaneChangePhase::undefined;
        }

        virtual bool getValueIfAvailable(double t, double s, double ds,double & ref)const override
        {
          return false;
        }
        virtual void update(double t0,double s0,double ds0)override
        {
          sgap_ =ptac_->getFrontSGap();
          width_ = pveh_->get_bodyWidth();
         


          //amin_ = ptac_->getAssumedNominalAccelerationMinimum();
          double to_front = pveh_->get_a() + pveh_->get_b() + pveh_->get_c() - pgen_->get_rho();
          const adore::view::TrafficQueue& q = lcv_->getSourceLane()->getOnLaneTraffic();
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
              double buffer = sgap_;
              s_front_t0_ = object_s0 - buffer ;
              v_front_t0_ = object_v0;
              break;
            }
          }
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
     *  AdvancedLateralOffsetConstraintLC - provides the left=UB or right=LB bound for lateral planning wrt the lane change view, depending on lane change phase.
     */
    class AdvancedLateralOffsetConstraintLC:public ANominalConstraint
    {
      private:
        adore::view::ALaneChangeView *lcv_;
        LaneChangePhaseEstimator* phaseEstimator_;
        adore::params::APVehicle* pv_;
        adore::params::APLateralPlanner* plat_;
        double width_;
        double hard_safety_distance_;
        double soft_safety_distance_;
        double min_control_space_;
        ConstraintDirection direction_;
        adore::view::AGap* gap_;
      public:
        AdvancedLateralOffsetConstraintLC(adore::view::ALaneChangeView* lcv,
                                  LaneChangePhaseEstimator* phaseEstimator,
                                  adore::params::APVehicle* pv,
                                  adore::params::APLateralPlanner* plat,
                                  ConstraintDirection direction)
          :lcv_(lcv),phaseEstimator_(phaseEstimator),pv_(pv),plat_(plat),direction_(direction),gap_(nullptr){}

        void setGap(adore::view::AGap* gap){gap_=gap;}

        virtual double getValue(double t,double s,double ds)const override
        {
          double dl,dr,dc,d;
          if(!lcv_->getSourceLane()->isValid())return 0;
          if(gap_==nullptr ||!lcv_->getTargetLane()->isValid() || phaseEstimator_ == nullptr)
          {
            dl = lcv_->getSourceLane()->getOffsetOfLeftBorder(s);
            dr = lcv_->getSourceLane()->getOffsetOfRightBorder(s);
          }
          else
          {
            auto phase = phaseEstimator_->getLaneChangePhase(t,s);
            switch( phase )
            {
              case LaneChangePhase::preTransition:
              {
                dl = lcv_->getSourceLane()->getOffsetOfLeftBorder(s);
                dr = lcv_->getSourceLane()->getOffsetOfRightBorder(s);
              }
              break;
              case LaneChangePhase::transition:
              {
                if(lcv_->getLCDirection()==adore::view::ALaneChangeView::LEFT)
                {
                  dl = lcv_->getOffsetOfDestinationOuterBorder(s);
                  dr = lcv_->getOffsetOfStartOuterBorder(s);
                }
                else
                {
                  dl = lcv_->getOffsetOfStartOuterBorder(s);
                  dr = lcv_->getOffsetOfDestinationOuterBorder(s);                  
                }
              }
              break;
              case LaneChangePhase::postTransition:
              {
                dl = lcv_->getTargetLane()->getOffsetOfLeftBorder(s);
                dr = lcv_->getTargetLane()->getOffsetOfRightBorder(s);
              }
              break;
            }
          }
          dc = (dl+dr)*0.5;//center
          d = dl-dr;
          //TODO: this part has been inserted for debugging
          {
            if(direction_==LB)
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
            return dc;
          }
          //not enough room to enforce min_control_space and hard_safety_distance -> enforce hard_safety_distance
          else if(d < 2.0*soft_safety_distance_ + width_ + min_control_space_)
          {
            if(direction_==LB)
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
            if(direction_==LB)
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

          //for lane change: operate only with hard safety distance
          // soft_safety_distance_ = hard_safety_distance_;
        }
        virtual ConstraintDirection getDirection() override
        {
          return direction_;
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
     * A constraint, which upper bounds the position of the ego vehicle according to the nearest preceding
     * on the lane to be left during a lane change maneuver
     */
    class FollowPrecedingVehicleOnLaneToBeLeft_BrakingCurve:public ANominalConstraint
    {
      private:
        fun::FollowPrecedingVehicle_BreakingCurve* fpv;
        LaneChangePhaseEstimator* phaseEstimator_;
      public:
        FollowPrecedingVehicleOnLaneToBeLeft_BrakingCurve(adore::view::ALane* lane,
                                LaneChangePhaseEstimator* phaseEstimator,
                                const adore::params::APVehicle* pveh,
                                const adore::params::APTacticalPlanner* ptac,
                                const adore::params::APTrajectoryGeneration* pgen)
                      :phaseEstimator_(phaseEstimator)
        {
          fpv = new FollowPrecedingVehicle_BreakingCurve(lane,pveh,ptac,pgen);
        }
        ~FollowPrecedingVehicleOnLaneToBeLeft_BrakingCurve()
        {
          delete fpv;
        }
        virtual double getValue(double t,double s,double ds)const override
        {
          if (phaseEstimator_->getLaneChangePhase(t,s) == LaneChangePhase::postTransition)
          {
            return 1.0e6;
          }
          return fpv->getValue(t,s,ds);
        }
        virtual void update(double t0,double s0,double ds0) override 
        {
          fpv->update(t0,s0,ds0);
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
     *  LateralOffsetConstraintLM - provides the left=UB or right=LB bound for lateral planning wrt the lane change view for merges.
     */
    class LateralOffsetConstraintLM:public ANominalConstraint
    {
      private:
        adore::view::ALaneChangeView *lcv_;
        adore::params::APVehicle* pv_;
        adore::params::APLateralPlanner* plat_;
        double width_;
        double hard_safety_distance_;
        double soft_safety_distance_;
        double min_control_space_;
        ConstraintDirection direction_;
      public:
        LateralOffsetConstraintLM(adore::view::ALaneChangeView* lcv,
                                  adore::params::APVehicle* pv,
                                  adore::params::APLateralPlanner* plat,
                                  ConstraintDirection direction)
          :lcv_(lcv),pv_(pv),plat_(plat),direction_(direction){}


        virtual double getValue(double t,double s,double ds)const override
        {
          double dl,dr,dc,d;
          if(!lcv_->getSourceLane()->isValid()||!lcv_->getTargetLane()->isValid())return 0;
          bool in_gate = lcv_->getProgressOfGateOpen()<s && s<lcv_->getProgressOfGateClosed();
          if(lcv_->getLCDirection()==adore::view::ALaneChangeView::LEFT)
          {
            if(in_gate)
            {
              dl = lcv_->getOffsetOfDestinationOuterBorder(s)
                 + lcv_->getSourceLane()->getOffsetOfLeftBorder(s);
            }
            else
            {
              dl = lcv_->getSourceLane()->getOffsetOfLeftBorder(s);
            }
            dr = lcv_->getSourceLane()->getOffsetOfRightBorder(s);
          }
          else
          {
            dl = lcv_->getSourceLane()->getOffsetOfLeftBorder(s);
            if(in_gate)
            {
              dr = lcv_->getOffsetOfDestinationOuterBorder(s)
                 + lcv_->getSourceLane()->getOffsetOfRightBorder(s);
            }
            else
            {
              dr = lcv_->getSourceLane()->getOffsetOfRightBorder(s);
            }
          }
          dc = (dl+dr)*0.5;//center
          d = dl-dr;
          

          //not enough room to enforce hard safety distance->singular solution
          if(d < 2.0*hard_safety_distance_ + width_)
          {
            return dc;
          }
          //not enough room to enforce min_control_space and hard_safety_distance -> enforce hard_safety_distance
          else if(d < 2.0*soft_safety_distance_ + width_ + min_control_space_)
          {
            if(direction_==LB)
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
            if(direction_==LB)
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
          //for lane merge: operate only with hard safety distance
          soft_safety_distance_ = hard_safety_distance_;
        }
        virtual ConstraintDirection getDirection() override
        {
          return direction_;
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
     *  LaneChangeIntoGapReference - use this reference for lateral profile of lane changes
     */
    class LaneChangeIntoGapReference:public ANominalReference
    {
      private:
        adore::view::ALaneChangeView* lcv_;
        adore::params::APVehicle* pv_;
        adore::params::APLateralPlanner* plat_;
        adore::view::AGap* gap_;
        double width_;
        double hard_safety_distance_;
        double soft_safety_distance_;
        double min_control_space_;
        double d_;///lateral grid scale 
        double i_grid_;//grid step index
      public:
        LaneChangeIntoGapReference(adore::view::ALaneChangeView* lcv,adore::params::APVehicle* pv,adore::params::APLateralPlanner* plat,double i_grid = 0.0)
          :lcv_(lcv),pv_(pv),plat_(plat),i_grid_(i_grid),gap_(nullptr){d_ = 0.2;}
        void setGap(adore::view::AGap* gap){gap_=gap;}

        virtual bool getValueIfAvailable(double t, double s, double ds,double & ref)const override
        {
          ref = 0.0;
          if(gap_!=nullptr && lcv_->getTargetLane()->isValid())
          {
            auto state = gap_->getState(s,t);
            state = adore::view::AGap::OPEN;
            if( s < lcv_->getProgressOfGateOpen()) state = adore::view::AGap::OPENING;
            if( s > lcv_->getProgressOfGateClosed()) state = adore::view::AGap::CLOSED;
            double dl,dr,dlc,drc;
            switch( state )
            {
              case adore::view::AGap::OPENING:
              {
                if(lcv_->getLCDirection()==adore::view::ALaneChangeView::LEFT)
                {
                  dl = lcv_->getOffsetOfSeparatingBorder(s);
                  dr = lcv_->getOffsetOfStartOuterBorder(s); 
                  dlc = lcv_->getOffsetOfSeparatingBorder(s)-hard_safety_distance_-width_*0.5;
                  drc = lcv_->getOffsetOfStartOuterBorder(s)+hard_safety_distance_+width_*0.5;
                } 
                else
                {
                  dr = lcv_->getOffsetOfSeparatingBorder(s);
                  dl = lcv_->getOffsetOfStartOuterBorder(s); 
                  drc = lcv_->getOffsetOfSeparatingBorder(s)+hard_safety_distance_+width_*0.5;
                  dlc = lcv_->getOffsetOfStartOuterBorder(s)-hard_safety_distance_-width_*0.5;
                }
              }
              break;
              case adore::view::AGap::OPEN:
              {
                if(lcv_->getLCDirection()==adore::view::ALaneChangeView::LEFT)
                {
                  dl = lcv_->getOffsetOfDestinationOuterBorder(s);
                  dr = lcv_->getOffsetOfSeparatingBorder(s); 
                  dlc = lcv_->getOffsetOfDestinationOuterBorder(s)-hard_safety_distance_-width_*0.5;
                  drc = lcv_->getOffsetOfStartOuterBorder(s)+hard_safety_distance_+width_*0.5;
                } 
                else
                {
                  dr = lcv_->getOffsetOfDestinationOuterBorder(s);
                  dl = lcv_->getOffsetOfSeparatingBorder(s); 
                  drc = lcv_->getOffsetOfDestinationOuterBorder(s)+hard_safety_distance_+width_*0.5;
                  dlc = lcv_->getOffsetOfStartOuterBorder(s)-hard_safety_distance_-width_*0.5;
                }
              }
              break;
              case adore::view::AGap::CLOSED:
              {
                if(lcv_->getLCDirection()==adore::view::ALaneChangeView::LEFT)
                {
                  dl = lcv_->getOffsetOfDestinationOuterBorder(s);
                  dr = lcv_->getOffsetOfSeparatingBorder(s); 
                  dlc = lcv_->getOffsetOfDestinationOuterBorder(s)-hard_safety_distance_-width_*0.5;
                  drc = lcv_->getOffsetOfSeparatingBorder(s)+hard_safety_distance_+width_*0.5;
                } 
                else
                {
                  dr = lcv_->getOffsetOfDestinationOuterBorder(s);
                  dl = lcv_->getOffsetOfSeparatingBorder(s); 
                  drc = lcv_->getOffsetOfDestinationOuterBorder(s)+hard_safety_distance_+width_*0.5;
                  dlc = lcv_->getOffsetOfSeparatingBorder(s)-hard_safety_distance_-width_*0.5;
                }
              }
              break;
            }
            const double dc = (dl+dr)*0.5;
            ref = dc + i_grid_ * d_;
            if(dlc<ref && ref<drc)ref = dc;//constraints invalid, use center
            else if(dlc<ref)ref = dlc;
            else if(ref<drc)ref = drc;
          }
          return true;
        }
        virtual void update(double t0,double s0,double ds0)override
        {
          width_ = pv_->get_bodyWidth();
          hard_safety_distance_ = plat_->getHardSafetyDistanceToLaneBoundary();
          soft_safety_distance_ = plat_->getSoftSafetyDistanceToLaneBoundary();
          min_control_space_ = plat_->getMinimumLateralControlSpace(); 
          d_ = plat_->getLateralGridScale();
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
     *  CancelMergeReference - use this reference for lateral profile of MRM, to cancel a merge with MRM
     */
    class CancelMergeReference:public ANominalReference
    {
      private:
        adore::view::ALaneChangeView* lcv_;
      public:
        CancelMergeReference(adore::view::ALaneChangeView* lcv):lcv_(lcv){}

        virtual bool getValueIfAvailable(double t, double s, double ds,double & ref)const override
        {
          if(lcv_->getTargetLane()->isValid())
          {
            ref = 0.5 * lcv_->getOffsetOfSeparatingBorder(s)
                + 0.5 * lcv_->getOffsetOfStartOuterBorder(s); 
            return true;
          }
          return false;
        }
        virtual void update(double t0,double s0,double ds0)override
        {
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
     *  ContinueLCReference - use this reference for lateral profile of MRM, to continue LC with MRM
     */
    class ContinueLCReference:public ANominalReference
    {
      private:
        adore::view::ALaneChangeView* lcv_;
      public:
        ContinueLCReference(adore::view::ALaneChangeView* lcv):lcv_(lcv){}

        virtual bool getValueIfAvailable(double t, double s, double ds,double & ref)const override
        {
          if(lcv_->getTargetLane()->isValid())
          {
            ref = 0.5 * lcv_->getOffsetOfSeparatingBorder(s)
                + 0.5 * lcv_->getOffsetOfDestinationOuterBorder(s); 
            return true;
          }
          return false;
        }
        virtual void update(double t0,double s0,double ds0)override
        {
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
* Constraints and references to change lanes into a gap defined by s-coordinate
*/
class StdGapConstraintAndReference : public ANominalPlannerInformation
{
private:
  adore::view::ALane *lfv_;
  adore::view::ALaneChangeView *lcv_;
  const adore::params::APTacticalPlanner *ptac_;
  const adore::params::APVehicle *pveh_;
  const adore::params::APTrajectoryGeneration *pgen_;
  double s_front_t0_; /**< front constraint position at t0*/
  double v_front_t0_; /**< front constraint velocity at t0*/
  double s_lead_t0_;  /**< lead constraint position at t0*/
  double v_lead_t0_;  /**< lead constraint velocity at t0*/
  double s_chase_t0_; /**< chase constraint position at t0*/
  double v_chase_t0_; /**< chase constraint velocity at t0*/
  double t0_;         /**< last update time*/
  double s_;          /**< s-coordinate to specify the gap*/
  enum gapState
  {
    gapOpen,
    gapOpening,
    gapClosed
  };

public:
  StdGapConstraintAndReference(adore::view::ALane *lfv,
                               adore::view::ALaneChangeView *lcv,
                               const adore::params::APVehicle *pveh,
                               const adore::params::APTacticalPlanner *ptac,
                               const adore::params::APTrajectoryGeneration *pgen)
      : lfv_(lfv), lcv_(lcv), ptac_(ptac), pveh_(pveh), pgen_(pgen) { s_ = 0.0; }
  gapState getGapState(double t) const
  {
    if (s_front_t0_ > 1.0e5 || s_chase_t0_ < -1.0e5 || getFrontBound(t) >= getChaseBound(t))
    {
      return gapOpen;
    }
    else if (getFrontBound(t + 10.0) >= getChaseBound(t + 10.0))
    {
      return gapOpening;
    }
    return gapClosed;
  }
  virtual double getUB(int dim, int der, double t, double s, double ds) const override
  {
    if (dim == 0 && der == 0)
    {
      switch (getGapState(t))
      {
      case gapClosed:
        return getLeadBound(t);
      case gapOpen:
        return std::min(getFrontBound(t), getLeadBound(t));
      case gapOpening:
        return getFrontBound(t);
      }
    }
    else if (dim == 1 && der == 0)
    {
      double half_width = 0.5 * pveh_->get_bodyWidth();
      switch (getGapState(t))
      {
      case gapClosed:
      {
        if (lcv_->getLCDirection() == adore::view::ALaneChangeView::LEFT)
          return -half_width + lcv_->getOffsetOfDestinationOuterBorder(s);
        return -half_width + lcv_->getOffsetOfSeparatingBorder(s);
      }
      case gapOpen:
      {
        if (lcv_->getLCDirection() == adore::view::ALaneChangeView::LEFT)
        {
          if (s > getChaseBound(t))
            return -half_width + lcv_->getOffsetOfDestinationOuterBorder(s);
          return -half_width + lcv_->getOffsetOfSeparatingBorder(s);
        }
        return -half_width + lcv_->getOffsetOfStartOuterBorder(s);
      }
      case gapOpening:
      {
        if (lcv_->getLCDirection() == adore::view::ALaneChangeView::LEFT)
          return -half_width + lcv_->getOffsetOfSeparatingBorder(s);
        return -half_width + lcv_->getOffsetOfStartOuterBorder(s);
      }
      }
    }
    return 1.0e6;
  }
  virtual double getLB(int dim, int der, double t, double s, double ds) const override
  {
    if (dim == 0 && der == 0)
    {
      switch (getGapState(t))
      {
      case gapOpen:
        return -1.0e6;
      case gapClosed:
        return getChaseBound(t);
      case gapOpening:
        return -1.0e6;
      }
    }
    else if (dim == 1 && der == 0)
    {
      double half_width = 0.5 * pveh_->get_bodyWidth();
      switch (getGapState(t))
      {
      case gapClosed:
      {
        if (lcv_->getLCDirection() == adore::view::ALaneChangeView::LEFT)
          return half_width + lcv_->getOffsetOfSeparatingBorder(s);
        return half_width + lcv_->getOffsetOfDestinationOuterBorder(s);
      }
      case gapOpen:
      {
        if (lcv_->getLCDirection() == adore::view::ALaneChangeView::LEFT)
          return half_width + lcv_->getOffsetOfStartOuterBorder(s);
        if (s > getChaseBound(t))
          return half_width + lcv_->getOffsetOfDestinationOuterBorder(s);
        return half_width + lcv_->getOffsetOfSeparatingBorder(s);
      }
      case gapOpening:
      {
        if (lcv_->getLCDirection() == adore::view::ALaneChangeView::LEFT)
          return half_width + lcv_->getOffsetOfStartOuterBorder(s);
        return half_width + lcv_->getOffsetOfSeparatingBorder(s);
      }
      }
    }
    return -1.0e6;
  }
  virtual bool getReferenceIfAvailable(int dim, int der, double t, double s, double ds, double &ref) const override
  {
    if (dim == 0 && der == 0)
    {
      switch (getGapState(t))
      {
      case gapOpen:
        ref = getFrontBound(t);
        return true;
      case gapOpening: // fix -Wswitch
        // TODO investigate if not setting ref is actually a bug
        return false;
      case gapClosed:
        ref = getLeadBound(t);
        return true;
      }
      return false;
    }
    if (dim == 1 && der == 0)
    {
      switch (getGapState(t))
      {
      case gapOpen:
      {
        if (s > getChaseBound(t))
        {
          ref = lcv_->getOffsetOfSeparatingBorder(s) + 0.5 * (lcv_->getOffsetOfDestinationOuterBorder(s) - lcv_->getOffsetOfSeparatingBorder(s));
        }
        else
        {
          double d = lcv_->getOffsetOfSeparatingBorder(s);
          d = d > 0 ? d - 0.5 * pveh_->get_bodyWidth() : d + 0.5 * pveh_->get_bodyWidth();
          ref = 0.5 * d;
        }
        return true;
      }
      case gapOpening:
      {
        double d = lcv_->getOffsetOfSeparatingBorder(s);
        d = d > 0 ? d - 0.5 * pveh_->get_bodyWidth() : d + 0.5 * pveh_->get_bodyWidth();
        ref = 0.5 * d;
        return true;
      }
      case gapClosed:
      {
        ref = lcv_->getOffsetOfSeparatingBorder(s) + 0.5 * (lcv_->getOffsetOfDestinationOuterBorder(s) - lcv_->getOffsetOfSeparatingBorder(s));
        return true;
      }
      }
      return false;
    }
    return false;
  }
  virtual void update(double t0, double s0, double ds0) override
  {
    t0_ = t0;
    double to_rear = pveh_->get_d() + pgen_->get_rho();
    double to_front = pveh_->get_a() + pveh_->get_b() + pveh_->get_c() - pgen_->get_rho();
    const adore::view::TrafficQueue &traffic_original_lane = lfv_->getOnLaneTraffic();
    const adore::view::TrafficQueue &traffic_target_lane = lcv_->getTargetLane()->getOnLaneTraffic();
    // front constraint
    s_front_t0_ = 1.0e6; //initialize unbounded
    v_front_t0_ = 0.0;
    for (auto &object : traffic_original_lane)
    {
      double object_v0 = object.getCurrentSpeed();
      double delay = t0_ - object.getObservationTime();
      double object_s0 = object.getCurrentProgress() - object.getLength() * 0.5 - to_front + delay * object_v0;
      if (object_s0 > s0)
      {
        double buffer = object_v0 * ptac_->getFrontTimeGap() + ptac_->getFrontSGap();
        s_front_t0_ = object_s0 - buffer;
        v_front_t0_ = object_v0;
        break;
      }
    }
    // lead constraint
    s_lead_t0_ = 1.0e6; // initialize unbounded
    v_lead_t0_ = 0.0;
    double s_lead_coord = 1.0e6; // var for calculating new s_ coordinate
    auto object_it = traffic_target_lane.begin();
    for (; object_it != traffic_target_lane.end(); ++object_it)
    {
      double object_v0 = object_it->getCurrentSpeed();
      double delay = t0_ - object_it->getObservationTime();
      double object_s0 = object_it->getCurrentProgress() - object_it->getLength() * 0.5 - to_front + delay * object_v0;
      if (object_s0 > s_)
      {
        double buffer = object_v0 * ptac_->getFrontTimeGap() + ptac_->getFrontSGap();
        s_lead_t0_ = object_s0 - buffer;
        v_lead_t0_ = object_v0;
        s_lead_coord = object_s0;
        break;
      }
    }
    // chase constraint
    s_chase_t0_ = -1.0e6; //initialize unbounded
    v_chase_t0_ = 0.0;
    double s_chase_coord = -1.0e6; // var for calculating new s_ coordinate
    if (object_it != traffic_target_lane.begin())
    {
      std::advance(object_it, -1);
      double object_v0 = object_it->getCurrentSpeed();
      double delay = t0_ - object_it->getObservationTime();
      double object_s0 = object_it->getCurrentProgress() + object_it->getLength() * 0.5 + to_rear + delay * object_v0;
      double buffer = object_v0 * ptac_->getRearTimeGap() + ptac_->getRearSGap();
      s_chase_t0_ = object_s0 + buffer;
      v_chase_t0_ = object_v0;
      s_chase_coord = object_s0;
    }
    // update the s coordinate so that it will be approximately in de middle of the gap in t + 0.1
    if (s_chase_t0_ > -1.0e5 && s_lead_t0_ < 1.0e5) // chase and lead exist
    {
      s_ = 0.5 * (s_chase_coord + 0.1 * v_chase_t0_ + s_lead_coord + 0.1 * v_lead_t0_);
    }
    else if (s_chase_t0_ > -1.0e5) // chase exists, lead not
    {
      s_ = 0.5 * (s_chase_coord + 0.1 * v_chase_t0_ + s0 + 0.1 * ds0);
    }
    else if (s_lead_t0_ < 1.0e5) // lead exists, chase not
    {
      s_ = 0.5 * (s_lead_t0_ + 0.1 * v_lead_t0_ + s0 + 0.1 * ds0);
    }
    else // neither lead nor chase exists
    {
      s_ = s0 + 0.1 * ds0;
    }
  }
  double getFrontBound(double t) const
  {
    return s_front_t0_ + v_front_t0_ * (t - t0_);
  }
  double getLeadBound(double t) const
  {
    return s_lead_t0_ + v_lead_t0_ * (t - t0_);
  }
  double getChaseBound(double t) const
  {
    return s_chase_t0_ + v_chase_t0_ * (t - t0_);
  }
  double getGapCoordinate() const
  {
    return s_;
  }
  void setGapCoordinate(double s)
  {
    s_ = s;
  }
};

} // namespace fun
} // namespace adore
