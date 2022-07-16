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
#include <adore/fun/afactory.h>
#include <adore/params/afactory.h>
#include <adore/fun/vehiclemotionstatebuffer.h>
#include <adore/fun/activationstateobserver.h>

namespace adore
{
namespace fun
{
    /**
     * @brief class helps to dispatch SetPointRequest to controller
     * SetPointRequestDispatcher handles selection of initial state, coordinate conversion for odometry based tracking and dispatching of trajectories.
     */
    class SetPointRequestDispatcher
    {
        private:
        adore::params::APTacticalPlanner* ptac_;/**<tactical planner parameters*/
        adore::params::APVehicle* pveh_;/**<vehicle parameters*/
        AFactory::TMotionStateReader* localization_state_reader_;/**<get vehicle state from localization state estimator*/
        VehicleMotionStateBuffer odometry_buffer_;/**<get vehicle state from odometry state estimator*/
        VehicleMotionStateBuffer localization_buffer_;/**<get vehicle state from localization state estimator, use lowest time of both buffers for initial state*/
        AFactory::TSetPointRequestWriter* setpointrequest_localization_writer_;/**<write spr in localization coordinates*/
        AFactory::TSetPointRequestWriter* setpointrequest_odometry_writer_;/**<write spr in odometry coordinates*/
        AFactory::TSetPointRequestWriter* nominal_trajectory_writer_;/**<write nominal trajectory in localization coordinates*/
        SetPointRequest spr_odom_;/**<last spr written in odometry coordinates*/
        SetPointRequest spr_localization_;/**<last spr written in localization coordinates*/
        VehicleMotionState9d x_localization_;/**<state in localization coordinates*/
        VehicleMotionState9d x_odometry_;/**<state in odometry coordinates*/
        VehicleMotionState9d x0_localization_;/**<initial state in localization coordinates*/
        VehicleMotionState9d x0_odometry_;/**<initial state in odometry coordinates*/
        ActivationStateObserver activation_observer_;/**<is automation in control?*/
        bool reset_;/**<indicates whether start point is resetted*/
        bool has_vehicle_state_;
        std::string status_msg_;/**<explanation why resetting*/
        private:
        void computeResetCondition()
        {
            if(!activation_observer_.isAutomaticControlEnabled())
            {
                reset_ = true;//while in manual control mode, always reset initial state to current vehicle state
                status_msg_ = "reset: manual control";
                return;
            }

            if( spr_odom_.isActive(x_odometry_.getTime()) )
            {
                auto x_ref_ = spr_odom_.interpolateReference(x_odometry_.getTime(), pveh_);
                double dx = x_odometry_.getX() - x_ref_.getX();
                double dy = x_odometry_.getY() - x_ref_.getY();
                double R = ptac_->getResetRadius();
                if (dx * dx + dy * dy < R*R)
                {
                    reset_ = false;
                    status_msg_ = "";
                }
                else
                {
                   reset_ = true;
                   std::stringstream ss;
                   ss<<"reset: radius R="<<R<<" exceeded with dx="<<dx<<" and dy="<<dy;
                   status_msg_ = ss.str();
                }
            }
            else
            {
                reset_ = true;
                if(spr_localization_.setPoints.size()==0)
                {
                    status_msg_ = "reset: spr empty";
                }
                else
                {
                    std::stringstream ss;
                    ss<<"reset: spr["<< spr_localization_.setPoints.begin()->tStart
                      <<","<< spr_localization_.setPoints.rbegin()->tEnd
                      <<"]inactive at t="<<x_localization_.getTime();
                }
            }
        }

        void computePlanStartPoint()
        {        
            if(reset_)
            {
                x0_localization_ = x_localization_;
                x0_odometry_ = x_odometry_;
                if(x0_odometry_.getvx()<0.1)
                {
                    x0_odometry_.setAx(0.0);
                    x0_localization_.setAx(0.0);
                }
            }
            else
            {
                double t0 = x_localization_.getTime();
                auto xref_odom = spr_odom_.interpolateReference(t0, pveh_);
                x0_odometry_.setTime(t0);
                x0_odometry_.setX(xref_odom.getX());
                x0_odometry_.setY(xref_odom.getY());
                x0_odometry_.setPSI(xref_odom.getPSI());
                x0_odometry_.setvx(xref_odom.getvx());
                x0_odometry_.setvy(xref_odom.getvy());
                x0_odometry_.setOmega(xref_odom.getOmega());
                x0_odometry_.setAx(xref_odom.getAx());
                x0_odometry_.setDelta(xref_odom.getDelta());
                x0_localization_ = x0_odometry_;

                const double c0 = (std::cos)(x_odometry_.getPSI());
                const double s0 = (std::sin)(x_odometry_.getPSI());
                ///the position vector from state to trajectory in current odom frame, (et;en), awa heading vector in current odom frame, (epsit;epsin)
                const double et = c0 * (x0_odometry_.getX()-x_odometry_.getX()) + s0 * (x0_odometry_.getY()-x_odometry_.getY());
                const double en =-s0 * (x0_odometry_.getX()-x_odometry_.getX()) + c0 * (x0_odometry_.getY()-x_odometry_.getY());
                const double epsit = c0 * (std::cos)(x0_odometry_.getPSI()) + s0 * (std::sin)(x0_odometry_.getPSI());
                const double epsin =-s0 * (std::cos)(x0_odometry_.getPSI()) + c0 * (std::sin)(x0_odometry_.getPSI());
                const double c1 = (std::cos)(x_localization_.getPSI());
                const double s1 = (std::sin)(x_localization_.getPSI());
                x0_localization_.setX(x_localization_.getX()+c1*et-s1*en);
                x0_localization_.setY(x_localization_.getY()+s1*et+c1*en);
                x0_localization_.setPSI(std::atan2(s1*epsit+c1*epsin,c1*epsit-s1*epsin));
            }
        }
        /**
         *  update vehicle state variables
         */
        void update()
        {
            status_msg_ = "";
            has_vehicle_state_ = false;
            //the localization/odometry signal are interpolated to the earlier time of both
            localization_buffer_.update();
            if(localization_buffer_.size()==0)
            {
                status_msg_ = "no localization state";
                return;
            }
            odometry_buffer_.update();
            if(odometry_buffer_.size()==0)
            {
                status_msg_ = "no odometry state";
                return;
            }
            double t = (std::min)(localization_buffer_.getTmax(),odometry_buffer_.getTmax());
            has_vehicle_state_ =   odometry_buffer_.interpolate_or_latest(t,x_odometry_)
                                && localization_buffer_.interpolate_or_latest(t,x_localization_);
            if(!has_vehicle_state_)
            {
                status_msg_ = "can not interpolate vehicle states";
            }
        }
        public:
        SetPointRequestDispatcher()
        : odometry_buffer_(FunFactoryInstance::get()->getVehicleOdometryMotionStateFeed()),
          localization_buffer_(FunFactoryInstance::get()->getVehicleLocalizationMotionStateFeed())
        {
            localization_state_reader_ = FunFactoryInstance::get()->getVehicleMotionStateReader();
            setpointrequest_localization_writer_ = FunFactoryInstance::get()->getSetPointRequestWriter();
            setpointrequest_odometry_writer_ = FunFactoryInstance::get()->getOdomSetPointRequestWriter();
            nominal_trajectory_writer_ = FunFactoryInstance::get()->getNominalTrajectoryWriter();
            ptac_ = adore::params::ParamsFactoryInstance::get()->getTacticalPlanner();
            pveh_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
            has_vehicle_state_ = false;
        }
        ~SetPointRequestDispatcher()
        {
            delete localization_state_reader_;
            delete setpointrequest_localization_writer_;
            delete setpointrequest_odometry_writer_;
            delete nominal_trajectory_writer_;
            delete ptac_;
            delete pveh_;
        }
        /**
         * @brief compute and return initial state for next planning iteration
         * The according initial state in odometry coordinates is saved for the future dispatch of the planned trajectory.
         * @return VehicleMotionState9d in localization coordinates
         */
        bool getInitialState(VehicleMotionState9d& result)
        {
            update();
            if(!has_vehicle_state_)return false;
            computeResetCondition();   
            computePlanStartPoint();
            result = x0_localization_;
            return true;
        }

        /**
         * @brief dispatch SetPointRequests computed in localization coordinates
         * The combined trajectory will be converted to odometry coordinates and published both in odometry and localization coordinates
         * The nominal trajectory will be published in localization coordinates 
         * @param combined_trajectory combined trajectory in localization coordinates
         * @param nominal_trajectory nominal trajectory in localization coordinates
         */
        void dispatch(SetPointRequest& combined_trajectory, SetPointRequest& nominal_trajectory)
        {
            spr_localization_ = combined_trajectory;
            spr_odom_ = combined_trajectory;
            spr_odom_.relocateTo(x0_odometry_.getX(),x0_odometry_.getY(),x0_odometry_.getPSI());
            setpointrequest_localization_writer_->write(spr_localization_);
            setpointrequest_odometry_writer_->write(spr_odom_);
            nominal_trajectory_writer_->write(nominal_trajectory);
        }

        std::string getStatus()
        {
            return status_msg_;
        }
   };

}
}