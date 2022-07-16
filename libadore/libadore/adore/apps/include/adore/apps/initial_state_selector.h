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
 *   Daniel Heß - initial implementation
 ********************************************************************************/


#pragma once

#include <adore/fun/setpointrequest.h>
#include <adore/fun/vehiclemotionstate9d.h>
#include <adore/fun/vehicleextendedstate.h>
#include <adore/fun/afactory.h>

namespace adore
{
namespace fun
{
    /**
     * @brief Helps to select initial state for motion planning.
     * Considers previous plan, current vehicle state, vehicle extended state
     */
    class InitialStateSelector
    {
        private:
            SetPointRequest spr_;
            adore::fun::VehicleMotionState9d x_replan_;
            adore::fun::VehicleMotionState9d x_;
            adore::fun::VehicleExtendedState xx_;
            adore::mad::AReader<adore::fun::VehicleMotionState9d>* xreader_;
            adore::mad::AReader<adore::fun::VehicleExtendedState>* xxreader_;
            bool hasValidInitialState_;
            adore::params::APTacticalPlanner* pTacticalPlanner_;
        public:
            InitialStateSelector()
            {
                hasValidInitialState_ = false;
                xreader_ = FunFactoryInstance::get()->getVehicleMotionStateReader();
                xxreader_ = FunFactoryInstance::get()->getVehicleExtendedStateReader();
            }
            ~InitialStateSelector()
            {
                delete xreader_;
                delete xxreader_;
            }
            void setExecutedTrajectory(const SetPointRequest& spr_ex)
            {
                spr_.setPoints.clear();
                spr_ex.copyTo(spr_);
                hasValidInitialState_ = false;
            }
            void update()
            {
                if(xreader_->hasData() && xxreader_->hasData())
                {
                    xreader_->getData(x_);
                    xxreader_->getData(xx_);
                    x_replan_ = x_;
                    hasValidInitialState_ = true;
                    if( spr_.setPoints.size()>0 //maneuver exists 
                        && xx_.getAutomaticControlOn() //< Freigabe im Fahrzeuginterface für Längs und Quer erhalten
                        && xx_.getAutomaticControlAccelerationActive())// Bestätigung der Freigabe durch Benutzer/Gaspedal erfolgt
                    {
                        double t = x_.getTime();
                        if( spr_.isActive(t) )
                        {
                            auto x_ref = spr_.interpolateReference(t,pvehicle_);
                            double dx = x_.getX()-x_ref.getX();
                            double dy = x_.getY()-x_ref.getY();
                            double R = pTacticalPlanner_->getResetRadius();
                            if(dx*dx+dy*dy<R*R)
                            {
                            x_replan.setX(x_ref.getX());
                            x_replan.setY(x_ref.getY());
                            x_replan.setPSI(x_ref.getPSI());
                            x_replan.setvx(x_ref.getvx());
                            x_replan.setvy(x_ref.getvy());
                            x_replan.setOmega(x_ref.getOmega());
                            x_replan.setAx(x_ref.getAx());
                            x_replan.setDelta(x_ref.getDelta());
                            reset = false;
                            }
                            else
                            {
                            std::cout <<"trajectory cannot be resumed: reset readius exceeded"<<std::endl;
                            combined_maneuver_.setPoints.clear();
                            }
                        }
                        else
                        {
                            if(combined_maneuver_.setPoints.size()>0)
                            {
                            std::cout <<"trajectory cannot be resumed due to timeout. t= "<<t<<", spr: ["<<combined_maneuver_.setPoints.front().tStart<<";"<<combined_maneuver_.setPoints.back().tEnd<<"]"<<std::endl;
                            }
                            combined_maneuver_.setPoints.clear();
                        }
                    }

                }
            }

    };
}
}