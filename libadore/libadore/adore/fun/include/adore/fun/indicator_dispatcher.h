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
#include <adore/fun/activationstateobserver.h>
namespace adore
{
namespace fun
{
    /**
     * @brief Dispatches indicator command for maneuver, which is currently executed.
     */ 
    class IndicatorDispatcher
    {
        private:
        AFactory::TIndicatorCommandWriter* icw_; /**< sends actual commands*/
        ActivationStateObserver activation_observer_;
        AFactory::TMotionStateReader* xreader_;
        public:
        IndicatorDispatcher()
        {
            icw_ = FunFactoryInstance::get()->getIndicatorCommandWriter();
            xreader_ = FunFactoryInstance::get()->getVehicleMotionStateReader();
        }
        ~IndicatorDispatcher()
        {
            delete icw_;
        }
        void setIndicators(const PlanningResult& selected_maneuver)
        {
            if(!xreader_->hasData())return;
            VehicleMotionState9d x;
            xreader_->getData(x);

            adore::fun::IndicatorCommand ic;
            //set indicators only if automatic control enabled
            if( activation_observer_.isAutomaticControlEnabled() )
            {
                if( selected_maneuver.combined_maneuver_valid 
                &&  selected_maneuver.combined_maneuver.isActive(x.getTime()) )
                {
                    ic.setIndicatorLeftOn(selected_maneuver.indicator_left);
                    ic.setIndicatorRightOn(selected_maneuver.indicator_right);
                }
                else
                {
                    if(x.getvx()>2.0)
                    {
                        //if no maneuver is active, set emergency indicators
                        ic.setIndicatorLeftOn(true);
                        ic.setIndicatorRightOn(true);                    
                    }
                    else
                    {
                        ic.setIndicatorLeftOn(false);
                        ic.setIndicatorRightOn(false);
                    }
                }
            }
            else
            {
                //if in manual mode, always set automatic control of indicators to false
                ic.setIndicatorLeftOn(false);
                ic.setIndicatorRightOn(false);
            }            
            icw_->write(ic);
        }

    };
}
}