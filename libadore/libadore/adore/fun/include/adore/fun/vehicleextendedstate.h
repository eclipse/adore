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

namespace adore
{
    namespace fun
    {
        /**
         * Extended state information for vehicle.
         * Lists states, which are not included in the primary motion state of the vehicle.
         */
        class VehicleExtendedState
        {
            public:
            enum GearState
            {
                Park,Drive,Reverse,Neutral
            };
            private:
                GearState gearState_;
                bool indicatorLeftOn_;
                bool indicatorRightOn_;
                bool automaticControlAccelerationOn_;
                bool automaticControlAccelerationActive_;
                bool automaticControlSteeringOn_;
                bool checkpointClearance_; 
            public:
                VehicleExtendedState()
                {
                    gearState_ = Drive;
                    indicatorLeftOn_=false;
                    indicatorRightOn_=false;
                    automaticControlAccelerationOn_ = false;
                    automaticControlSteeringOn_= false ;
                    automaticControlAccelerationActive_ = false;
                    checkpointClearance_ = false;
                }
                GearState getGearState() const {
                	return this->gearState_;
                }
                void setGearState(GearState gearState) {
                	this->gearState_ = gearState;
                }


                bool getIndicatorLeftOn()  const{
                	return this->indicatorLeftOn_;
                }
                void setIndicatorLeftOn(bool indicatorLeftOn) {
                	this->indicatorLeftOn_ = indicatorLeftOn;
                }


                bool getIndicatorRightOn()  const{
                	return this->indicatorRightOn_;
                }
                void setIndicatorRightOn(bool indicatorRightOn) {
                	this->indicatorRightOn_ = indicatorRightOn;
                }


                bool getAutomaticControlAccelerationOn() const {
                	return this->automaticControlAccelerationOn_;
                }
                void setAutomaticControlAccelerationOn(bool automaticControlAccelerationOn) {
                	this->automaticControlAccelerationOn_ = automaticControlAccelerationOn;
                }


                bool getAutomaticControlAccelerationActive() const {
                	return this->automaticControlAccelerationActive_;
                }
                void setAutomaticControlAccelerationActive(bool automaticControlAccelerationActive) {
                	this->automaticControlAccelerationActive_ = automaticControlAccelerationActive;
                }


                bool getAutomaticControlSteeringOn()  const{
                	return this->automaticControlSteeringOn_;
                }
                void setAutomaticControlSteeringOn(bool automaticControlSteeringOn) {
                	this->automaticControlSteeringOn_ = automaticControlSteeringOn;
                }

                bool getAutomaticControlOn() const
                {
                    return getAutomaticControlSteeringOn() && getAutomaticControlAccelerationOn() && getAutomaticControlAccelerationActive();
                }

                bool getCheckpointClearance()  const{
                	return this->checkpointClearance_;
                }
                void setCheckpointClearance(bool checkpointClearance) {
                	this->checkpointClearance_ = checkpointClearance;
                }

        };
    }
}