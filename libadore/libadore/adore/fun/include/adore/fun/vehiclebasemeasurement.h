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
         * A set of measurement values obtained from vehicle base system
         */
        class VehicleBaseMeasurement
        {
            private:
                float steering_angle_;
                std::vector<float> wheel_speeds_;
                float yaw_rate_;
                float esp_ax_;
                float esp_ay_;                //@TODO: continue
            public:
            //VehicleBaseMeasurement():wheel_speeds_(4,0){}
                void setSteeringAngle(float value){steering_angle_=value;}
                float getSteeringAngle()const{return steering_angle_;}

                void setWheelSpeeds(const std::vector<float> value){wheel_speeds_=value;}
                const std::vector<float>& getWheelSpeeds()const{return wheel_speeds_;}

                void setYawRate(float value){yaw_rate_=value;}
                float getYawRate()const{return yaw_rate_;}

                void setEspAx(float value){esp_ax_=value;}
                float getEspAx()const{return esp_ax_;}

                void setEspAy(float value){esp_ay_=value;}
                float getEspAy()const{return esp_ay_;}

        };
    }
}