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
 *    Daniel Heß - initial implementation and API
 ********************************************************************************/

#pragma once

#include <adore/mad/adoremath.h>

namespace adore
{
    namespace env
    {
        namespace traffic
        {
            struct MovingBoxWithCovariance
            {
            public:
                adoreMatrix<double,3> center_;
                float yaw_; /**< orientation of the box: straight east is 0, straight north is pi/2 */
                float length_; /**< extend of box projected to yaw direction vector */
                float width_; /**< extend of box perpendicular to yaw and up */
                float height_; /**< extend of box projected on up vector */
                float velocity_x_; /**< speed in yaw direction */
                float velocity_y_; /**< speed perpendicular to yaw direction */
                float yawrate_; /**< d/dt yaw */
                float acceleration_x_; /**< acceleration in yaw direction */


                bool covariance_center_;   /**<  If true, elements of the covariance matrix which
                                            refer to the center_x, center_y are filled with
                                            values. They are set to NaN otherwise. */
                bool covariance_angle_;    /**<  If true, the block of the covariance matrix
                                            which refer to the angle are filled with values.
                                            They are set to NaN otherwise. */
                bool covariance_length_width_; /**< If true, the block of the covariance matrix
                                            which refer to the length and width are filled
                                            with values. They are set to NaN otherwise. */
                bool covariance_velocity;  /**< If true, the block of the covariance matrix
                                            which refer to the velocity_x and velocity_y are
                                            filled with values. They are set to NaN
                                            otherwise. */
                adoreMatrix<double,7,7> covariance_;

            };
        }
    }
}