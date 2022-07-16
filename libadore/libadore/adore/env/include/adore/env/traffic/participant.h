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
#include <vector>
#include "movingbox.h"

namespace adore
{
    namespace env
    {
        namespace traffic
        {
            /**
             * @brief Struct for representing a participant in traffic
             * 
             */
            struct Participant
            {
            public:
                typedef int TTrackingID;
                typedef long long int TV2XStationID;
                typedef adoreMatrix<double,7,7> TCovariance;
                enum EClassification: int
                {
                    UNCLASSIFIED=0,
                    UNKNOWN_SMALL=1,
                    UNKNOWN_BIG=2,
                    UNDER_DRIVABLE=3  ,
                    OVER_DRIVABLE=4  , 
                    PEDESTRIAN=5,
                    BIKE=6,
                    BICYCLE=7,
                    MOPED=8,
                    MOTORCYCLE=9,
                    CAR=10,
                    VAN=11,
                    BUS=12,
                    TRUCK=13,
                    LIGHT_TRUCK=14,
                    HEAVY_TRUCK=15,
                    TRAM=16,
                    TRAILER=17,
                    SPECIAL_VEHICLE=18,
                    PEDESTRIAN_GROUP=19,
                    TRAIN=20,
                    HORSE_RIDER=21,
                    ANIMAL_SMALL=22,
                    ANIMAL_BIG=23
                };
                static const unsigned int UNDEFINED = 1;
                static const unsigned int RADAR = 2;
                static const unsigned int LIDAR = 4;
                static const unsigned int MONO_VIDEO = 8;
                static const unsigned int STEREO_VISION = 16;
                static const unsigned int SPERICAL_CAMERA = 32;
                static const unsigned int NIGHT_VISION = 64;
                static const unsigned int ULTRASONIC = 128;
                static const unsigned int INDUCTIONLOOP = 256;
                static const unsigned int PMD = 512;
                static const unsigned int V2X_CAM = 1024;
                static const unsigned int V2X_CPM = 2048;
                static const unsigned int V2X_MCM = 4096;
                static const unsigned int SPECULATIVE = 2147483648; /**< 2^31, not observed by any sensor but rather speculated to exist in occlusion area */

            public:
                TTrackingID trackingID_; /**< internal tracking id, providing highest possible continuity */
                TV2XStationID v2xStationID_; /**< etsi vehicle to x id */
                double existance_certainty_; /**<  confidence that traffic participant exists 0..100 */
                double observation_time_; /**< time corresponding to state */

                bool leftIndicatorOn_; /**< indicator in yaw direction on the left side */
                double leftIndicator_certainty_; /**< confidence of detection of left indicator state 0..100 */
                bool rightIndicatorOn_; /**< indicator in yaw direction on the right side */
                double rightIndicator_certainty_; // confidence of detection of right indicator state 0..100 */
                bool brakeLightsOn_; /**< state of the brake lights */
                double brakeLight_certainty_; /**< confidence about brake light state 0..100 */
                bool lowBeamOn_; /**< state of the head lights */
                double lowBeamCertainty_; /**< confidence about head light state 0..100 */
                bool highBeamOn_; /**< state of the head lights */
                double highBeamCertainty_; /**< confidence about head light state 0..100 */

                unsigned int detection_by_sensor_; /**< bit array -> sensor type values */
                EClassification classification_; /**< type of traffic participant -> classification values */
                double classification_certainty_; /**< confidence of classification 0..100 */

                adoreMatrix<double,3,1> center_;
                double yaw_; /**< orientation of the box: straight east is 0, straight north is pi/2 */
                double length_; /**< extend of box projected to yaw direction vector */
                double width_; /**< xtend of box perpendicular to yaw and up */
                double height_; /**< extend of box projected on up vector */
                double vx_; /**< speed in yaw direction */
                double vy_; /**< speed perpendicular to yaw direction */
                double yawrate_; /**< d/dt yaw */
                double acceleration_x_; /**< acceleration in yaw direction */
                TCovariance covariance_; /**< [X,Y,Z,yaw,vx,vy,yawrate]^2 */

                TTrackingID getTrackingID()const {return trackingID_;}
                TV2XStationID getStationID()const{return v2xStationID_;}
                double getExistanceCertainty()const{return existance_certainty_;}
                double getObservationTime()const{return observation_time_;}
                bool getLeftIndicatorOn()const{return leftIndicatorOn_;}
                double getLeftIndicatorCertainty()const{return leftIndicator_certainty_;}
                bool getRightIndicatorOn()const{return rightIndicatorOn_;}
                double getRightIndicatorCertainty()const{return rightIndicator_certainty_;}
                bool getLowBeamOn()const{return lowBeamOn_;}
                double getLowBeamCertainty()const{return lowBeamCertainty_;}
                bool getHighBeamOn()const{return highBeamOn_;}
                double getHighBeamCertainty()const{return highBeamCertainty_;}
                unsigned int getDetectionBySensor()const{return detection_by_sensor_;}
                EClassification getClassification()const{return classification_;}
                double getClassificationCertainty()const{return classification_certainty_;}
                const adoreMatrix<double,3,1>& getCenter()const{return center_;}
                adoreMatrix<double,3,1> getFrontLeftCorner()const
                {
                    adoreMatrix<double,3,1> vector = center_;
                    vector(0) = vector(0) + std::cos(yaw_) * length_*0.5 - std::sin(yaw_) * width_*0.5;
                    vector(1) = vector(1) + std::sin(yaw_) * length_*0.5 + std::cos(yaw_) * width_*0.5;
                    return vector;
                }
                adoreMatrix<double,3,1> getFrontRightCorner()const
                {
                    adoreMatrix<double,3,1> vector = center_;
                    vector(0) = vector(0) + std::cos(yaw_) * length_*0.5 + std::sin(yaw_) * width_*0.5;
                    vector(1) = vector(1) + std::sin(yaw_) * length_*0.5 - std::cos(yaw_) * width_*0.5;
                    return vector;
                }
                adoreMatrix<double,3,1> getRearLeftCorner()const
                {
                    adoreMatrix<double,3,1> vector = center_;
                    vector(0) = vector(0) - std::cos(yaw_) * length_*0.5 - std::sin(yaw_) * width_*0.5;
                    vector(1) = vector(1) - std::sin(yaw_) * length_*0.5 + std::cos(yaw_) * width_*0.5;
                    return vector;
                }
                adoreMatrix<double,3,1> getRearRightCorner()const
                {
                    adoreMatrix<double,3,1> vector = center_;
                    vector(0) = vector(0) - std::cos(yaw_) * length_*0.5 + std::sin(yaw_) * width_*0.5;
                    vector(1) = vector(1) - std::sin(yaw_) * length_*0.5 - std::cos(yaw_) * width_*0.5;
                    return vector;
                }
                
                double getYaw()const {return yaw_;}
                double getLength()const{return length_;}
                double getWidth()const{return width_;}
                double getVx()const{return vx_;}
                double getVy()const{return vy_;}
                double getYawRate()const{return yawrate_;}
                double getAx()const{return acceleration_x_;}
                const TCovariance& getCovariance()const{return covariance_;}
            };
            
            typedef std::vector<Participant> TParticipantSet;
            
                        
        }
    }
}