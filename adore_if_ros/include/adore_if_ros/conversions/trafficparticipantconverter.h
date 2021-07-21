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
#include "quaternion.h"
#include <adore_if_ros_msg/TrafficParticipantSet.h>
#include <adore_if_ros_msg/TrafficParticipantSimulation.h>
#include <adore/env/traffic/participant.h>

namespace adore
{
    namespace if_ROS
    {
        /**
         * Convert between adore::env::traffic:TParticipantSet and adore_if_ros_msg::TrafficParticipantSet.
         */
        class TrafficParticipantConverter
        {
            public:
            /**
             * Convert a adore_if_ros_msg::TrafficParticipantDetection to a adore::env::traffic::Participant
             */
            void decodeTrafficParticipantDetection(const adore_if_ros_msg::TrafficParticipantDetection* msg,adore::env::traffic::Participant* object)
            {
                decodeTrafficParticipant(msg->data,object);
                object->detection_by_sensor_ = msg->detection_by_sensor;
                object->trackingID_ = msg->trackingID;
            }
            /**
             * Convert a adore::env::traffic::Participant to a adore_if_ros_msg::TrafficParticipantDetection
             */
            void encodeTrafficParticipantDetection(const adore::env::traffic::Participant& object,adore_if_ros_msg::TrafficParticipantDetection* msg)
            {
                encodeTrafficParticipant(object,&msg->data);
                msg->detection_by_sensor = object.getDetectionBySensor();
                msg->trackingID = object.getTrackingID();
            }
            /**
             * Convert a adore_if_ros_msg::TrafficParticipant to a adore::env::traffic::Participant
             */
            void decodeTrafficParticipant(const adore_if_ros_msg::TrafficParticipant& data,adore::env::traffic::Participant* object)
            {
                object->acceleration_x_ = 0.0;
                object->brakeLight_certainty_ = data.brakeLight_certainty;
                object->brakeLightsOn_ = data.brakeLightOn;
                object->center_(0) = data.motion_state.pose.pose.position.x;
                object->center_(1) = data.motion_state.pose.pose.position.y;
                object->center_(2) = data.motion_state.pose.pose.position.z;
                object->classification_ = static_cast<adore::env::traffic::Participant::EClassification>(data.classification.type_id);
                object->classification_certainty_ = data.classification_certainty;
                object->existance_certainty_ = data.existance_certainty;
                object->height_ = data.shape.dimensions[data.shape.BOX_Z];
                object->highBeamCertainty_ = data.highBeam_certainty;
                object->highBeamOn_ = data.highBeamOn;
                object->leftIndicator_certainty_ = data.leftIndicator_certainty;
                object->leftIndicatorOn_ = data.leftIndicatorOn;
                object->length_ = data.shape.dimensions[data.shape.BOX_X];
                object->lowBeamCertainty_ = data.lowBeam_certainty;
                object->lowBeamOn_ = data.lowBeamOn;
                object->observation_time_ = data.time;
                object->rightIndicator_certainty_ = data.rightIndicator_certainty;
                object->rightIndicatorOn_ = data.rightIndicatorOn;
                object->v2xStationID_ = data.v2xStationID;
                object->vx_ = data.motion_state.twist.twist.linear.x;
                object->vy_ = data.motion_state.twist.twist.linear.y;
                object->width_ = data.shape.dimensions[data.shape.BOX_Y];
                QuaternionConverter qc;
                object->yaw_ = qc.quaternionToHeading(data.motion_state.pose.pose);
                object->yawrate_ = data.motion_state.twist.twist.angular.z;
            }
            /**
             * Convert a adore::env::traffic::Participant to a adore_if_ros_msg::TrafficParticipant
             */
            void encodeTrafficParticipant(const adore::env::traffic::Participant& object,adore_if_ros_msg::TrafficParticipant* msg)
            {
                msg->time = object.observation_time_;
                msg->brakeLight_certainty = object.brakeLight_certainty_;
                msg->brakeLightOn = object.brakeLightsOn_;
                msg->motion_state.pose.pose.position.x = object.center_(0);
                msg->motion_state.pose.pose.position.y = object.center_(1);
                msg->motion_state.pose.pose.position.z = object.center_(2);
                msg->classification.type_id = (char) object.classification_;
                msg->classification_certainty = object.classification_certainty_;
                msg->existance_certainty = object.existance_certainty_;
                msg->shape.dimensions.push_back(object.length_);
                msg->shape.dimensions.push_back(object.width_);
                msg->shape.dimensions.push_back(object.height_);
                msg->highBeam_certainty = object.highBeamCertainty_;
                msg->highBeamOn = object.highBeamOn_;
                msg->leftIndicator_certainty = object.leftIndicator_certainty_;
                msg->leftIndicatorOn = object.leftIndicatorOn_;
                msg->lowBeam_certainty = object.lowBeamCertainty_;
                msg->lowBeamOn = object.lowBeamOn_;
                msg->motion_state.header.stamp.fromSec( object.observation_time_ );
                msg->rightIndicator_certainty = object.rightIndicator_certainty_;
                msg->rightIndicatorOn = object.rightIndicatorOn_;
                msg->v2xStationID = object.v2xStationID_;
                msg->motion_state.twist.twist.linear.x = object.vx_;
                msg->motion_state.twist.twist.linear.y = object.vy_;
                QuaternionConverter qc;
                qc.setHeading(object.yaw_,msg->motion_state.pose.pose);
                msg->motion_state.twist.twist.angular.z = object.yawrate_;
             }
        };

        class TPSetConverter:private TrafficParticipantConverter
        {
            public:
            /**
             * Convert a adore_if_ros_msg::TrafficParticipantSet to a adore::env::traffic::TParticipantSet.
             */
            void operator()(adore_if_ros_msg::TrafficParticipantSetConstPtr msg,adore::env::traffic::TParticipantSet* set)
            {
                set->clear();
                for(auto& entry:msg->data)
                {
                    adore::env::traffic::Participant object;
                    decodeTrafficParticipantDetection(&entry,&object);
                    set->push_back(object);
                }
            }
            adore_if_ros_msg::TrafficParticipantSet operator()(const adore::env::traffic::TParticipantSet& set)
            {
                adore_if_ros_msg::TrafficParticipantSet msg;
                for(auto& p:set)
                {
                    adore_if_ros_msg::TrafficParticipantDetection pmsg;
                    encodeTrafficParticipantDetection(p,&pmsg);
                    msg.data.push_back(pmsg);
                }
                return msg;
            }
        };

        class TPSimulationConverter:private TrafficParticipantConverter
        {
            public:
            /**
             * Convert a adore::env::traffic::Participant to a adore_if_ros_msg::TrafficParticipantSimulation
             * The trackingID is used as simulationID
             */
            adore_if_ros_msg::TrafficParticipantSimulation operator()(const adore::env::traffic::Participant& object)
            {
                adore_if_ros_msg::TrafficParticipantSimulation msg;
                encodeTrafficParticipant(object,&msg.data);
                msg.simulationID = object.getTrackingID();    
                return msg;
            }
            /**
             * Convert a adore_if_ros_msg::TrafficParticipantSimulation to  a adore::env::traffic::Participant 
             */
            void operator()(adore_if_ros_msg::TrafficParticipantSimulationConstPtr msg,adore::env::traffic::Participant& object)
            {
                decodeTrafficParticipant(msg->data,&object);
                object.trackingID_ = msg->simulationID;
            }
            /**
             * Convert a adore_if_ros_msg::TrafficParticipantSimulation to  a adore::env::traffic::Participant 
             */
            void operator()(const adore_if_ros_msg::TrafficParticipantSimulation& msg,adore::env::traffic::Participant& object)
            {
                decodeTrafficParticipant(msg.data,&object);
                object.trackingID_ = msg.simulationID;
            }

        };

        class TPDetectionConverter:private TrafficParticipantConverter
        {
            public:
            /**
             * Convert a adore_if_ros_msg::TrafficParticipantDetection to a adore::env::traffic::Participant
             */
            void operator()(adore_if_ros_msg::TrafficParticipantDetectionConstPtr msg,adore::env::traffic::Participant* object)
            {
                decodeTrafficParticipantDetection(msg.get(),object);
            }
            /**
             * Convert a  adore::env::traffic::Participant to a adore_if_ros_msg::TrafficParticipantDetection 
             */
            adore_if_ros_msg::TrafficParticipantDetection operator()(const adore::env::traffic::Participant& object)
            {
                adore_if_ros_msg::TrafficParticipantDetection msg;
                encodeTrafficParticipantDetection(object,&msg);
                return msg;
            }
        };

        class TPMessageConverter:private TrafficParticipantConverter
        {
            public:
            /**
             * Convert a adore::env::traffic::Participant to a adore_if_ros_msg::TrafficParticipant
             */
            adore_if_ros_msg::TrafficParticipant operator()(const adore::env::traffic::Participant& object)
            {
                adore_if_ros_msg::TrafficParticipant msg;
                encodeTrafficParticipant(object,&msg);
                return msg;
            }
        };
    }
}