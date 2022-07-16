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
#include <vector>

namespace adore
{
    namespace view
    {
        /**
         * The mapping of a traffic participant to ALane.
         */
        struct TrafficObject
        {
        public:
            typedef int TTrackingID;
            typedef long long int TV2XStationID;
        private:
                TTrackingID trackingID_;        /**< An id assigned by sensor data fusion, with a high likelihood of stable association of object over time*/
                TV2XStationID v2xStationID_;    /**< etsi vehicle to x id */
                double observationTime_;        /**< Point of time at which observation was made and to which position information in this representation corresponds*/
                double currentProgress_;        /**< Progress of TrafficObject along ALane or ConflictZone*/
                double currentSpeed_;           /**< Current speed of object*/
                double currentAcceleration_;    /**< Current acceleration of object*/
                double entranceProgress_;       /**< Estimate of progress at which TrafficObject will enter ALane or ConflictZone*/
                double entranceSpeed_;          /**< Estimate of speed at which TrafficObject will enter ALane or ConflictZone*/
                double entranceTime_;           /**< Estimate of time at which TrafficObject will enter ALane or ConflictZone*/
                double exitTime_;               /**< Estimate of time at which TrafficObject will exit ALane or ConflictZone*/
                double exitProgress_;           /**< Estimate of progress at which TrafficObject will exit ALane or ConflictZone*/
                double length_;                 /**< Length of the TrafficObject*/
        public:
            /**
             *  getTrackingID - returns an id for the traffic object. The consistency of the id over time should be maintained by sensor data fusion.
             */
            TTrackingID getTrackingID()const 
            {
                return trackingID_;
            }
            /**
             *  getV2XStationID - returns the etsi vehicle to x station id
             */
            TV2XStationID getV2XStationID()const 
            {
                return v2xStationID_;
            }
            /**
             * getCurrentProgress - returns current progress of a traffic object along a lane or towards a conflict zone
             */
            double getCurrentProgress()const 
            {
                return currentProgress_;
            }
            /**
             * getCurrentSpeed - returns current speed of a traffic object
             */
            double getCurrentSpeed()const 
            {
                return currentSpeed_;
            }
            /**
             * getCurrentAcceleration - returns current acceleration of a traffic object
             */
            double getCurrentAcceleration() const
            {
                return currentAcceleration_;                
            }
            /**
             * getEntranceProgress - returns progress at which object's entrance to lane or to conflict zone will occur/is estimated to occur
             */
            double getEntranceProgress()const 
            {
                return entranceProgress_;
            }
            /**
             * getEntranceSpeed - returns speed at which object is estimated to enter lane or conflict zone
             */
            double getEntranceSpeed()const 
            {
                return entranceSpeed_;
            }
            /**
             * getEntranceSpeed - returns time at which object is estimated to enter lane or conflict zone
             */
            double getEntranceTime()const 
            {
                return entranceTime_;
            }
            /**
             * getEntranceSpeed - returns time at which object is estimated to exit lane or conflict zone
             */
            double getExitTime()const 
            {
                return exitTime_;
            }
            /**
             * getExitProgress - returns progress at which object's will exit / is estimated to exit lane or conflict zone
             */
            double getExitProgress()const 
            {
                return exitProgress_;
            }
            /**
             * getLength - length of the traffic object
             */
            double getLength()const
            {
                return length_;
            }
            /**
             * get point of time at which observation was made and to which position information in this representation corresponds
             */
            double getObservationTime()const
            {
                return observationTime_;
            }
        public:
            void setTrackingID(TTrackingID value) 
            {
                trackingID_ = value;
            }
            void setV2XStationID(TV2XStationID value) 
            {
                v2xStationID_ = value;
            }
            void setCurrentProgress(double value) 
            {
                currentProgress_ = value;
            }
            void setCurrentSpeed(double value) 
            {
                currentSpeed_ = value;
            }
            void setCurrentAcceleration(double value) 
            {
                currentAcceleration_ = value;                
            }
            void setEntranceProgress(double value) 
            {
                entranceProgress_ = value;
            }
            void setEntranceSpeed(double value) 
            {
                entranceSpeed_ = value;
            }
            void setEntranceTime(double value) 
            {
                entranceTime_ = value;
            }
            void setExitTime(double value) 
            {
                exitTime_ = value;
            }
            void setExitProgress(double value) 
            {
                exitProgress_ = value;
            }
            void setLength(double value)
            {
                length_ = value;
            }
            void setObservationTime(double value)
            {
                observationTime_ = value;
            }
        };

        /**
         *  A queue of traffic objects. On lane, ordered by current progress. Approaching conflict zone, ordered by EntranceTime. 
         */
        typedef std::vector<TrafficObject> TrafficQueue;

    }
}