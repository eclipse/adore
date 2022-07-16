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
    namespace view
    {
        /**
         * LimitLine contains state information about regulation of a conflict zone.
         */
        struct LimitLine
        {
            public:
            /**
             * Defines access to critical area guarded by a LimitLine.
             */
            enum EState
            {
                // UNKNOWN = 0,            /**< default: state is unknown -> wait and/or transition of control*/
                // WAIT = 1,               /**< traffic light is red or train crossing is closed, car has to wait*/
                // STOP = 2,               /**< stop first, then yield, e.g. stop sign*/
                // CONFLICT = 3,           /**< yield and respect higher priority traffic, traffic light is green or flashing yellow or non existant*/
                // GO = 4 ,                /**< safe to drive, no higher priority traffic*/
                // CLEAR = 5               /**< allowed to clear junction, but do not enter*/
                /**
                 * values according to itsg5::dsrc_v2::MovementPhaseState
                 */
                unavailable                 = 0,/**< default: state is unknown -> wait and/or transition of control*/
                dark                        = 1,/**< stop, transition of control, proceed manually*/
                stop_Then_Proceed           = 2,/**< stop first, then yield, e.g. stop sign*/
                stop_And_Remain             = 3,/**< stop and wait */
                pre_Movement                = 4,/**< red/yellow*/
                permissive_Movement_Allowed = 5,/**< green, go and yield */
                protected_Movement_Allowed  = 6,/**< green, go, no conflict*/
                permissive_clearance        = 7,/**< yellow, clear connection and yield*/
                protected_clearance         = 8,/**< yellow, clear connection no conflict*/
                caution_Conflicting_Traffic = 9 /**< green, conflicting traffic*/
            };
            private:
            EState current_state_;      /**< the current accessibility of the critical area*/
            EState next_state_;         /**< the next accessibility of the critical area, with state change occuring at t_next_*/
            double t_next_;             /**< switching time when current_state_ becomes next_state_. set to big number if unknown*/
            double s_;                  /**< progress of LimitLine along the containing lane*/

            public:
            LimitLine():current_state_(unavailable),next_state_(unavailable),t_next_(0.0),s_(0.0){}
            void setCurrentState(EState value)
            {
                current_state_ = value;
            }
            void setNextState(EState value)
            {
                next_state_ = value;
            }
            void setTimeToNextState(double value)
            {
                t_next_ = value;
            }
            void setProgress(double value)
            {
                s_ = value;
            }

            public:
            EState getCurrentState()const 
            {
                return current_state_;
            }
            EState getNextState()const
            {
                return next_state_;
            }
            double getTimeToNextState()const
            {
                return t_next_;
            }
            double getProgress()const
            {
                return s_;
            }

        };
    }
}