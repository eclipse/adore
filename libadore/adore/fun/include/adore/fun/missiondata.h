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
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/

#pragma once

namespace adore
{
    namespace fun
    {
        /**
         * Mission data
         */
        class MissionData
        {
            public:
            enum MissionState
            {
                AUTOMATION_NOT_ACTIVE,
                DRIVING_TO_GOAL,
                GOAL_REACHED
            };

            private:
                MissionState missionState_;
                
            public:
                MissionData()
                {
                    missionState_ = AUTOMATION_NOT_ACTIVE;
                }
                MissionState getMissionState() const {
                	return this->missionState_;
                }
                void setMissionState(MissionState missionState) {
                	this->missionState_ = missionState;
                }

        };
    }
}