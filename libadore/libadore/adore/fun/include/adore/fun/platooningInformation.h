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
 *   Reza Dariani - initial API and implementation
 ********************************************************************************/

#pragma once
#include <vector>
#include <adore_if_ros_msg/CooperativePlanning.h>
//#include <adore/fun/logic/platoonLogic.h>

namespace adore
{
	namespace fun
	{
		/**
		 * 	TBDone
		 */
		class PlatooningInformation
		{
		private:
			double ToleratedDistanceAhead;
			double ToleratedDistanceBehind;
			unsigned int    TargetAutomationLevel;
			int    LanePosition;
			int    id;

		public:
			void set(adore_if_ros_msg::CooperativePlanningConstPtr msg)
			{
				this->ToleratedDistanceAhead=msg->tolerated_distance_ahead;
				this->ToleratedDistanceBehind=msg->tolerated_distance_behind;
				this->TargetAutomationLevel=msg->target_automation_level;
				this->LanePosition = msg->lane_position;
				this->id = msg->id;
				//std::cout<<"\n"<<this->LanePosition <<"\t"<<this->TargetAutomationLevel;
			}
			void setToleratedDistanceAhead(double tda) {ToleratedDistanceAhead = tda;}
			void setToleratedDistanceBehind(double tdb) {ToleratedDistanceBehind = tdb;}
			void setTargetAutomationLevel(unsigned int tal) {TargetAutomationLevel = tal;}
			void setLanePosition(int lp) {LanePosition = lp;}
			void setId(int id) {this->id = id;}
			double getToleratedDistanceAhead()const{return ToleratedDistanceAhead;}
			double getToleratedDistanceBehind()const{return ToleratedDistanceBehind;}
			unsigned int getTargetAutomationLevel()const{return TargetAutomationLevel;}
			int getLanePosition()const{return LanePosition;}
			int getId()const{return id;}

			// maximum urban velocity (13.6) * (3) second time headway = 40.8
			//platoonLogic::SAE_LEVEL2 --> 2
			//platoonLogic::outermostDrivingLane --> 
			PlatooningInformation():ToleratedDistanceAhead(40.8),ToleratedDistanceBehind(40.8),TargetAutomationLevel(2),LanePosition(1), id(-1) {}    
			PlatooningInformation(double ToleratedDistanceAhead,double ToleratedDistanceBehind,unsigned int TargetAutomationLevel,int LanePosition, int id):ToleratedDistanceAhead(ToleratedDistanceAhead),ToleratedDistanceBehind(ToleratedDistanceBehind),TargetAutomationLevel(TargetAutomationLevel),LanePosition(LanePosition), id(id){}
		};
	}
}