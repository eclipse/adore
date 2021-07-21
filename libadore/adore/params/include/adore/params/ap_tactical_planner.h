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
	namespace params
	{
		/**
		 * @brief abstract classs containing parameters to configure aspects and constraints of the tactical planner
		 * 
		 */
		class APTacticalPlanner
		{
		public:
			virtual double getGlobalSpeedLimit()const =0;
			virtual double getResetRadius()const =0;
			virtual double getAccLatUB()const =0;
			virtual double getAccLatUB_minVelocity()const =0;
			virtual double getAccLonUB()const =0;
			virtual double getAccLonLB()const =0;
			virtual double getFrontTimeGap()const =0;
			virtual double getRearTimeGap()const =0;
			virtual double getFrontSGap()const =0;
			virtual double getRearSGap()const =0;
			virtual double getChaseReferenceOffset()const =0;
			virtual double getLeadReferenceOffset()const =0;
			virtual double getFrontReferenceOffset()const =0;
			virtual double getGapAlignment()const =0;
			virtual double getAssumedNominalAccelerationMinimum()const =0;
			virtual double getMaxNavcostLoss()const=0;
		};
	}
}