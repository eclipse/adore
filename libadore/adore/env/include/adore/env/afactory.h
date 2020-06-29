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
 * 	  Robert Markowski - extension of API
 ********************************************************************************/


#pragma once

#include <adore/env/borderbased/border.h>
#include <adore/fun/navigationgoal.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/env/traffic/participant.h>
#include <adore/mad/com_patterns.h>
#include <adore/env/map/precedence.h>
#include <adore/env/situation/proposition.h>

namespace adore 
{
	namespace env
	{
		///abstract factory for adore::env communication
		class AFactory
		{
		public:
			typedef adore::mad::AFeed<BorderBased::Border> TBorderFeed;
			typedef adore::mad::AWriter<BorderBased::Border> TBorderWriter;
			typedef adore::mad::AReader<VehicleMotionState9d> TVehicleMotionStateReader;
			typedef adore::mad::AReader<traffic::TParticipantSet> TParticipantSetReader;
			typedef adore::mad::AReader<adore::fun::NavigationGoal> TNavigationGoalReader;
			typedef adore::mad::AWriter<std::pair<adore::env::BorderBased::BorderID,double>> TNavgationDataWriter;
			typedef adore::mad::AFeed<std::pair<adore::env::BorderBased::BorderID,double>> TNavigationDataFeed;
			typedef adore::mad::AWriter<adore::env::PrecedenceRule> TPrecedenceRuleWriter;
			typedef adore::mad::AFeed<adore::env::PrecedenceRule> TPrecedenceRuleFeed;
			typedef adore::mad::AWriter<Proposition> TPropositionWriter;
			typedef adore::mad::AFeed<Proposition> TPropositionFeed;

			// write border
			virtual TBorderWriter* getBorderWriter()=0;
			// read borders
			virtual TBorderFeed* getBorderFeed()=0;

			// env vehicle state -> odemetry msg
			// vehicle state 9d in adore::env (um Link gegen FUN zu vermeiden)
			// füllen mit Daten aus Odometry so weit wie möglich
			virtual TVehicleMotionStateReader* getVehicleMotionStateReader()=0;

			// read a set of traffic participants - provides the current, complete set of tracked objects
			virtual TParticipantSetReader* getTrafficParticipantSetReader()=0;

			// read navigation goal
			virtual TNavigationGoalReader* getNavigationGoalReader()=0;

			// write navigation data
			virtual TNavgationDataWriter* getNavigationDataWriter()=0;

			// read navigation data
			virtual TNavigationDataFeed* getNavigationDataFeed()=0;

			// write precedence rules
			virtual TPrecedenceRuleWriter* getPrecedenceRuleWriter()=0;

			// read precedence rules
			virtual TPrecedenceRuleFeed* getPrecedenceRuleFeed()=0;

			// write propositional logic information
			virtual TPropositionWriter* getPropositionWriter()=0;

			// read propositional logic information
			virtual TPropositionFeed* getPropositionFeed()=0;
		};
	}
}