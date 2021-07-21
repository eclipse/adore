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
#include <adore/view/navigationgoal.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/env/traffic/participant.h>
#include <adore/mad/com_patterns.h>
#include <adore/env/map/precedence.h>
#include <adore/env/situation/proposition.h>
#include <adore/env/tcd/controlledconnection.h>
#include <adore/env/traffic/occupancycylinderprediction.h>
#include <adore/env/borderbased/lanecombinedgeometry.h>
#include <adore/env/situation/areaofeffect.h>

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
			typedef adore::mad::AFeed<ControlledConnection> TControlledConnectionFeed;
			typedef adore::mad::AWriter<ControlledConnection> TControlledConnectionWriter;
			typedef adore::mad::AReader<OccupancyCylinderPredictionSet> TOCPredictionSetReader;
			typedef adore::mad::AWriter<OccupancyCylinderPredictionSet> TOCPredictionSetWriter;
			typedef adore::mad::AFeed<adore::env::BorderBased::CombinedLaneGeometry> TLaneGeometryFeed;
			typedef adore::mad::AWriter<adore::env::BorderBased::CombinedLaneGeometry> TLaneGeometryWriter;
			typedef adore::mad::AWriter<adore::env::AreaOfEffect> TAreaOfEffectWriter;

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

			// read traffic light information: controlled connection specifies state of a connection, which is controlled by for example a traffic light
			virtual TControlledConnectionFeed* getControlledConnectionFeed()=0;

			// read checkpoint information represented as controlled connections
			virtual TControlledConnectionFeed* getCheckPointFeed()=0;

			//write checkpoint information represented as controlled connections
			virtual TControlledConnectionWriter* getCheckPointWriter()=0;

			// read the latest prediction set for expected behavior
			virtual TOCPredictionSetReader* getExpectedPredictionSetReader()=0;

			// read the latest prediction set for worst-case behavior
			virtual TOCPredictionSetReader* getWorstCasePredictionSetReader()=0;

			// read the latest prediction set for expected behavior, unfiltered
			virtual TOCPredictionSetReader* getExpectedRawPredictionSetReader()=0;

			// read the latest prediction set for worst-case behavior, unfiltered
			virtual TOCPredictionSetReader* getWorstCaseRawPredictionSetReader()=0;

			// read the latest prediction set for desired behavior
			virtual TOCPredictionSetReader* getDesiredPredictionSetReader()=0;

			// write the latest prediction set for expected behavior
			virtual TOCPredictionSetWriter* getExpectedPredictionSetWriter()=0;

			// write the latest prediction set for worst-case behavior
			virtual TOCPredictionSetWriter* getWorstCasePredictionSetWriter()=0;

			// write the latest prediction set for expected behavior, unfiltered
			virtual TOCPredictionSetWriter* getExpectedRawPredictionSetWriter()=0;

			// write the latest prediction set for worst-case behavior, unfiltered
			virtual TOCPredictionSetWriter* getWorstCaseRawPredictionSetWriter()=0;

			// write the latest prediction set for desired behavior
			virtual TOCPredictionSetWriter* getDesiredPredictionSetWriter()=0;

			// // read lane geometry
			virtual TLaneGeometryFeed* getLaneGeometryFeed()=0;

			// write lane geometry
			virtual TLaneGeometryWriter* getLaneGeometryWriter()=0;

			// write the area of effect
			virtual TAreaOfEffectWriter* getAreaOfEffectWriter()=0;
		};


        /**
         * @brief Utility class to simplify factory access
         *
         * init() function should be used once per process to set factories
         */
        class EnvFactoryInstance final
        {
          private:
            adore::env::AFactory* envFactory_ = 0;

            /**
             * @brief Function to access singleton instance of the envFactory using magic static
             *
             * @return envFactory& reference on the threadsafe singleton instance
             */
            static EnvFactoryInstance& getInstance()
            {
                static EnvFactoryInstance instance;
                return instance;
            }

          public:
            static adore::env::AFactory* get()
            {
                if (getInstance().envFactory_ == 0)
                {
                    std::cerr << " WARNING Accessing singleton envFactory instance without ever running init()"
                              << std::endl;
                }
                return getInstance().envFactory_;
            }

            /**
             * @brief Initialize private members of AllFactory
             *
             * This function should ideally run only once per process before the singleton instance is used
             * Makes no guarantees on thread-safety
             *
             * @param envFactory
             */
            static void init(adore::env::AFactory* envFactory)
            {
				auto& instance = getInstance();
                instance.envFactory_ = envFactory;
            }

          private:
            EnvFactoryInstance() = default;
            ~EnvFactoryInstance() = default;

            EnvFactoryInstance(const EnvFactoryInstance&) = delete;
            EnvFactoryInstance& operator=(const EnvFactoryInstance&) = delete;
            EnvFactoryInstance(EnvFactoryInstance&&) = delete;
            EnvFactoryInstance& operator=(EnvFactoryInstance&&) = delete;
        };

	}
}
