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

#include <OpenDRIVE_1.4H.h>
#include <unordered_map>
#include <adore/env/borderbased/border.h>
#include <vector>


namespace adore
{
	namespace if_xodr
	{

		/**
		 * @brief BorderIDTranslation is a set of translation tables, which keeps track of road ids, junction ids and their geometric representation as borders
		 */
        class BorderIDTranslation
        {
            public:
            // typedef long long int TID;//integral data type used for integral ids
            typedef std::string TID;
            typedef adore::env::BorderBased::BorderID BorderID;
            typedef adore::env::BorderBased::BorderIDHasher BorderIDHasher;
			typedef std::unordered_map<BorderID, TID, BorderIDHasher> BorderID2TID;
			typedef std::unordered_multimap<TID,BorderID> TID2MultiBorderID;
            typedef std::unordered_map<TID,TID> TIDmap;
            typedef std::unordered_multimap<TID,TID> TIDmultimap;
            typedef TID2MultiBorderID::iterator Iterator;
            typedef std::pair<Iterator,Iterator> IteratorPair;
            typedef std::vector<BorderID> BorderIDVector;

            private:
            BorderID2TID borderID2roadID_;// borderID -> roadID
            TID2MultiBorderID roadID2borderID_;// roadID -> {borderID1,...,borderIDn}
            TIDmap roadID2junctionID_;//roadID -> junctionID (maps to a junctionID, if road is a connection road of a junction)
            TIDmultimap junctionID2roadID_;//junctionID -> {roadID1,...,roadIDn} (maps from a junctionid to all connecting roads)
            TID default_empty_id_;

            public:
            /**
             * insert entry into tables: link borderID and roadID
             */
            void insert(const BorderID& borderID,TID roadID)
            {
                borderID2roadID_.emplace(borderID,roadID);
                roadID2borderID_.emplace(roadID,borderID);
            }
            /**
             * insert entry into tables: link junctionID and roadID
             */
            void insert(TID junctionID,TID roadID)
            {
                roadID2junctionID_.emplace(roadID,junctionID);
                junctionID2roadID_.emplace(junctionID,roadID);
            }

            /**
             * get all BorderIDs associated with a roadID
             */
            IteratorPair getBorderIDsOfRoad(TID roadID)
            {
                return std::make_pair(roadID2borderID_.find(roadID),roadID2borderID_.end());
            }


            /**
             * get all BorderIDs associated with a roadID
             */
            BorderIDVector getBorderIDsOfJunction(TID junctionID)
            {
                BorderIDVector borderIDs;
                for(auto it = junctionID2roadID_.find(junctionID);it!=junctionID2roadID_.end();it++)
                {
                    for(auto it2 = roadID2borderID_.find(it->second);it2!=roadID2borderID_.end();it2++)
                    {
                        borderIDs.push_back(it2->second);
                    }
                }
                return borderIDs;
            }

            /**
             * get the associated roadID of a BorderID
             */
            TID getRoadID(const BorderID& borderID)
            {
                auto it = borderID2roadID_.find(borderID);
                if(it==borderID2roadID_.end())return default_empty_id_;
                else return it->second;
            }

            /**
             * get the associated junctionID of a BorderID
             */
            TID getJunctionID(const BorderID& borderID)
            {
                auto it = borderID2roadID_.find(borderID);
                if(it==borderID2roadID_.end())return default_empty_id_;
                auto it2 = roadID2junctionID_.find(it->second);
                if(it2==roadID2junctionID_.end())return default_empty_id_;
                else return it2->second;
            }

            /**
             * check whether a road is a connecting road inside a junction
             */
            bool isInJunction(TID roadID)
            {
                return roadID2junctionID_.find(roadID)!=roadID2junctionID_.end();
            }
            
            /**
             * check whether a borderID belongs to a connecting road inside a junction
             */
            bool isInJunction(const BorderID& borderID)
            {
                auto it = borderID2roadID_.find(borderID);
                if(it==borderID2roadID_.end())return false;
                else return roadID2junctionID_.find(it->second)!=roadID2junctionID_.end();
            }
            
        };
    }
}