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
 *   Robert Markowski - initial API and implementation
 ********************************************************************************/

#pragma once
#include <string>
#include <map>
#include <adore/if_r2s/r2sauxiliary.h>
#include <adore/env/borderbased/borderset.h>

namespace adore
{
    namespace if_r2s
    {
        /**
         * @brief 
         * input: 2 files, reference lines and borders
         * output: border file
         */
        class R2S2BorderBasedConverter
        {
            public:
            typedef std::map<int,Section> TSectionMap;
            typedef std::map<int,TFunctionTypePair> TDist2Function;
            
            private:
            bool extractOnlyReferenceLines_;
            
            private:
            void group(TReferenceLineVector rlm, TLaneBorderVector lbv, TSectionMap& sectionmap);
            /**
             * @brief manage overall conversion process from sectionmap to borderset
             * 
             * @param sectionmap 
             * @param targetset 
             */
            void convertToBorder(TSectionMap sectionmap, env::BorderBased::BorderSet& targetset);
            /**
             * @brief converts from LaneBorder::TYPE to BorderType::TYPE
             * 
             * @param type 
             * @return env::BorderBased::BorderType::TYPE 
             */
            env::BorderBased::BorderType::TYPE convertLaneBorderType(LaneBorder::TYPE type);
            /**
             * @brief sort borders by their distance to base function which is required to be available through dist2function.at(0)
             * 
             * @param functions 
             * @param dist2function 
             * @param s0 
             * @param s1 
             */
            void sortFunctionsByDistance(std::vector<TFunctionTypePair>& functions, TDist2Function& dist2function, double s0, double s1);
            /**
             * @brief borders for functions in dist2function from s0 to s1
             * 
             * @param dist2function 
             * @param s0 
             * @param s1 
             * @param max_points 
             * @param targetSet 
             * @param inverted 
             */
            void createBorders(TDist2Function& dist2function, double s0, double s1, int max_points, adore::env::BorderBased::BorderSet& targetSet, bool inverted);
            /**
             * @brief actual conversion
             * 
             * @param referenceLineFile 
             * @param laneBorderFile 
             * @param targetset 
             * @param sectionmap 
             */
            void do_convert(std::string referenceLineFile, std::string laneBorderFile, env::BorderBased::BorderSet& targetset, TSectionMap& sectionmap);
            
            public:
            R2S2BorderBasedConverter();
            /**
             * @brief converts sets of ReferenceLine and LaneBorder structs into adore borders
             */
            void toBorders(const TReferenceLineVector& reflines,const TLaneBorderVector& laneborders,env::BorderBased::BorderSet& targetset);
            /**
             * @brief convert to borders
             * 
             * @param referenceLineFile 
             * @param laneBorderFile 
             * @param targetset 
             */
            void convert(std::string referenceLineFile, std::string laneBorderFile, env::BorderBased::BorderSet& targetset);
            /**
             * @brief convert to borders and get sectionmap with base data
             * 
             * @param referenceLineFile 
             * @param laneBorderFile 
             * @param targetset 
             * @param sectionmap 
             */
            void convert(std::string referenceLineFile, std::string laneBorderFile, env::BorderBased::BorderSet& targetset, TSectionMap& sectionmap);
            /**
             * @brief cleanup leftover functions in sectionmap
             * 
             * @param sectionmap 
             */
            void cleanup(TSectionMap& sectionmap);
            /**
             * @brief set option to only convert reference lines and ignore laneborders
             * 
             * @param b 
             */
            void setExtractOnlyReferenceLines(bool b);
        };
    }
}