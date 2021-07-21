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
#include <vector>
#include <map>
#include <adore/if_r2s/r2sauxiliary.h>

namespace adore
{
    namespace if_r2s
    {
        class R2SFileReader
        {
            public:
            static void readReferenceLines(std::string filestring, TReferenceLineVector& referenceLines);
            static void readLaneBorders(std::string filestring, TLaneBorderVector& laneBorders);
            
            private:            
            static std::string stripChar(std::string token, char c);
            static void readGeometryStream(std::stringstream& linestream, TR2SGeometry& geometry);
            static LaneBorder::TYPE convertLaneBorderType(std::string token);
            
            
        };
    }
}