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
#include <adore/if_r2s/r2sfilereader.h>
#include <adore/mad/csvlog.h>

#include <algorithm>
#include <fstream>
#include <sstream>


namespace adore
{
    namespace if_r2s
    {
        std::string R2SFileReader::stripChar(std::string token, char c)
        {
            if(token.front()==c)
            {
                token.erase(token.begin());
            }
            if(token.back()==c)
            {                
                token.pop_back();
            }
            return token;
        }
        void R2SFileReader::readGeometryStream(std::stringstream& linestream, TR2SGeometry& geometry)
        {
            /* structure: LINESTRING(x0 y0, x1 y1, ...) */
            std::string token;
            std::getline(linestream,token,'(');
            std::getline(linestream,token,')');
            std::stringstream tokenstream(token);
            std::string subtoken;
            while(std::getline(tokenstream,subtoken,','))
            {
                std::stringstream subtokenstream(subtoken);
                double x,y;
                subtokenstream >> x >> y;
                geometry.push_back(adore::env::BorderBased::Coordinate(x,y,0.0));
                // LOG_T("x: %.2f, y: %.2f", x, y);
            }
            std::getline(linestream,token,',');            
        }
        LaneBorder::TYPE R2SFileReader::convertLaneBorderType(std::string token)
        {
            token = stripChar(token,'"');
            if(token.compare("driving")==0) 
            {
                return LaneBorder::DRIVING;
            }
            return LaneBorder::OTHER;
        }
        void R2SFileReader::readLaneBorders(std::string filestring, TLaneBorderVector& laneBorders)
        {
            std::ifstream file;
            try
            {
                file.open(filestring);
            }
            catch(...)
            {
                LOG_E("Could not open lane border file: %s", filestring.c_str());
                return;
            }
            try
            {
                std::string line;
                /* first line is header */
                std::getline(file,line);
                /* read all lines */
                while(std::getline(file,line))
                {
                    LOG_T("Reading lane border...");
                    LaneBorder lb;
                    std::stringstream linestream(line);
                    std::string token;
                    
                    /* id */
                    std::getline(linestream,token,',');
                    token = stripChar(token,'"');
                    lb.id_=stoi(token);
                    LOG_T("ID: %i", lb.id_);
                    
                    /* geometry */
                    readGeometryStream(linestream,lb.geometry_);
                    LOG_T("#points = %zi", lb.geometry_.size());
                    
                    /* type */
                    std::getline(linestream,token,',');
                    lb.type_=convertLaneBorderType(token);
                    LOG_T("type: %i", lb.type_);
                    
                    /* material */
                    std::getline(linestream,token,',');
                    
                    /* datasourcedescription_id */
                    std::getline(linestream,token,',');
                    
                    /* parent_id */
                    std::getline(linestream,token,',');
                    token = stripChar(token,'"');
                    lb.parent_id_ = stoi(token);
                    LOG_T("parent: %i", lb.parent_id_);
                    
                    laneBorders.push_back(lb);
                }
            }
            catch(...)
            {
                LOG_E("Error while parsing lane border file: %s", filestring.c_str());
            }
        }
        void R2SFileReader::readReferenceLines(std::string filestring, TReferenceLineVector& referenceLines)
        {
            std::ifstream file;
            try
            {
                file.open(filestring);
            }
            catch(...)
            {
                LOG_E("Could not open reference line file: %s", filestring.c_str());
                return;
            }
            try
            {
                std::string line;
                /* first line is header */
                std::getline(file,line);
                /* read all lines */
                while(std::getline(file,line))
                {
                    LOG_T("Reading reference line...");
                    ReferenceLine rl;
                    /* read tokens */
                    std::stringstream linestream(line);
                    std::string token;

                    /* id */
                    std::getline(linestream,token,',');
                    token = stripChar(token,'"');
                    rl.id_ = stoi(token);
                    LOG_T("ID: %i", rl.id_);
                    
                    /* geometry */
                    readGeometryStream(linestream,rl.geometry_);
                    LOG_T("#points = %zi", rl.geometry_.size());
                    
                    /* linetype */
                    std::getline(linestream,token,',');
                    token = stripChar(token,'"');
                    if(token.compare("StandardLine")==0)
                    {
                        rl.linetype_ = ReferenceLine::StandardLine;
                    }
                    else
                    {
                        rl.linetype_ = ReferenceLine::ConnectionLine;
                    }                    
                    
                    /* oneway */
                    std::getline(linestream,token,',');
                    if(token.compare("True")==0) {rl.oneway_ = true;}
                    else {rl.oneway_ = false;}
                    LOG_T("oneway: %i", rl.oneway_);
                    
                    /* category */
                    std::getline(linestream,token,',');
                    
                    /* turn */
                    std::getline(linestream,token,',');
                    
                    /* datasourcedescription_id */
                    std::getline(linestream,token,',');
                    
                    /* predecessor_id */
                    std::getline(linestream,token,',');
                    
                    /* successor_id */
                    std::getline(linestream,token,',');

                    // if(!rl.oneway_ || rl.linetype_==ReferenceLine::ConnectionLine) continue;
                    referenceLines.push_back(rl);
                }
            }
            catch(...)
            {
                LOG_E("Error while parsing reference line file: %s", filestring.c_str());
            }            
        }
    }
}