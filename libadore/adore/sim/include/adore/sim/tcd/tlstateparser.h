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
 *   Stephan Lapoehn - initial API and implementation
 ********************************************************************************/
#pragma once

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/framework/LocalFileInputSource.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <string>
#include <iostream>
#include <unordered_map>
#include <adore/mad/csvlog.h>
#include <adore/env/tcd/trafficlight.h>

namespace adore
{
namespace sim
{
/**
 * TLStateParser is used to generate a .sig (signal) file from .xodr maps which can be extendet with
 * signal phase data (like red-phase duration etc.). Alternativly, this tool can be used to automatically
 * apply default vlaues while reading xodr files.
 */
class TLStateParser
{

private:
  bool init_success_;

  xercesc::DOMDocument* tl_configuration_;

  xercesc::XercesDOMParser* dom_parser_;

  xercesc::DOMDocument* createNewDocument(std::string rootElementName);

  xercesc::DOMNodeList* searchForNodes(xercesc::DOMDocument* document, std::string nodeName);

  xercesc::DOMDocument* getCurrentConfiguration();

  void readFile(std::string fname, xercesc::DOMDocument ** domDoc);

public:

  std::string tostr(int);

  std::string red_duration_str_;

  std::string yellow_duration_str_;

  std::string red_yellow_duration_str_;

  std::string green_duration_str_;

  std::string start_state_;

/**
 * writes traffic light configuration to specified path
 * @param filePath path to file
 */
  int writeFile(std::string fname);

/**
 * reads *.sig files that contain relevant informations for trafficlights, related controllers and intersections
 * @param fname path to the *.sig - file 
 * @return pointer to the data structures of the .sig file (do not free pointer -> TLStateParser is the owner)
 */
  xercesc::DOMDocument* readSIGFile(std::string fname);

/**
 * reads *.xodr files that contain relevant informations for trafficlights, related controllers and intersections
 * and converts it into a .sig file
 * @param fname path to the *.xodr - file
 * @return pointer to the data structures of the .sig file (do not free pointer -> TLStateParser is the owner) 
 */
  xercesc::DOMDocument* readXODRFile(std::string fname);

/**
 * read current configuration @see TLStateParser::readSIGFile / @see readXODRFile and generate corresponding trafficlight objects
 * @param buildTime the point in time the tlstate parser was initiated (will be used as starting point for the state calculations of each traffic light)
 * @return map of TrafficLights and corresponding intersection ids.
 */ 
  std::unordered_map<int, adore::env::SimTrafficLight*> getTCDObjectListFromConfig(long buildTime);

  TLStateParser(std::string redPhaseDuration = "3000", std::string yellowPhaseDuration = "3000",
                std::string redYellowPhaseDuration = "3000", std::string greenPhaseDuration = "3000",
                std::string startState = "r");

  ~TLStateParser();

  void setRedPhaseDuration(std::string durationInMillis);

  void setRedYellowPhaseDuration(std::string durationInMillis);

  void setYellowPhaseDuration(std::string durationInMillis);

  void setGreenPhaseDuration(std::string durationInMillis);

  void setInitialtState(std::string durationInMillis);
};
}  // namespace sim
}  // namespace adore
