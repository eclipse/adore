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
#include <adore/sim/tcd/tlstateparser.h>	

using namespace xercesc;
using namespace adore::sim;

xercesc::DOMDocument* TLStateParser::getCurrentConfiguration()
{
	return tl_configuration_;
}

TLStateParser::TLStateParser(std::string redPhaseDuration,std::string yellowPhaseDuration,
		std::string redYellowPhaseDuration, std::string greenPhaseDuration, 
		std::string startState):red_duration_str_(redPhaseDuration), 
		yellow_duration_str_(yellowPhaseDuration),
		red_yellow_duration_str_(redYellowPhaseDuration),
		green_duration_str_(greenPhaseDuration), start_state_(startState)
{
	init_success_ = false;

	try {
		XMLPlatformUtils::Initialize();
	}
	catch (const XMLException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		LOG_E("Exception message is: %s \n", message );
		XMLString::release(&message);
		return;
	}

	dom_parser_ = new XercesDOMParser;

	dom_parser_->setValidationScheme(XercesDOMParser::Val_Always);

	dom_parser_->setDoNamespaces(true);  

	ErrorHandler* errHandler = (ErrorHandler*) new HandlerBase();
	dom_parser_->setErrorHandler(errHandler);

	init_success_= true;

	LOG_SET_LVL_I();
}

DOMDocument* TLStateParser::createNewDocument(std::string rootElementName)
{
	DOMImplementation * DOMImpl = DOMImplementation::getImplementation();

	return DOMImpl->createDocument(0, XMLString::transcode(rootElementName.c_str()),0);
}

int TLStateParser::writeFile(std::string filePath)
{
	if(tl_configuration_ == NULL || filePath == "")
		return -1;

	LOG_I("Writing file %s .",filePath.c_str());

	DOMImplementation * DOMImpl = DOMImplementation::getImplementation();
	DOMLSSerializer * serializer = DOMImpl->createLSSerializer();

	// Make the output more human readable by inserting line feeds.
	if (serializer->getDomConfig()->canSetParameter(XMLUni::fgDOMWRTFormatPrettyPrint, true))
		serializer->getDomConfig()->setParameter(XMLUni::fgDOMWRTFormatPrettyPrint, true);

	// Convert the path into Xerces compatible XMLCh*.
	XMLCh *tempFilePath = XMLString::transcode(filePath.c_str());

	// Specify the target for the XML output.
	XMLFormatTarget *formatTarget = new LocalFileFormatTarget(tempFilePath);

	DOMLSOutput *output = ((DOMImplementationLS*)DOMImpl)->createLSOutput();

	// Set the stream to our target.
	output->setByteStream(formatTarget);

	// Write the serialized output to the destination.
	serializer->write(tl_configuration_, output);

	// Cleanup.
	serializer->release();
	XMLString::release(&tempFilePath);
	delete formatTarget;
	output->release();

	return 1;
}

DOMNodeList* TLStateParser::searchForNodes(DOMDocument *document, std::string nodeName)
{
	XMLCh * tagName = XMLString::transcode(nodeName.c_str());

	DOMNodeList * listOfSignals = document->getElementsByTagName(tagName);

	XMLString::release(&tagName);

	return listOfSignals;
}

void TLStateParser::readFile(std::string fname, DOMDocument ** domDoc)
{
	if(!init_success_)
	{
		domDoc = nullptr;
		return;
	}

	try {
	
		XMLCh * XMLFileName = XMLString::transcode(fname.c_str());

		LocalFileInputSource source(XMLFileName);

		dom_parser_->parse(fname.c_str());

		*domDoc  = dom_parser_->getDocument();

	}
	catch (const XMLException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		LOG_E("Exception message is: %s \n", message );
		XMLString::release(&message);
		domDoc = nullptr;
	}
	catch (const DOMException& toCatch) {
		char* message = XMLString::transcode(toCatch.msg);
		LOG_E("Exception message is: %s \n", message );
		XMLString::release(&message);
		domDoc = nullptr;
	}
	catch (...) {
		LOG_E("Unexpected Exception \n");
		domDoc = nullptr;
	}

}


 DOMDocument* TLStateParser::readSIGFile(std::string fname)
 {
	 readFile(fname, &tl_configuration_);

	 return tl_configuration_;
 }

DOMDocument* TLStateParser::readXODRFile(std::string fname)
{
	DOMDocument * domDoc;
	
	readFile(fname, &domDoc) ;

	if(domDoc == nullptr)
	{
		LOG_E("Error reading file: %s . Application terminated ..", fname.c_str());
		
		std::exit(0);

		return NULL;
	}

	/* First, get junction id from .xodr to map controllerID (SPAT = MovementID) to intersection_id 
	*
	*	<junction name="" id="10">
	*		<controller id="21" type="0"/>
	*		<controller id="23" type="0"/>
	*		<controller id="22" type="0"/>
	*		<controller id="24" type="0"/>
	*	</junction>
	*/

	DOMNodeList * listOfIntersections = searchForNodes(domDoc,"junction");

	std::unordered_map<int, int> controllerTointersectionMap;

	for(int i = 0; i< listOfIntersections->getLength();i++)
	{
		LOG_I("Processing intersections %d/%d",(i+1),listOfIntersections->getLength());

		xercesc::DOMNode* intersection = listOfIntersections->item(i);

		xercesc::DOMNamedNodeMap* intersectionAttr = intersection->getAttributes();

		int intersectionID = std::atoi(XMLString::transcode(intersectionAttr->getNamedItem(XMLString::transcode("id"))->getNodeValue()));
	
		if(!intersection->hasChildNodes())
			continue;

		xercesc::DOMNodeList* controllerInIntersection=  intersection->getChildNodes();

		for(int j = 0; j < controllerInIntersection->getLength(); j++)
		{
			std::string nodeName = XMLString::transcode(controllerInIntersection->item(j)->getNodeName());

			if(nodeName  != "controller")
				continue;

			xercesc::DOMNode* idNode = controllerInIntersection->item(j)->getAttributes()->getNamedItem(XMLString::transcode("id"));

			int controllerID = std::atoi(XMLString::transcode(idNode->getNodeValue()));

			controllerTointersectionMap[controllerID] = intersectionID;
		}
	}

	/*  Then, get controllers from .xodr to map signal_id to movementID 
	*
	*    <controller name="ctrl018" id="18">
	*		<control signalId="42442" type="0" />
	*		<control signalId="42443" type="0" />
	*		<control signalId="36380" type="0" />
	*		<control signalId="36381" type="0" />
	*	</controller>
	*/

	DOMNodeList * listOfSignalGroups = searchForNodes(domDoc,"controller");

	std::unordered_map<int, int> signalGroupMap;

	for(int i = 0; i< listOfSignalGroups->getLength();i++)
	{
		LOG_I("Processing controller %d/%d",(i+1),listOfSignalGroups->getLength());

		xercesc::DOMNode* controller = listOfSignalGroups->item(i);

		xercesc::DOMNamedNodeMap* controllerAttr = controller->getAttributes();

		int controllerID = std::atoi(XMLString::transcode(controllerAttr->getNamedItem(XMLString::transcode("id"))->getNodeValue()));
	
		if(!controller->hasChildNodes())
			continue;

		xercesc::DOMNodeList* signalsInSignalGroup =  controller->getChildNodes();

		for(int j = 0; j < signalsInSignalGroup->getLength(); j++)
		{
			std::string nodeName = XMLString::transcode(signalsInSignalGroup->item(j)->getNodeName());

			if(nodeName  != "control")
				continue;

			xercesc::DOMNode* idNode = signalsInSignalGroup->item(j)->getAttributes()->getNamedItem(XMLString::transcode("signalId"));

			int signalID = std::atoi(XMLString::transcode(idNode->getNodeValue()));

			signalGroupMap[signalID] = controllerID;
		}
	}

	/*  Then, get signal information and push corresponding intersectionID-Attribute into the newly created signal
	*	additionally to the newly created signal_rotation-tag that enables sim. signal phases
	*	
	*	From:
	*       <signal s="0.0000000000000000e+00" t="5.2999999999999998e+00" id="3040" name="_Sg3040" dynamic="yes" orientation="-" zOffset="0.0000000000000000e+00" type="1000001" country="OpenDRIVE" subtype="-1" hOffset="-1.7453292519943295e-01" pitch="0.0000000000000000e+00" roll="0.0000000000000000e+00" height="3.22" width="0.45">
	*			<dependency id="3042" type="limitline"/>
	*		</signal>
	*
	*	To: 
	*	     <signal country="OpenDRIVE" intersectionID = "2" dynamic="yes" hOffset="-1.7453292519943295e-01" height="3.22" id="3040" name="_Sg3040" orientation="-" pitch="0.0000000000000000e+00" roll="0.0000000000000000e+00" s="0.0000000000000000e+00" subtype="-1" t="5.2999999999999998e+00" type="1000001" width="0.45" zOffset="0.0000000000000000e+00">
	*			<signal_rotation g="3000" r="3000" ry="3000" sstate="r" y="3000"/>
	*		</signal>
	*/

	tl_configuration_ = createNewDocument("signal_list");

	DOMElement* pRoot = tl_configuration_->getDocumentElement();

	DOMNodeList * listOfSignals = searchForNodes(domDoc,"signal");

	for(int i = 0; i < listOfSignals->getLength(); i++)
	{

		DOMNode * node = tl_configuration_->importNode(listOfSignals->item(i),false);

		DOMNamedNodeMap* attMap = node->getAttributes();

		xercesc::DOMNode* dynamicAttr = attMap->getNamedItem(XMLString::transcode("dynamic"));

		std::string isDynamic = XMLString::transcode(dynamicAttr->getNodeValue());

		if( isDynamic == "no")
		{
			LOG_I("Processing node %d/%d. -- SKIPPED (not dynamic)",(i+1),listOfSignals->getLength());

			continue;
		}

		/*
		* Insert intersectionID in signal-tag as attribute
		*/
		int signalID = std::atoi(XMLString::transcode(attMap->getNamedItem(XMLString::transcode("id"))->getNodeValue()));

		int movementID = signalGroupMap[signalID];

		int intersectionID = controllerTointersectionMap[movementID];


		// convert node to DOMElement to have write access to attribute list
		DOMElement* elem = (DOMElement*) node; 

		XMLCh *intersElementName = XMLString::transcode("intersectionID");

		std::string strIntersectionID = std::to_string((long long)intersectionID);

		XMLCh *intersElementValue = XMLString::transcode(strIntersectionID.c_str());

		elem->setAttribute(intersElementName,intersElementValue); 


		XMLCh *movElementName = XMLString::transcode("movementID");

		std::string strMovementID = std::to_string((long long)movementID);

		XMLCh *movElementValue = XMLString::transcode(strMovementID.c_str());

		elem->setAttribute(movElementName,movElementValue); 

		/*
		* calc red/green phase of intersection
		*/

		int GreenTime = std::stoi(green_duration_str_);
		int YellowTime = std::stoi(yellow_duration_str_);
		int RedTime = std::stoi(red_duration_str_);
		int RedYellowTime = std::stoi(red_yellow_duration_str_);
		int fullCycleTime = GreenTime + YellowTime+ RedTime +RedYellowTime;
		std::string startColor = start_state_;

		//if id is odd-numbered, change color
		if(movementID % 2 == 0)
		{
			// when x has red, x+1 can use the time for r,ry and y)
			YellowTime = RedTime / 5;
			RedYellowTime = RedTime / 5;
			GreenTime = (RedTime * 3) / 5;

			// controler x+1 must have red, when x has green, yellow or red-yellow
			RedTime = fullCycleTime - RedTime;

			if(start_state_ == "r")
				start_state_ = "g";
			else
				start_state_ = "r";
		}	

		std::stringstream converter;
		converter << RedTime;

		/*
		* add child-tag to signal, named signal_rotation that holds information for signal phase timing
		*/
		std::string buffer;
		XMLCh *signalRotationString = XMLString::transcode("signal_rotation");
		DOMElement * default_signal_rotation = tl_configuration_->createElement(signalRotationString);
		XMLString::release(&signalRotationString);

		XMLCh *rString = XMLString::transcode("r");
		buffer = tostr(RedTime);
		default_signal_rotation->setAttribute(rString, XMLString::transcode(buffer.c_str()));
		XMLString::release(&rString);

		XMLCh *ryString = XMLString::transcode("ry");
		buffer = tostr(RedYellowTime);
		default_signal_rotation->setAttribute(ryString, XMLString::transcode(buffer.c_str()));
		XMLString::release(&ryString);

		XMLCh *yString = XMLString::transcode("y");
		buffer = tostr(YellowTime);
		default_signal_rotation->setAttribute(yString, XMLString::transcode(buffer.c_str()));
		XMLString::release(&yString);

		XMLCh *gString = XMLString::transcode("g");
		buffer = tostr(GreenTime);
		default_signal_rotation->setAttribute(gString, XMLString::transcode(buffer.c_str()));
		XMLString::release(&gString);

		XMLCh *sstateString = XMLString::transcode("sstate");
		default_signal_rotation->setAttribute(sstateString, XMLString::transcode(start_state_.c_str()));
		XMLString::release(&sstateString);


		node->appendChild(default_signal_rotation);

		try {

			pRoot->appendChild(node);

			LOG_I("Processing node %d/%d. -- ADDED",(i+1),listOfSignals->getLength());

		}
		catch (const DOMException& toCatch) {
			char* message = XMLString::transcode(toCatch.getMessage());
			LOG_E("Exception message is: %s \n", message );
			XMLString::release(&message);

		}
	}
	
	return tl_configuration_;
}
 
std::unordered_map<int,adore::env::SimTrafficLight*> TLStateParser::getTCDObjectListFromConfig(long buildTime)
{
	std::unordered_map<int,adore::env::SimTrafficLight*> tlList;

	//convert the configuration to manageable objects in TCDStateManager
	xercesc::DOMNode* tl =  tl_configuration_->getFirstChild();

	xercesc::DOMNodeList * listOfSignals = tl->getChildNodes();

	for(int i = 0; i< listOfSignals->getLength(); i++)
	{
		xercesc::DOMNode* node = listOfSignals->item(i);

		std::string nodeName = xercesc::XMLString::transcode(node->getNodeName());

		if( nodeName == "signal")
		{
			adore::env::SimTrafficLight * tl = new adore::env::SimTrafficLight();

			xercesc::DOMNamedNodeMap* attrMap =  node->getAttributes();

			xercesc::DOMNode * id = attrMap->getNamedItem(xercesc::XMLString::transcode("id"));

			xercesc::DOMNode * intersectionID = attrMap->getNamedItem(xercesc::XMLString::transcode("intersectionID"));

			xercesc::DOMNode * movementID = attrMap->getNamedItem(xercesc::XMLString::transcode("movementID"));

			xercesc::DOMNode * status = attrMap->getNamedItem(xercesc::XMLString::transcode("type"));

			std::string strID = xercesc::XMLString::transcode(id->getNodeValue());

			std::string strIntersectionID = xercesc::XMLString::transcode(intersectionID->getNodeValue());

			std::string strMovementID = xercesc::XMLString::transcode(movementID->getNodeValue());

			std::string strStatus = xercesc::XMLString::transcode(status->getNodeValue());

			LOG_I("processing %s with id: %s on intersection: %s", nodeName.c_str(), strID.c_str(), strIntersectionID.c_str());

			tl->setID(std::atoi(strID.c_str()));

			//diff between TrafficLight and TrafficLigtPedestrian
			if(strStatus.compare("1000001") == 0) // status for a specific vehicle trafficlight signal
				tl->setType(adore::env::TrafficControlDevice::TRAFFIC_LIGHT);
			else if(strStatus.compare("1000002") == 0)// status for a specific pedestrian trafficlight signal
				tl->setType(adore::env::TrafficControlDevice::TRAFFIC_LIGHT_PEDESTRIAN);
			else
				tl->setType(adore::env::TrafficControlDevice::UNKNOWN);
			

			tl->intersection_id_ = std::atoi(strIntersectionID.c_str());

			tl->movement_id_ = std::atoi(strMovementID.c_str());

			xercesc::DOMNode * signalRotationNode = NULL;

			xercesc::DOMNodeList* subNodes = node->getChildNodes();

			//should be signal rotation
			for(int j = 0; j <  subNodes->getLength(); j++)
			{
				std::string subNode = xercesc::XMLString::transcode(subNodes->item(j)->getNodeName());

				if( subNode == "signal_rotation")
				{
					signalRotationNode = node->getChildNodes()->item(j);
					break;
				}

			}

			if(signalRotationNode == NULL)
			{
				LOG_I("Could not find signal_rotation-tag at signal id %s --- skipping.", strID.c_str());
				delete tl;
				continue;
			}

			xercesc::DOMNamedNodeMap* srAttr = signalRotationNode->getAttributes();

			tl->red_duration_= std::atoi(xercesc::XMLString::transcode(srAttr->getNamedItem(xercesc::XMLString::transcode("r"))->getNodeValue()));

			tl->yellow_red_duration_= std::atoi(xercesc::XMLString::transcode(srAttr->getNamedItem(xercesc::XMLString::transcode("ry"))->getNodeValue()));

			tl->yellow_duration_ = std::atoi(xercesc::XMLString::transcode(srAttr->getNamedItem(xercesc::XMLString::transcode("y"))->getNodeValue()));

			tl->green_duration_ = std::atoi(xercesc::XMLString::transcode(srAttr->getNamedItem(xercesc::XMLString::transcode("g"))->getNodeValue()));

			std::string startMode = xercesc::XMLString::transcode(srAttr->getNamedItem(xercesc::XMLString::transcode("sstate"))->getNodeValue());

			int tmpduration = tl->red_duration_;

			if(startMode == "yf")
			{	
				tl->start_state_ = adore::env::TrafficLightColor::YELLOW_FLASHING;
				tmpduration = INT_MAX;
			}
			else if(startMode == "y")
			{
				tl->start_state_ = adore::env::TrafficLightColor::YELLOW;
				tmpduration = tl->yellow_duration_;
			}
			else if(startMode == "ry")
			{	
				tl->start_state_ =adore::env::TrafficLightColor::RED_YELLOW;
				tmpduration = tl->yellow_red_duration_;

			}
			else if(startMode == "g")
			{	
				tl->start_state_ = adore::env::TrafficLightColor::GREEN;
				tmpduration = tl->green_duration_;
			}
			else tl->start_state_ = adore::env::TrafficLightColor::RED;

			tl->getStatus()->setCurrentColor(tl->start_state_);

			tl->getStatus()->setValidUntilTimestamp(buildTime + tmpduration);

			tlList[tl->getID()] = tl;
		}
	}

	return tlList;
}

std::string TLStateParser::tostr (int integer)
{
	std::stringstream converter;
	converter << integer;
	std::string test23 = converter.str();
	return test23;
}

TLStateParser::~TLStateParser()
{
	delete dom_parser_->getErrorHandler();
	delete dom_parser_;
}

 void TLStateParser::setRedPhaseDuration(std::string durationInMillis)
  {
    red_duration_str_ = durationInMillis;
  }

  void TLStateParser::setRedYellowPhaseDuration(std::string durationInMillis)
  {
    red_yellow_duration_str_ = durationInMillis;
  }

  void TLStateParser::setYellowPhaseDuration(std::string durationInMillis)
  {
    yellow_duration_str_ = durationInMillis;
  }

  void TLStateParser::setGreenPhaseDuration(std::string durationInMillis)
  {
    green_duration_str_ = durationInMillis;
  }

  void TLStateParser::setInitialtState(std::string durationInMillis)
  {
    start_state_ = durationInMillis;
  }