/*
 *
 * Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)
 * Institute of Transportation Systems. (https://www.dlr.de/ts/)
 *
 * 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 * 
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 * 
 * SPDX-License-Identifier: EPL-2.0
 * 
 *
 * 
 * File automatically generated with DLR Wind v2 (2021)
 * 
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:odds:12.0
 * 
 * Module: ODDInfo {}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */

#include <odds_oddinfo_translator_ros2wind.h>

void wind::wind_ros::ros2wind(const odds_oddinfo::ODDMSG::ConstPtr& ros, wind::cpp::ODDInfo::ODDMSG* wind)
{

    wind->header.protocolVersion.value =
        ros->header.protocolVersion.value;

    wind->header.messageID.value =
        ros->header.messageID.value;

    wind->header.stationID.value =
        ros->header.stationID.value;

    wind->oddinfo.status.value =
        ros->oddinfo.status.value;

    wind->oddinfo.creationTime.value =
        ros->oddinfo.creationTime.value;

    wind->oddinfo.serverTime.value =
        ros->oddinfo.serverTime.value;

    // Start SEQUENCE OF SectionObjectList
    wind->oddinfo.odds.sectionObjects.count =
        ros->oddinfo.odds.sectionObjects.count;

    int count_a = ros->oddinfo.odds.sectionObjects.count;

    for(int a = 0; a < count_a; a++) {
        wind->oddinfo.odds.sectionObjects.elements[a].opConditionsPresent =
            ros->oddinfo.odds.sectionObjects.elements[a].opConditionsPresent;
        wind->oddinfo.odds.sectionObjects.elements[a].envConditionsPresent =
            ros->oddinfo.odds.sectionObjects.elements[a].envConditionsPresent;


        for(int b = 0; b < 63; b++) {  // Section
            if(b < ros->oddinfo.odds.sectionObjects.elements[a].sectionType.value.length())
                wind->oddinfo.odds.sectionObjects.elements[a].sectionType.value[b] = ros->oddinfo.odds.sectionObjects.elements[a].sectionType.value.c_str()[b];
            else
                wind->oddinfo.odds.sectionObjects.elements[a].sectionType.value[b] = ' ';
        }

        wind->oddinfo.odds.sectionObjects.elements[a].id.value =
            ros->oddinfo.odds.sectionObjects.elements[a].id.value;

        wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.latitude.value =
            ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.latitude.value;

        wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.longitude.value =
            ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.longitude.value;

        wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.bearing.value =
            ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.bearing.value;

        wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.latitude.value =
            ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.latitude.value;

        wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.longitude.value =
            ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.longitude.value;

        wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.bearing.value =
            ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.bearing.value;

        wind->oddinfo.odds.sectionObjects.elements[a].validityInfo.startTime.value =
            ros->oddinfo.odds.sectionObjects.elements[a].validityInfo.startTime.value;

        wind->oddinfo.odds.sectionObjects.elements[a].validityInfo.expirationTime.value =
            ros->oddinfo.odds.sectionObjects.elements[a].validityInfo.expirationTime.value;

        wind->oddinfo.odds.sectionObjects.elements[a].validityInfo.confidence.value =
            ros->oddinfo.odds.sectionObjects.elements[a].validityInfo.confidence.value;
        wind->oddinfo.odds.sectionObjects.elements[a].staticInf.zonePresent =
            ros->oddinfo.odds.sectionObjects.elements[a].staticInf.zonePresent;
        wind->oddinfo.odds.sectionObjects.elements[a].staticInf.boundaryPresent =
            ros->oddinfo.odds.sectionObjects.elements[a].staticInf.boundaryPresent;
        
        // START BIT STRING InfraCategory
        for(int c = 0; c < 3; c++) {
            wind->oddinfo.odds.sectionObjects.elements[a].staticInf.category.values[c] = 
                ros->oddinfo.odds.sectionObjects.elements[a].staticInf.category.values[c];
        }
        // END BIT STRING InfraCategory
        
        if(ros->oddinfo.odds.sectionObjects.elements[a].staticInf.zonePresent) {
            
            // START BIT STRING InfraZone
            for(int d = 0; d < 8; d++) {
                wind->oddinfo.odds.sectionObjects.elements[a].staticInf.zone.values[d] = 
                    ros->oddinfo.odds.sectionObjects.elements[a].staticInf.zone.values[d];
            }
            // END BIT STRING InfraZone
            
        }

        wind->oddinfo.odds.sectionObjects.elements[a].staticInf.sigNode.value =
            ros->oddinfo.odds.sectionObjects.elements[a].staticInf.sigNode.value;
        if(ros->oddinfo.odds.sectionObjects.elements[a].staticInf.boundaryPresent) {
            
            // START BIT STRING InfraBoundary
            for(int e = 0; e < 4; e++) {
                wind->oddinfo.odds.sectionObjects.elements[a].staticInf.boundary.values[e] = 
                    ros->oddinfo.odds.sectionObjects.elements[a].staticInf.boundary.values[e];
            }
            // END BIT STRING InfraBoundary
            
        }
        if(ros->oddinfo.odds.sectionObjects.elements[a].opConditionsPresent) {
            wind->oddinfo.odds.sectionObjects.elements[a].opConditions.eventsPresent =
                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.eventsPresent;
            wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficConditionPresent =
                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficConditionPresent;
            wind->oddinfo.odds.sectionObjects.elements[a].opConditions.dspPresent =
                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.dspPresent;
            wind->oddinfo.odds.sectionObjects.elements[a].opConditions.i2VPresent =
                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.i2VPresent;
            wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehiclePresent =
                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehiclePresent;
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.eventsPresent) {
                
                // START BIT STRING InfraEvents
                for(int f = 0; f < 4; f++) {
                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.events.values[f] = 
                        ros->oddinfo.odds.sectionObjects.elements[a].opConditions.events.values[f];
                }
                // END BIT STRING InfraEvents
                
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficConditionPresent) {
                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTrafficPresent =
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTrafficPresent;
                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.losPresent =
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.losPresent;
                if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTrafficPresent) {

                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTraffic.value =
                        ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTraffic.value;
                }
                if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.losPresent) {
                    
                    // START BIT STRING InfraTrafficLOS
                    for(int g = 0; g < 4; g++) {
                        wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.los.values[g] = 
                            ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.los.values[g];
                    }
                    // END BIT STRING InfraTrafficLOS
                    
                }
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.dspPresent) {

                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.dsp.value =
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.dsp.value;
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.i2VPresent) {
                
                // START BIT STRING InfraI2V
                for(int h = 0; h < 5; h++) {
                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.i2V.values[h] = 
                        ros->oddinfo.odds.sectionObjects.elements[a].opConditions.i2V.values[h];
                }
                // END BIT STRING InfraI2V
                
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehiclePresent) {

                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePresent.value =
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePresent.value;

                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.latitude.value =
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.latitude.value;

                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.longitude.value =
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.longitude.value;

                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.bearing.value =
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.bearing.value;
            }
        }
        if(ros->oddinfo.odds.sectionObjects.elements[a].envConditionsPresent) {
            wind->oddinfo.odds.sectionObjects.elements[a].envConditions.weatherPresent =
                ros->oddinfo.odds.sectionObjects.elements[a].envConditions.weatherPresent;
            wind->oddinfo.odds.sectionObjects.elements[a].envConditions.lightingPresent =
                ros->oddinfo.odds.sectionObjects.elements[a].envConditions.lightingPresent;
            wind->oddinfo.odds.sectionObjects.elements[a].envConditions.obstaclePresent =
                ros->oddinfo.odds.sectionObjects.elements[a].envConditions.obstaclePresent;
            if(ros->oddinfo.odds.sectionObjects.elements[a].envConditions.weatherPresent) {
                
                // START BIT STRING WeatherCondition
                for(int i = 0; i < 6; i++) {
                    wind->oddinfo.odds.sectionObjects.elements[a].envConditions.weather.values[i] = 
                        ros->oddinfo.odds.sectionObjects.elements[a].envConditions.weather.values[i];
                }
                // END BIT STRING WeatherCondition
                
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].envConditions.lightingPresent) {
                
                // START BIT STRING LightingCondition
                for(int j = 0; j < 3; j++) {
                    wind->oddinfo.odds.sectionObjects.elements[a].envConditions.lighting.values[j] = 
                        ros->oddinfo.odds.sectionObjects.elements[a].envConditions.lighting.values[j];
                }
                // END BIT STRING LightingCondition
                
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].envConditions.obstaclePresent) {
                
                // START BIT STRING ObstacleInformation
                for(int k = 0; k < 3; k++) {
                    wind->oddinfo.odds.sectionObjects.elements[a].envConditions.obstacle.values[k] = 
                        ros->oddinfo.odds.sectionObjects.elements[a].envConditions.obstacle.values[k];
                }
                // END BIT STRING ObstacleInformation
                
            }
        }
    }
    // End Sequence of SectionObjectList
}
