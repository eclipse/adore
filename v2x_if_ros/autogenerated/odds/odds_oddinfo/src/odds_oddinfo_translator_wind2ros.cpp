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

#include <odds_oddinfo_translator_wind2ros.h>

void wind::wind_ros::wind2ros(odds_oddinfo::ODDMSG* ros, wind::cpp::ODDInfo::ODDMSG* wind)
{

    ros->header.protocolVersion.value =
        wind->header.protocolVersion.value;

    ros->header.messageID.value =
        wind->header.messageID.value;

    ros->header.stationID.value =
        wind->header.stationID.value;

    ros->oddinfo.status.value =
        wind->oddinfo.status.value;

    ros->oddinfo.creationTime.value =
        wind->oddinfo.creationTime.value;

    ros->oddinfo.serverTime.value =
        wind->oddinfo.serverTime.value;

    // Start SEQUENCE OF SectionObjectList
    ros->oddinfo.odds.sectionObjects.count =
        wind->oddinfo.odds.sectionObjects.count;

    int count_a = ros->oddinfo.odds.sectionObjects.count;

    for(int a = 0; a < count_a; a++) {
        odds_oddinfo::SectionObject tmp_a;
        ros->oddinfo.odds.sectionObjects.elements.push_back(tmp_a);
        ros->oddinfo.odds.sectionObjects.elements[a].opConditionsPresent =
            wind->oddinfo.odds.sectionObjects.elements[a].opConditionsPresent;
        ros->oddinfo.odds.sectionObjects.elements[a].envConditionsPresent =
            wind->oddinfo.odds.sectionObjects.elements[a].envConditionsPresent;

        ros->oddinfo.odds.sectionObjects.elements[a].sectionType.value.assign(wind->oddinfo.odds.sectionObjects.elements[a].sectionType.value);

        ros->oddinfo.odds.sectionObjects.elements[a].id.value =
            wind->oddinfo.odds.sectionObjects.elements[a].id.value;

        ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.latitude.value =
            wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.latitude.value;

        ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.longitude.value =
            wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.longitude.value;

        ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.bearing.value =
            wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.startPosition.bearing.value;

        ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.latitude.value =
            wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.latitude.value;

        ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.longitude.value =
            wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.longitude.value;

        ros->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.bearing.value =
            wind->oddinfo.odds.sectionObjects.elements[a].geoInfo.endPosition.bearing.value;

        ros->oddinfo.odds.sectionObjects.elements[a].validityInfo.startTime.value =
            wind->oddinfo.odds.sectionObjects.elements[a].validityInfo.startTime.value;

        ros->oddinfo.odds.sectionObjects.elements[a].validityInfo.expirationTime.value =
            wind->oddinfo.odds.sectionObjects.elements[a].validityInfo.expirationTime.value;

        ros->oddinfo.odds.sectionObjects.elements[a].validityInfo.confidence.value =
            wind->oddinfo.odds.sectionObjects.elements[a].validityInfo.confidence.value;
        ros->oddinfo.odds.sectionObjects.elements[a].staticInf.zonePresent =
            wind->oddinfo.odds.sectionObjects.elements[a].staticInf.zonePresent;
        ros->oddinfo.odds.sectionObjects.elements[a].staticInf.boundaryPresent =
            wind->oddinfo.odds.sectionObjects.elements[a].staticInf.boundaryPresent;
        
        // START BIT STRING InfraCategory
        for(int b = 0; b < 3; b++) {
            uint8_t tmp_b;
            ros->oddinfo.odds.sectionObjects.elements[a].staticInf.category.values.push_back(tmp_b);
            ros->oddinfo.odds.sectionObjects.elements[a].staticInf.category.values[b] = 
                wind->oddinfo.odds.sectionObjects.elements[a].staticInf.category.values[b];
        }
        // END BIT STRING InfraCategory
        
        if(ros->oddinfo.odds.sectionObjects.elements[a].staticInf.zonePresent) {
            
            // START BIT STRING InfraZone
            for(int c = 0; c < 8; c++) {
                uint8_t tmp_c;
                ros->oddinfo.odds.sectionObjects.elements[a].staticInf.zone.values.push_back(tmp_c);
                ros->oddinfo.odds.sectionObjects.elements[a].staticInf.zone.values[c] = 
                    wind->oddinfo.odds.sectionObjects.elements[a].staticInf.zone.values[c];
            }
            // END BIT STRING InfraZone
            
        }

        ros->oddinfo.odds.sectionObjects.elements[a].staticInf.sigNode.value =
            wind->oddinfo.odds.sectionObjects.elements[a].staticInf.sigNode.value;
        if(ros->oddinfo.odds.sectionObjects.elements[a].staticInf.boundaryPresent) {
            
            // START BIT STRING InfraBoundary
            for(int d = 0; d < 4; d++) {
                uint8_t tmp_d;
                ros->oddinfo.odds.sectionObjects.elements[a].staticInf.boundary.values.push_back(tmp_d);
                ros->oddinfo.odds.sectionObjects.elements[a].staticInf.boundary.values[d] = 
                    wind->oddinfo.odds.sectionObjects.elements[a].staticInf.boundary.values[d];
            }
            // END BIT STRING InfraBoundary
            
        }
        if(ros->oddinfo.odds.sectionObjects.elements[a].opConditionsPresent) {
            ros->oddinfo.odds.sectionObjects.elements[a].opConditions.eventsPresent =
                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.eventsPresent;
            ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficConditionPresent =
                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficConditionPresent;
            ros->oddinfo.odds.sectionObjects.elements[a].opConditions.dspPresent =
                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.dspPresent;
            ros->oddinfo.odds.sectionObjects.elements[a].opConditions.i2VPresent =
                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.i2VPresent;
            ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehiclePresent =
                wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehiclePresent;
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.eventsPresent) {
                
                // START BIT STRING InfraEvents
                for(int e = 0; e < 4; e++) {
                    uint8_t tmp_e;
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.events.values.push_back(tmp_e);
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.events.values[e] = 
                        wind->oddinfo.odds.sectionObjects.elements[a].opConditions.events.values[e];
                }
                // END BIT STRING InfraEvents
                
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficConditionPresent) {
                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTrafficPresent =
                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTrafficPresent;
                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.losPresent =
                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.losPresent;
                if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTrafficPresent) {

                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTraffic.value =
                        wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.constructionAreaTraffic.value;
                }
                if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.losPresent) {
                    
                    // START BIT STRING InfraTrafficLOS
                    for(int f = 0; f < 4; f++) {
                        uint8_t tmp_f;
                        ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.los.values.push_back(tmp_f);
                        ros->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.los.values[f] = 
                            wind->oddinfo.odds.sectionObjects.elements[a].opConditions.trafficCondition.los.values[f];
                    }
                    // END BIT STRING InfraTrafficLOS
                    
                }
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.dspPresent) {

                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.dsp.value =
                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.dsp.value;
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.i2VPresent) {
                
                // START BIT STRING InfraI2V
                for(int g = 0; g < 5; g++) {
                    uint8_t tmp_g;
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.i2V.values.push_back(tmp_g);
                    ros->oddinfo.odds.sectionObjects.elements[a].opConditions.i2V.values[g] = 
                        wind->oddinfo.odds.sectionObjects.elements[a].opConditions.i2V.values[g];
                }
                // END BIT STRING InfraI2V
                
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehiclePresent) {

                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePresent.value =
                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePresent.value;

                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.latitude.value =
                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.latitude.value;

                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.longitude.value =
                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.longitude.value;

                ros->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.bearing.value =
                    wind->oddinfo.odds.sectionObjects.elements[a].opConditions.emergencyVehicle.emergencyVehiclePos.bearing.value;
            }
        }
        if(ros->oddinfo.odds.sectionObjects.elements[a].envConditionsPresent) {
            ros->oddinfo.odds.sectionObjects.elements[a].envConditions.weatherPresent =
                wind->oddinfo.odds.sectionObjects.elements[a].envConditions.weatherPresent;
            ros->oddinfo.odds.sectionObjects.elements[a].envConditions.lightingPresent =
                wind->oddinfo.odds.sectionObjects.elements[a].envConditions.lightingPresent;
            ros->oddinfo.odds.sectionObjects.elements[a].envConditions.obstaclePresent =
                wind->oddinfo.odds.sectionObjects.elements[a].envConditions.obstaclePresent;
            if(ros->oddinfo.odds.sectionObjects.elements[a].envConditions.weatherPresent) {
                
                // START BIT STRING WeatherCondition
                for(int h = 0; h < 6; h++) {
                    uint8_t tmp_h;
                    ros->oddinfo.odds.sectionObjects.elements[a].envConditions.weather.values.push_back(tmp_h);
                    ros->oddinfo.odds.sectionObjects.elements[a].envConditions.weather.values[h] = 
                        wind->oddinfo.odds.sectionObjects.elements[a].envConditions.weather.values[h];
                }
                // END BIT STRING WeatherCondition
                
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].envConditions.lightingPresent) {
                
                // START BIT STRING LightingCondition
                for(int i = 0; i < 3; i++) {
                    uint8_t tmp_i;
                    ros->oddinfo.odds.sectionObjects.elements[a].envConditions.lighting.values.push_back(tmp_i);
                    ros->oddinfo.odds.sectionObjects.elements[a].envConditions.lighting.values[i] = 
                        wind->oddinfo.odds.sectionObjects.elements[a].envConditions.lighting.values[i];
                }
                // END BIT STRING LightingCondition
                
            }
            if(ros->oddinfo.odds.sectionObjects.elements[a].envConditions.obstaclePresent) {
                
                // START BIT STRING ObstacleInformation
                for(int j = 0; j < 3; j++) {
                    uint8_t tmp_j;
                    ros->oddinfo.odds.sectionObjects.elements[a].envConditions.obstacle.values.push_back(tmp_j);
                    ros->oddinfo.odds.sectionObjects.elements[a].envConditions.obstacle.values[j] = 
                        wind->oddinfo.odds.sectionObjects.elements[a].envConditions.obstacle.values[j];
                }
                // END BIT STRING ObstacleInformation
                
            }
        }
    }
    // End Sequence of SectionObjectList
}
