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
 * This file belongs to the DLR Wind project de.dlr.ts.v2x:denm_v2:1.5
 * 
 * Module: DENM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
 *     wg1(1) en(302637) denm(1) version(2)}
 * 
 * For support contact v2x-ts@dlr.de
 * 
 *
 */
#include <denm_v2_denm_pdu_descriptions_rserial_decoder.h>

void wind::wind_ros::decode(denm_v2_denm_pdu_descriptions::DENM* ros, const char *buffer)
{

    // DENM  SEQUENCE
       //  header     ItsPduHeader                                    
       //  denm       DecentralizedEnvironmentalNotificationMessage   
    
    // Field name: header
        // ItsPduHeader  SEQUENCE
           //  protocolVersion ItsPduHeader_protocolVersion   
           //  messageID       ItsPduHeader_messageID         
           //  stationID       StationID                      
        
        // Field name: protocolVersion
        // Value
        wind::cpp::ITS_Container::ItsPduHeader_protocolVersion* _tmp_1 = (wind::cpp::ITS_Container::ItsPduHeader_protocolVersion*)buffer;
        buffer += sizeof(wind::cpp::ITS_Container::ItsPduHeader_protocolVersion);
        ros->header.protocolVersion.value = _tmp_1->value;
        
        // Field name: messageID
        // Value
        wind::cpp::ITS_Container::ItsPduHeader_messageID* _tmp_2 = (wind::cpp::ITS_Container::ItsPduHeader_messageID*)buffer;
        buffer += sizeof(wind::cpp::ITS_Container::ItsPduHeader_messageID);
        ros->header.messageID.value = _tmp_2->value;
        
        // Field name: stationID
        // Value
        wind::cpp::ITS_Container::StationID* _tmp_3 = (wind::cpp::ITS_Container::StationID*)buffer;
        buffer += sizeof(wind::cpp::ITS_Container::StationID);
        ros->header.stationID.value = _tmp_3->value;
    
    // Field name: denm
        // DecentralizedEnvironmentalNotificationMessage  SEQUENCE
           //  management ManagementContainer   
           //  situation  SituationContainer    OPTIONAL
           //  location   LocationContainer     OPTIONAL
           //  alacarte   AlacarteContainer     OPTIONAL
        ros->denm.situationPresent = *(buffer++);
        ros->denm.locationPresent = *(buffer++);
        ros->denm.alacartePresent = *(buffer++);
        
        // Field name: management
            // ManagementContainer  SEQUENCE
               //  actionID                  ActionID                    
               //  detectionTime             TimestampIts                
               //  referenceTime             TimestampIts                
               //  termination               Termination                 OPTIONAL
               //  eventPosition             ReferencePosition           
               //  relevanceDistance         RelevanceDistance           OPTIONAL
               //  relevanceTrafficDirection RelevanceTrafficDirection   OPTIONAL
               //  validityDuration          ValidityDuration            
               //  transmissionInterval      TransmissionInterval        OPTIONAL
               //  stationType               StationType                 
            ros->denm.management.terminationPresent = *(buffer++);
            ros->denm.management.relevanceDistancePresent = *(buffer++);
            ros->denm.management.relevanceTrafficDirectionPresent = *(buffer++);
            ros->denm.management.transmissionIntervalPresent = *(buffer++);
            
            // Field name: actionID
                // ActionID  SEQUENCE
                   //  originatingStationID StationID        
                   //  sequenceNumber       SequenceNumber   
                
                // Field name: originatingStationID
                // Value
                wind::cpp::ITS_Container::StationID* _tmp_4 = (wind::cpp::ITS_Container::StationID*)buffer;
                buffer += sizeof(wind::cpp::ITS_Container::StationID);
                ros->denm.management.actionID.originatingStationID.value = _tmp_4->value;
                
                // Field name: sequenceNumber
                // Value
                wind::cpp::ITS_Container::SequenceNumber* _tmp_5 = (wind::cpp::ITS_Container::SequenceNumber*)buffer;
                buffer += sizeof(wind::cpp::ITS_Container::SequenceNumber);
                ros->denm.management.actionID.sequenceNumber.value = _tmp_5->value;
            
            // Field name: detectionTime
            // Value
            wind::cpp::ITS_Container::TimestampIts* _tmp_6 = (wind::cpp::ITS_Container::TimestampIts*)buffer;
            buffer += sizeof(wind::cpp::ITS_Container::TimestampIts);
            ros->denm.management.detectionTime.value = _tmp_6->value;
            
            // Field name: referenceTime
            // Value
            wind::cpp::ITS_Container::TimestampIts* _tmp_7 = (wind::cpp::ITS_Container::TimestampIts*)buffer;
            buffer += sizeof(wind::cpp::ITS_Container::TimestampIts);
            ros->denm.management.referenceTime.value = _tmp_7->value;
            
            if(ros->denm.management.terminationPresent) {
                // Field name: termination
                // Value
                wind::cpp::DENM_PDU_Descriptions::Termination* _tmp_8 = (wind::cpp::DENM_PDU_Descriptions::Termination*)buffer;
                buffer += sizeof(wind::cpp::DENM_PDU_Descriptions::Termination);
                ros->denm.management.termination.value = _tmp_8->value;
            }
            
            // Field name: eventPosition
                // ReferencePosition  SEQUENCE
                   //  latitude                  Latitude               
                   //  longitude                 Longitude              
                   //  positionConfidenceEllipse PosConfidenceEllipse   
                   //  altitude                  Altitude               
                
                // Field name: latitude
                // Value
                wind::cpp::ITS_Container::Latitude* _tmp_9 = (wind::cpp::ITS_Container::Latitude*)buffer;
                buffer += sizeof(wind::cpp::ITS_Container::Latitude);
                ros->denm.management.eventPosition.latitude.value = _tmp_9->value;
                
                // Field name: longitude
                // Value
                wind::cpp::ITS_Container::Longitude* _tmp_10 = (wind::cpp::ITS_Container::Longitude*)buffer;
                buffer += sizeof(wind::cpp::ITS_Container::Longitude);
                ros->denm.management.eventPosition.longitude.value = _tmp_10->value;
                
                // Field name: positionConfidenceEllipse
                    // PosConfidenceEllipse  SEQUENCE
                       //  semiMajorConfidence  SemiAxisLength   
                       //  semiMinorConfidence  SemiAxisLength   
                       //  semiMajorOrientation HeadingValue     
                    
                    // Field name: semiMajorConfidence
                    // Value
                    wind::cpp::ITS_Container::SemiAxisLength* _tmp_11 = (wind::cpp::ITS_Container::SemiAxisLength*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::SemiAxisLength);
                    ros->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence.value = _tmp_11->value;
                    
                    // Field name: semiMinorConfidence
                    // Value
                    wind::cpp::ITS_Container::SemiAxisLength* _tmp_12 = (wind::cpp::ITS_Container::SemiAxisLength*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::SemiAxisLength);
                    ros->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence.value = _tmp_12->value;
                    
                    // Field name: semiMajorOrientation
                    // Value
                    wind::cpp::ITS_Container::HeadingValue* _tmp_13 = (wind::cpp::ITS_Container::HeadingValue*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::HeadingValue);
                    ros->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation.value = _tmp_13->value;
                
                // Field name: altitude
                    // Altitude  SEQUENCE
                       //  altitudeValue      AltitudeValue        
                       //  altitudeConfidence AltitudeConfidence   
                    
                    // Field name: altitudeValue
                    // Value
                    wind::cpp::ITS_Container::AltitudeValue* _tmp_14 = (wind::cpp::ITS_Container::AltitudeValue*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::AltitudeValue);
                    ros->denm.management.eventPosition.altitude.altitudeValue.value = _tmp_14->value;
                    
                    // Field name: altitudeConfidence
                    // Value
                    wind::cpp::ITS_Container::AltitudeConfidence* _tmp_15 = (wind::cpp::ITS_Container::AltitudeConfidence*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::AltitudeConfidence);
                    ros->denm.management.eventPosition.altitude.altitudeConfidence.value = _tmp_15->value;
            
            if(ros->denm.management.relevanceDistancePresent) {
                // Field name: relevanceDistance
                // Value
                wind::cpp::ITS_Container::RelevanceDistance* _tmp_16 = (wind::cpp::ITS_Container::RelevanceDistance*)buffer;
                buffer += sizeof(wind::cpp::ITS_Container::RelevanceDistance);
                ros->denm.management.relevanceDistance.value = _tmp_16->value;
            }
            
            if(ros->denm.management.relevanceTrafficDirectionPresent) {
                // Field name: relevanceTrafficDirection
                // Value
                wind::cpp::ITS_Container::RelevanceTrafficDirection* _tmp_17 = (wind::cpp::ITS_Container::RelevanceTrafficDirection*)buffer;
                buffer += sizeof(wind::cpp::ITS_Container::RelevanceTrafficDirection);
                ros->denm.management.relevanceTrafficDirection.value = _tmp_17->value;
            }
            
            // Field name: validityDuration
            // Value
            wind::cpp::ITS_Container::ValidityDuration* _tmp_18 = (wind::cpp::ITS_Container::ValidityDuration*)buffer;
            buffer += sizeof(wind::cpp::ITS_Container::ValidityDuration);
            ros->denm.management.validityDuration.value = _tmp_18->value;
            
            if(ros->denm.management.transmissionIntervalPresent) {
                // Field name: transmissionInterval
                // Value
                wind::cpp::ITS_Container::TransmissionInterval* _tmp_19 = (wind::cpp::ITS_Container::TransmissionInterval*)buffer;
                buffer += sizeof(wind::cpp::ITS_Container::TransmissionInterval);
                ros->denm.management.transmissionInterval.value = _tmp_19->value;
            }
            
            // Field name: stationType
            // Value
            wind::cpp::ITS_Container::StationType* _tmp_20 = (wind::cpp::ITS_Container::StationType*)buffer;
            buffer += sizeof(wind::cpp::ITS_Container::StationType);
            ros->denm.management.stationType.value = _tmp_20->value;
        
        if(ros->denm.situationPresent) {
            // Field name: situation
                // SituationContainer  SEQUENCE
                   //  informationQuality InformationQuality   
                   //  eventType          CauseCode            
                   //  linkedCause        CauseCode            OPTIONAL
                   //  eventHistory       EventHistory         OPTIONAL
                ros->denm.situation.linkedCausePresent = *(buffer++);
                ros->denm.situation.eventHistoryPresent = *(buffer++);
                
                // Field name: informationQuality
                // Value
                wind::cpp::ITS_Container::InformationQuality* _tmp_21 = (wind::cpp::ITS_Container::InformationQuality*)buffer;
                buffer += sizeof(wind::cpp::ITS_Container::InformationQuality);
                ros->denm.situation.informationQuality.value = _tmp_21->value;
                
                // Field name: eventType
                    // CauseCode  SEQUENCE
                       //  causeCode    CauseCodeType      
                       //  subCauseCode SubCauseCodeType   
                    
                    // Field name: causeCode
                    // Value
                    wind::cpp::ITS_Container::CauseCodeType* _tmp_22 = (wind::cpp::ITS_Container::CauseCodeType*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::CauseCodeType);
                    ros->denm.situation.eventType.causeCode.value = _tmp_22->value;
                    
                    // Field name: subCauseCode
                    // Value
                    wind::cpp::ITS_Container::SubCauseCodeType* _tmp_23 = (wind::cpp::ITS_Container::SubCauseCodeType*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::SubCauseCodeType);
                    ros->denm.situation.eventType.subCauseCode.value = _tmp_23->value;
                
                if(ros->denm.situation.linkedCausePresent) {
                    // Field name: linkedCause
                        // CauseCode  SEQUENCE
                           //  causeCode    CauseCodeType      
                           //  subCauseCode SubCauseCodeType   
                        
                        // Field name: causeCode
                        // Value
                        wind::cpp::ITS_Container::CauseCodeType* _tmp_24 = (wind::cpp::ITS_Container::CauseCodeType*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::CauseCodeType);
                        ros->denm.situation.linkedCause.causeCode.value = _tmp_24->value;
                        
                        // Field name: subCauseCode
                        // Value
                        wind::cpp::ITS_Container::SubCauseCodeType* _tmp_25 = (wind::cpp::ITS_Container::SubCauseCodeType*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::SubCauseCodeType);
                        ros->denm.situation.linkedCause.subCauseCode.value = _tmp_25->value;
                }
                
                if(ros->denm.situation.eventHistoryPresent) {
                    // Field name: eventHistory
                    // SequenceOf
                    // Data Type Byte
                    ros->denm.situation.eventHistory.count = *(buffer++);
                    
                    int count_a = ros->denm.situation.eventHistory.count;
                    for(int a = 0; a < count_a; a++) {
                        its_container_v2_its_container::EventPoint seqof_a;  // SEQUENCE
                        ros->denm.situation.eventHistory.elements.push_back(seqof_a);
                    
                            // EventPoint  SEQUENCE
                               //  eventPosition      DeltaReferencePosition   
                               //  eventDeltaTime     PathDeltaTime            OPTIONAL
                               //  informationQuality InformationQuality       
                            ros->denm.situation.eventHistory.elements[a].eventDeltaTimePresent = *(buffer++);
                            
                            // Field name: eventPosition
                                // DeltaReferencePosition  SEQUENCE
                                   //  deltaLatitude  DeltaLatitude    
                                   //  deltaLongitude DeltaLongitude   
                                   //  deltaAltitude  DeltaAltitude    
                                
                                // Field name: deltaLatitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLatitude* _tmp_26 = (wind::cpp::ITS_Container::DeltaLatitude*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLatitude);
                                ros->denm.situation.eventHistory.elements[a].eventPosition.deltaLatitude.value = _tmp_26->value;
                                
                                // Field name: deltaLongitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLongitude* _tmp_27 = (wind::cpp::ITS_Container::DeltaLongitude*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLongitude);
                                ros->denm.situation.eventHistory.elements[a].eventPosition.deltaLongitude.value = _tmp_27->value;
                                
                                // Field name: deltaAltitude
                                // Value
                                wind::cpp::ITS_Container::DeltaAltitude* _tmp_28 = (wind::cpp::ITS_Container::DeltaAltitude*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaAltitude);
                                ros->denm.situation.eventHistory.elements[a].eventPosition.deltaAltitude.value = _tmp_28->value;
                            
                            if(ros->denm.situation.eventHistory.elements[a].eventDeltaTimePresent) {
                                // Field name: eventDeltaTime
                                // Value
                                wind::cpp::ITS_Container::PathDeltaTime* _tmp_29 = (wind::cpp::ITS_Container::PathDeltaTime*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::PathDeltaTime);
                                ros->denm.situation.eventHistory.elements[a].eventDeltaTime.value = _tmp_29->value;
                            }
                            
                            // Field name: informationQuality
                            // Value
                            wind::cpp::ITS_Container::InformationQuality* _tmp_30 = (wind::cpp::ITS_Container::InformationQuality*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::InformationQuality);
                            ros->denm.situation.eventHistory.elements[a].informationQuality.value = _tmp_30->value;
                            
                    }
                }
        }
        
        if(ros->denm.locationPresent) {
            // Field name: location
                // LocationContainer  SEQUENCE
                   //  eventSpeed           Speed        OPTIONAL
                   //  eventPositionHeading Heading      OPTIONAL
                   //  traces               Traces       
                   //  roadType             RoadType     OPTIONAL
                ros->denm.location.eventSpeedPresent = *(buffer++);
                ros->denm.location.eventPositionHeadingPresent = *(buffer++);
                ros->denm.location.roadTypePresent = *(buffer++);
                
                if(ros->denm.location.eventSpeedPresent) {
                    // Field name: eventSpeed
                        // Speed  SEQUENCE
                           //  speedValue      SpeedValue        
                           //  speedConfidence SpeedConfidence   
                        
                        // Field name: speedValue
                        // Value
                        wind::cpp::ITS_Container::SpeedValue* _tmp_31 = (wind::cpp::ITS_Container::SpeedValue*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::SpeedValue);
                        ros->denm.location.eventSpeed.speedValue.value = _tmp_31->value;
                        
                        // Field name: speedConfidence
                        // Value
                        wind::cpp::ITS_Container::SpeedConfidence* _tmp_32 = (wind::cpp::ITS_Container::SpeedConfidence*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::SpeedConfidence);
                        ros->denm.location.eventSpeed.speedConfidence.value = _tmp_32->value;
                }
                
                if(ros->denm.location.eventPositionHeadingPresent) {
                    // Field name: eventPositionHeading
                        // Heading  SEQUENCE
                           //  headingValue      HeadingValue        
                           //  headingConfidence HeadingConfidence   
                        
                        // Field name: headingValue
                        // Value
                        wind::cpp::ITS_Container::HeadingValue* _tmp_33 = (wind::cpp::ITS_Container::HeadingValue*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::HeadingValue);
                        ros->denm.location.eventPositionHeading.headingValue.value = _tmp_33->value;
                        
                        // Field name: headingConfidence
                        // Value
                        wind::cpp::ITS_Container::HeadingConfidence* _tmp_34 = (wind::cpp::ITS_Container::HeadingConfidence*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::HeadingConfidence);
                        ros->denm.location.eventPositionHeading.headingConfidence.value = _tmp_34->value;
                }
                
                // Field name: traces
                // SequenceOf
                // Data Type Byte
                ros->denm.location.traces.count = *(buffer++);
                
                int count_b = ros->denm.location.traces.count;
                for(int b = 0; b < count_b; b++) {
                    its_container_v2_its_container::PathHistory seqof_b;  // SEQUENCE_OF
                    ros->denm.location.traces.elements.push_back(seqof_b);
                
                    // SequenceOf
                    // Data Type Byte
                    ros->denm.location.traces.elements[b].count = *(buffer++);
                    
                    int count_c = ros->denm.location.traces.elements[b].count;
                    for(int c = 0; c < count_c; c++) {
                        its_container_v2_its_container::PathPoint seqof_c;  // SEQUENCE
                        ros->denm.location.traces.elements[b].elements.push_back(seqof_c);
                    
                            // PathPoint  SEQUENCE
                               //  pathPosition  DeltaReferencePosition   
                               //  pathDeltaTime PathDeltaTime            OPTIONAL
                            ros->denm.location.traces.elements[b].elements[c].pathDeltaTimePresent = *(buffer++);
                            
                            // Field name: pathPosition
                                // DeltaReferencePosition  SEQUENCE
                                   //  deltaLatitude  DeltaLatitude    
                                   //  deltaLongitude DeltaLongitude   
                                   //  deltaAltitude  DeltaAltitude    
                                
                                // Field name: deltaLatitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLatitude* _tmp_35 = (wind::cpp::ITS_Container::DeltaLatitude*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLatitude);
                                ros->denm.location.traces.elements[b].elements[c].pathPosition.deltaLatitude.value = _tmp_35->value;
                                
                                // Field name: deltaLongitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLongitude* _tmp_36 = (wind::cpp::ITS_Container::DeltaLongitude*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLongitude);
                                ros->denm.location.traces.elements[b].elements[c].pathPosition.deltaLongitude.value = _tmp_36->value;
                                
                                // Field name: deltaAltitude
                                // Value
                                wind::cpp::ITS_Container::DeltaAltitude* _tmp_37 = (wind::cpp::ITS_Container::DeltaAltitude*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaAltitude);
                                ros->denm.location.traces.elements[b].elements[c].pathPosition.deltaAltitude.value = _tmp_37->value;
                            
                            if(ros->denm.location.traces.elements[b].elements[c].pathDeltaTimePresent) {
                                // Field name: pathDeltaTime
                                // Value
                                wind::cpp::ITS_Container::PathDeltaTime* _tmp_38 = (wind::cpp::ITS_Container::PathDeltaTime*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::PathDeltaTime);
                                ros->denm.location.traces.elements[b].elements[c].pathDeltaTime.value = _tmp_38->value;
                            }
                            
                    }
                    
                }
                
                if(ros->denm.location.roadTypePresent) {
                    // Field name: roadType
                    // Value
                    wind::cpp::ITS_Container::RoadType* _tmp_39 = (wind::cpp::ITS_Container::RoadType*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::RoadType);
                    ros->denm.location.roadType.value = _tmp_39->value;
                }
        }
        
        if(ros->denm.alacartePresent) {
            // Field name: alacarte
                // AlacarteContainer  SEQUENCE
                   //  lanePosition        LanePosition                 OPTIONAL
                   //  impactReduction     ImpactReductionContainer     OPTIONAL
                   //  externalTemperature Temperature                  OPTIONAL
                   //  roadWorks           RoadWorksContainerExtended   OPTIONAL
                   //  positioningSolution PositioningSolutionType      OPTIONAL
                   //  stationaryVehicle   StationaryVehicleContainer   OPTIONAL
                ros->denm.alacarte.lanePositionPresent = *(buffer++);
                ros->denm.alacarte.impactReductionPresent = *(buffer++);
                ros->denm.alacarte.externalTemperaturePresent = *(buffer++);
                ros->denm.alacarte.roadWorksPresent = *(buffer++);
                ros->denm.alacarte.positioningSolutionPresent = *(buffer++);
                ros->denm.alacarte.stationaryVehiclePresent = *(buffer++);
                
                if(ros->denm.alacarte.lanePositionPresent) {
                    // Field name: lanePosition
                    // Value
                    wind::cpp::ITS_Container::LanePosition* _tmp_40 = (wind::cpp::ITS_Container::LanePosition*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::LanePosition);
                    ros->denm.alacarte.lanePosition.value = _tmp_40->value;
                }
                
                if(ros->denm.alacarte.impactReductionPresent) {
                    // Field name: impactReduction
                        // ImpactReductionContainer  SEQUENCE
                           //  heightLonCarrLeft         HeightLonCarr               
                           //  heightLonCarrRight        HeightLonCarr               
                           //  posLonCarrLeft            PosLonCarr                  
                           //  posLonCarrRight           PosLonCarr                  
                           //  positionOfPillars         PositionOfPillars           
                           //  posCentMass               PosCentMass                 
                           //  wheelBaseVehicle          WheelBaseVehicle            
                           //  turningRadius             TurningRadius               
                           //  posFrontAx                PosFrontAx                  
                           //  positionOfOccupants       PositionOfOccupants         
                           //  vehicleMass               VehicleMass                 
                           //  requestResponseIndication RequestResponseIndication   
                        
                        // Field name: heightLonCarrLeft
                        // Value
                        wind::cpp::ITS_Container::HeightLonCarr* _tmp_41 = (wind::cpp::ITS_Container::HeightLonCarr*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::HeightLonCarr);
                        ros->denm.alacarte.impactReduction.heightLonCarrLeft.value = _tmp_41->value;
                        
                        // Field name: heightLonCarrRight
                        // Value
                        wind::cpp::ITS_Container::HeightLonCarr* _tmp_42 = (wind::cpp::ITS_Container::HeightLonCarr*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::HeightLonCarr);
                        ros->denm.alacarte.impactReduction.heightLonCarrRight.value = _tmp_42->value;
                        
                        // Field name: posLonCarrLeft
                        // Value
                        wind::cpp::ITS_Container::PosLonCarr* _tmp_43 = (wind::cpp::ITS_Container::PosLonCarr*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::PosLonCarr);
                        ros->denm.alacarte.impactReduction.posLonCarrLeft.value = _tmp_43->value;
                        
                        // Field name: posLonCarrRight
                        // Value
                        wind::cpp::ITS_Container::PosLonCarr* _tmp_44 = (wind::cpp::ITS_Container::PosLonCarr*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::PosLonCarr);
                        ros->denm.alacarte.impactReduction.posLonCarrRight.value = _tmp_44->value;
                        
                        // Field name: positionOfPillars
                        // SequenceOf
                        // Data Type Byte
                        ros->denm.alacarte.impactReduction.positionOfPillars.count = *(buffer++);
                        
                        int count_d = ros->denm.alacarte.impactReduction.positionOfPillars.count;
                        for(int d = 0; d < count_d; d++) {
                            its_container_v2_its_container::PosPillar seqof_d;  // REAL
                            ros->denm.alacarte.impactReduction.positionOfPillars.elements.push_back(seqof_d);
                        
                            // Value
                            wind::cpp::ITS_Container::PosPillar* _tmp_45 = (wind::cpp::ITS_Container::PosPillar*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::PosPillar);
                            ros->denm.alacarte.impactReduction.positionOfPillars.elements[d].value = _tmp_45->value;
                            
                        }
                        
                        // Field name: posCentMass
                        // Value
                        wind::cpp::ITS_Container::PosCentMass* _tmp_46 = (wind::cpp::ITS_Container::PosCentMass*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::PosCentMass);
                        ros->denm.alacarte.impactReduction.posCentMass.value = _tmp_46->value;
                        
                        // Field name: wheelBaseVehicle
                        // Value
                        wind::cpp::ITS_Container::WheelBaseVehicle* _tmp_47 = (wind::cpp::ITS_Container::WheelBaseVehicle*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::WheelBaseVehicle);
                        ros->denm.alacarte.impactReduction.wheelBaseVehicle.value = _tmp_47->value;
                        
                        // Field name: turningRadius
                        // Value
                        wind::cpp::ITS_Container::TurningRadius* _tmp_48 = (wind::cpp::ITS_Container::TurningRadius*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::TurningRadius);
                        ros->denm.alacarte.impactReduction.turningRadius.value = _tmp_48->value;
                        
                        // Field name: posFrontAx
                        // Value
                        wind::cpp::ITS_Container::PosFrontAx* _tmp_49 = (wind::cpp::ITS_Container::PosFrontAx*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::PosFrontAx);
                        ros->denm.alacarte.impactReduction.posFrontAx.value = _tmp_49->value;
                        
                        // Field name: positionOfOccupants
                        // BitString
                        wind::cpp::ITS_Container::PositionOfOccupants* _tmp_50 = (wind::cpp::ITS_Container::PositionOfOccupants*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::PositionOfOccupants);
                        for(int e = 0; e < 20; e++) {
                            uint8_t tmp_e;
                            ros->denm.alacarte.impactReduction.positionOfOccupants.values.push_back(tmp_e);
                            ros->denm.alacarte.impactReduction.positionOfOccupants.values[e] = _tmp_50->values[e];
                        }
                        
                        // Field name: vehicleMass
                        // Value
                        wind::cpp::ITS_Container::VehicleMass* _tmp_51 = (wind::cpp::ITS_Container::VehicleMass*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::VehicleMass);
                        ros->denm.alacarte.impactReduction.vehicleMass.value = _tmp_51->value;
                        
                        // Field name: requestResponseIndication
                        // Value
                        wind::cpp::ITS_Container::RequestResponseIndication* _tmp_52 = (wind::cpp::ITS_Container::RequestResponseIndication*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::RequestResponseIndication);
                        ros->denm.alacarte.impactReduction.requestResponseIndication.value = _tmp_52->value;
                }
                
                if(ros->denm.alacarte.externalTemperaturePresent) {
                    // Field name: externalTemperature
                    // Value
                    wind::cpp::ITS_Container::Temperature* _tmp_53 = (wind::cpp::ITS_Container::Temperature*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::Temperature);
                    ros->denm.alacarte.externalTemperature.value = _tmp_53->value;
                }
                
                if(ros->denm.alacarte.roadWorksPresent) {
                    // Field name: roadWorks
                        // RoadWorksContainerExtended  SEQUENCE
                           //  lightBarSirenInUse      LightBarSirenInUse       OPTIONAL
                           //  closedLanes             ClosedLanes              OPTIONAL
                           //  restriction             RestrictedTypes          OPTIONAL
                           //  speedLimit              SpeedLimit               OPTIONAL
                           //  incidentIndication      CauseCode                OPTIONAL
                           //  recommendedPath         ItineraryPath            OPTIONAL
                           //  startingPointSpeedLimit DeltaReferencePosition   OPTIONAL
                           //  trafficFlowRule         TrafficRule              OPTIONAL
                           //  referenceDenms          ReferenceDenms           OPTIONAL
                        ros->denm.alacarte.roadWorks.lightBarSirenInUsePresent = *(buffer++);
                        ros->denm.alacarte.roadWorks.closedLanesPresent = *(buffer++);
                        ros->denm.alacarte.roadWorks.restrictionPresent = *(buffer++);
                        ros->denm.alacarte.roadWorks.speedLimitPresent = *(buffer++);
                        ros->denm.alacarte.roadWorks.incidentIndicationPresent = *(buffer++);
                        ros->denm.alacarte.roadWorks.recommendedPathPresent = *(buffer++);
                        ros->denm.alacarte.roadWorks.startingPointSpeedLimitPresent = *(buffer++);
                        ros->denm.alacarte.roadWorks.trafficFlowRulePresent = *(buffer++);
                        ros->denm.alacarte.roadWorks.referenceDenmsPresent = *(buffer++);
                        
                        if(ros->denm.alacarte.roadWorks.lightBarSirenInUsePresent) {
                            // Field name: lightBarSirenInUse
                            // BitString
                            wind::cpp::ITS_Container::LightBarSirenInUse* _tmp_54 = (wind::cpp::ITS_Container::LightBarSirenInUse*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::LightBarSirenInUse);
                            for(int f = 0; f < 2; f++) {
                                uint8_t tmp_f;
                                ros->denm.alacarte.roadWorks.lightBarSirenInUse.values.push_back(tmp_f);
                                ros->denm.alacarte.roadWorks.lightBarSirenInUse.values[f] = _tmp_54->values[f];
                            }
                        }
                        
                        if(ros->denm.alacarte.roadWorks.closedLanesPresent) {
                            // Field name: closedLanes
                                // ClosedLanes  SEQUENCE
                                   //  innerhardShoulderStatus HardShoulderStatus   OPTIONAL
                                   //  outerhardShoulderStatus HardShoulderStatus   OPTIONAL
                                   //  drivingLaneStatus       DrivingLaneStatus    OPTIONAL
                                ros->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatusPresent = *(buffer++);
                                ros->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatusPresent = *(buffer++);
                                ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent = *(buffer++);
                                
                                if(ros->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatusPresent) {
                                    // Field name: innerhardShoulderStatus
                                    // Value
                                    wind::cpp::ITS_Container::HardShoulderStatus* _tmp_55 = (wind::cpp::ITS_Container::HardShoulderStatus*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::HardShoulderStatus);
                                    ros->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatus.value = _tmp_55->value;
                                }
                                
                                if(ros->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatusPresent) {
                                    // Field name: outerhardShoulderStatus
                                    // Value
                                    wind::cpp::ITS_Container::HardShoulderStatus* _tmp_56 = (wind::cpp::ITS_Container::HardShoulderStatus*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::HardShoulderStatus);
                                    ros->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatus.value = _tmp_56->value;
                                }
                                
                                if(ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent) {
                                    // Field name: drivingLaneStatus
                                    // BitString
                                    wind::cpp::ITS_Container::DrivingLaneStatus* _tmp_57 = (wind::cpp::ITS_Container::DrivingLaneStatus*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::DrivingLaneStatus);
                                    for(int g = 0; g < 13; g++) {
                                        uint8_t tmp_g;
                                        ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values.push_back(tmp_g);
                                        ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values[g] = _tmp_57->values[g];
                                    }
                                }
                        }
                        
                        if(ros->denm.alacarte.roadWorks.restrictionPresent) {
                            // Field name: restriction
                            // SequenceOf
                            // Data Type Byte
                            ros->denm.alacarte.roadWorks.restriction.count = *(buffer++);
                            
                            int count_h = ros->denm.alacarte.roadWorks.restriction.count;
                            for(int h = 0; h < count_h; h++) {
                                its_container_v2_its_container::StationType seqof_h;  // INTEGER
                                ros->denm.alacarte.roadWorks.restriction.elements.push_back(seqof_h);
                            
                                // Value
                                wind::cpp::ITS_Container::StationType* _tmp_58 = (wind::cpp::ITS_Container::StationType*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::StationType);
                                ros->denm.alacarte.roadWorks.restriction.elements[h].value = _tmp_58->value;
                                
                            }
                        }
                        
                        if(ros->denm.alacarte.roadWorks.speedLimitPresent) {
                            // Field name: speedLimit
                            // Value
                            wind::cpp::ITS_Container::SpeedLimit* _tmp_59 = (wind::cpp::ITS_Container::SpeedLimit*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::SpeedLimit);
                            ros->denm.alacarte.roadWorks.speedLimit.value = _tmp_59->value;
                        }
                        
                        if(ros->denm.alacarte.roadWorks.incidentIndicationPresent) {
                            // Field name: incidentIndication
                                // CauseCode  SEQUENCE
                                   //  causeCode    CauseCodeType      
                                   //  subCauseCode SubCauseCodeType   
                                
                                // Field name: causeCode
                                // Value
                                wind::cpp::ITS_Container::CauseCodeType* _tmp_60 = (wind::cpp::ITS_Container::CauseCodeType*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::CauseCodeType);
                                ros->denm.alacarte.roadWorks.incidentIndication.causeCode.value = _tmp_60->value;
                                
                                // Field name: subCauseCode
                                // Value
                                wind::cpp::ITS_Container::SubCauseCodeType* _tmp_61 = (wind::cpp::ITS_Container::SubCauseCodeType*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::SubCauseCodeType);
                                ros->denm.alacarte.roadWorks.incidentIndication.subCauseCode.value = _tmp_61->value;
                        }
                        
                        if(ros->denm.alacarte.roadWorks.recommendedPathPresent) {
                            // Field name: recommendedPath
                            // SequenceOf
                            // Data Type Byte
                            ros->denm.alacarte.roadWorks.recommendedPath.count = *(buffer++);
                            
                            int count_i = ros->denm.alacarte.roadWorks.recommendedPath.count;
                            for(int i = 0; i < count_i; i++) {
                                its_container_v2_its_container::ReferencePosition seqof_i;  // SEQUENCE
                                ros->denm.alacarte.roadWorks.recommendedPath.elements.push_back(seqof_i);
                            
                                    // ReferencePosition  SEQUENCE
                                       //  latitude                  Latitude               
                                       //  longitude                 Longitude              
                                       //  positionConfidenceEllipse PosConfidenceEllipse   
                                       //  altitude                  Altitude               
                                    
                                    // Field name: latitude
                                    // Value
                                    wind::cpp::ITS_Container::Latitude* _tmp_62 = (wind::cpp::ITS_Container::Latitude*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::Latitude);
                                    ros->denm.alacarte.roadWorks.recommendedPath.elements[i].latitude.value = _tmp_62->value;
                                    
                                    // Field name: longitude
                                    // Value
                                    wind::cpp::ITS_Container::Longitude* _tmp_63 = (wind::cpp::ITS_Container::Longitude*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::Longitude);
                                    ros->denm.alacarte.roadWorks.recommendedPath.elements[i].longitude.value = _tmp_63->value;
                                    
                                    // Field name: positionConfidenceEllipse
                                        // PosConfidenceEllipse  SEQUENCE
                                           //  semiMajorConfidence  SemiAxisLength   
                                           //  semiMinorConfidence  SemiAxisLength   
                                           //  semiMajorOrientation HeadingValue     
                                        
                                        // Field name: semiMajorConfidence
                                        // Value
                                        wind::cpp::ITS_Container::SemiAxisLength* _tmp_64 = (wind::cpp::ITS_Container::SemiAxisLength*)buffer;
                                        buffer += sizeof(wind::cpp::ITS_Container::SemiAxisLength);
                                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMajorConfidence.value = _tmp_64->value;
                                        
                                        // Field name: semiMinorConfidence
                                        // Value
                                        wind::cpp::ITS_Container::SemiAxisLength* _tmp_65 = (wind::cpp::ITS_Container::SemiAxisLength*)buffer;
                                        buffer += sizeof(wind::cpp::ITS_Container::SemiAxisLength);
                                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMinorConfidence.value = _tmp_65->value;
                                        
                                        // Field name: semiMajorOrientation
                                        // Value
                                        wind::cpp::ITS_Container::HeadingValue* _tmp_66 = (wind::cpp::ITS_Container::HeadingValue*)buffer;
                                        buffer += sizeof(wind::cpp::ITS_Container::HeadingValue);
                                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMajorOrientation.value = _tmp_66->value;
                                    
                                    // Field name: altitude
                                        // Altitude  SEQUENCE
                                           //  altitudeValue      AltitudeValue        
                                           //  altitudeConfidence AltitudeConfidence   
                                        
                                        // Field name: altitudeValue
                                        // Value
                                        wind::cpp::ITS_Container::AltitudeValue* _tmp_67 = (wind::cpp::ITS_Container::AltitudeValue*)buffer;
                                        buffer += sizeof(wind::cpp::ITS_Container::AltitudeValue);
                                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].altitude.altitudeValue.value = _tmp_67->value;
                                        
                                        // Field name: altitudeConfidence
                                        // Value
                                        wind::cpp::ITS_Container::AltitudeConfidence* _tmp_68 = (wind::cpp::ITS_Container::AltitudeConfidence*)buffer;
                                        buffer += sizeof(wind::cpp::ITS_Container::AltitudeConfidence);
                                        ros->denm.alacarte.roadWorks.recommendedPath.elements[i].altitude.altitudeConfidence.value = _tmp_68->value;
                                    
                            }
                        }
                        
                        if(ros->denm.alacarte.roadWorks.startingPointSpeedLimitPresent) {
                            // Field name: startingPointSpeedLimit
                                // DeltaReferencePosition  SEQUENCE
                                   //  deltaLatitude  DeltaLatitude    
                                   //  deltaLongitude DeltaLongitude   
                                   //  deltaAltitude  DeltaAltitude    
                                
                                // Field name: deltaLatitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLatitude* _tmp_69 = (wind::cpp::ITS_Container::DeltaLatitude*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLatitude);
                                ros->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaLatitude.value = _tmp_69->value;
                                
                                // Field name: deltaLongitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLongitude* _tmp_70 = (wind::cpp::ITS_Container::DeltaLongitude*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLongitude);
                                ros->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaLongitude.value = _tmp_70->value;
                                
                                // Field name: deltaAltitude
                                // Value
                                wind::cpp::ITS_Container::DeltaAltitude* _tmp_71 = (wind::cpp::ITS_Container::DeltaAltitude*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaAltitude);
                                ros->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaAltitude.value = _tmp_71->value;
                        }
                        
                        if(ros->denm.alacarte.roadWorks.trafficFlowRulePresent) {
                            // Field name: trafficFlowRule
                            // Value
                            wind::cpp::ITS_Container::TrafficRule* _tmp_72 = (wind::cpp::ITS_Container::TrafficRule*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::TrafficRule);
                            ros->denm.alacarte.roadWorks.trafficFlowRule.value = _tmp_72->value;
                        }
                        
                        if(ros->denm.alacarte.roadWorks.referenceDenmsPresent) {
                            // Field name: referenceDenms
                            // SequenceOf
                            // Data Type Byte
                            ros->denm.alacarte.roadWorks.referenceDenms.count = *(buffer++);
                            
                            int count_j = ros->denm.alacarte.roadWorks.referenceDenms.count;
                            for(int j = 0; j < count_j; j++) {
                                its_container_v2_its_container::ActionID seqof_j;  // SEQUENCE
                                ros->denm.alacarte.roadWorks.referenceDenms.elements.push_back(seqof_j);
                            
                                    // ActionID  SEQUENCE
                                       //  originatingStationID StationID        
                                       //  sequenceNumber       SequenceNumber   
                                    
                                    // Field name: originatingStationID
                                    // Value
                                    wind::cpp::ITS_Container::StationID* _tmp_73 = (wind::cpp::ITS_Container::StationID*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::StationID);
                                    ros->denm.alacarte.roadWorks.referenceDenms.elements[j].originatingStationID.value = _tmp_73->value;
                                    
                                    // Field name: sequenceNumber
                                    // Value
                                    wind::cpp::ITS_Container::SequenceNumber* _tmp_74 = (wind::cpp::ITS_Container::SequenceNumber*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::SequenceNumber);
                                    ros->denm.alacarte.roadWorks.referenceDenms.elements[j].sequenceNumber.value = _tmp_74->value;
                                    
                            }
                        }
                }
                
                if(ros->denm.alacarte.positioningSolutionPresent) {
                    // Field name: positioningSolution
                    // Value
                    wind::cpp::ITS_Container::PositioningSolutionType* _tmp_75 = (wind::cpp::ITS_Container::PositioningSolutionType*)buffer;
                    buffer += sizeof(wind::cpp::ITS_Container::PositioningSolutionType);
                    ros->denm.alacarte.positioningSolution.value = _tmp_75->value;
                }
                
                if(ros->denm.alacarte.stationaryVehiclePresent) {
                    // Field name: stationaryVehicle
                        // StationaryVehicleContainer  SEQUENCE
                           //  stationarySince        StationarySince          OPTIONAL
                           //  stationaryCause        CauseCode                OPTIONAL
                           //  carryingDangerousGoods DangerousGoodsExtended   OPTIONAL
                           //  numberOfOccupants      NumberOfOccupants        OPTIONAL
                           //  vehicleIdentification  VehicleIdentification    OPTIONAL
                           //  energyStorageType      EnergyStorageType        OPTIONAL
                        ros->denm.alacarte.stationaryVehicle.stationarySincePresent = *(buffer++);
                        ros->denm.alacarte.stationaryVehicle.stationaryCausePresent = *(buffer++);
                        ros->denm.alacarte.stationaryVehicle.carryingDangerousGoodsPresent = *(buffer++);
                        ros->denm.alacarte.stationaryVehicle.numberOfOccupantsPresent = *(buffer++);
                        ros->denm.alacarte.stationaryVehicle.vehicleIdentificationPresent = *(buffer++);
                        ros->denm.alacarte.stationaryVehicle.energyStorageTypePresent = *(buffer++);
                        
                        if(ros->denm.alacarte.stationaryVehicle.stationarySincePresent) {
                            // Field name: stationarySince
                            // Value
                            wind::cpp::ITS_Container::StationarySince* _tmp_76 = (wind::cpp::ITS_Container::StationarySince*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::StationarySince);
                            ros->denm.alacarte.stationaryVehicle.stationarySince.value = _tmp_76->value;
                        }
                        
                        if(ros->denm.alacarte.stationaryVehicle.stationaryCausePresent) {
                            // Field name: stationaryCause
                                // CauseCode  SEQUENCE
                                   //  causeCode    CauseCodeType      
                                   //  subCauseCode SubCauseCodeType   
                                
                                // Field name: causeCode
                                // Value
                                wind::cpp::ITS_Container::CauseCodeType* _tmp_77 = (wind::cpp::ITS_Container::CauseCodeType*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::CauseCodeType);
                                ros->denm.alacarte.stationaryVehicle.stationaryCause.causeCode.value = _tmp_77->value;
                                
                                // Field name: subCauseCode
                                // Value
                                wind::cpp::ITS_Container::SubCauseCodeType* _tmp_78 = (wind::cpp::ITS_Container::SubCauseCodeType*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::SubCauseCodeType);
                                ros->denm.alacarte.stationaryVehicle.stationaryCause.subCauseCode.value = _tmp_78->value;
                        }
                        
                        if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoodsPresent) {
                            // Field name: carryingDangerousGoods
                                // DangerousGoodsExtended  SEQUENCE
                                   //  dangerousGoodsType  DangerousGoodsBasic                          
                                   //  unNumber            DangerousGoodsExtended_unNumber              
                                   //  elevatedTemperature DangerousGoodsExtended_elevatedTemperature   
                                   //  tunnelsRestricted   DangerousGoodsExtended_tunnelsRestricted     
                                   //  limitedQuantity     DangerousGoodsExtended_limitedQuantity       
                                   //  emergencyActionCode DangerousGoodsExtended_emergencyActionCode   OPTIONAL
                                   //  phoneNumber         PhoneNumber                                  OPTIONAL
                                   //  companyName         DangerousGoodsExtended_companyName           OPTIONAL
                                ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCodePresent = *(buffer++);
                                ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumberPresent = *(buffer++);
                                ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyNamePresent = *(buffer++);
                                
                                // Field name: dangerousGoodsType
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsBasic* _tmp_79 = (wind::cpp::ITS_Container::DangerousGoodsBasic*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsBasic);
                                ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.dangerousGoodsType.value = _tmp_79->value;
                                
                                // Field name: unNumber
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsExtended_unNumber* _tmp_80 = (wind::cpp::ITS_Container::DangerousGoodsExtended_unNumber*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_unNumber);
                                ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.unNumber.value = _tmp_80->value;
                                
                                // Field name: elevatedTemperature
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsExtended_elevatedTemperature* _tmp_81 = (wind::cpp::ITS_Container::DangerousGoodsExtended_elevatedTemperature*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_elevatedTemperature);
                                ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.elevatedTemperature.value = _tmp_81->value;
                                
                                // Field name: tunnelsRestricted
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsExtended_tunnelsRestricted* _tmp_82 = (wind::cpp::ITS_Container::DangerousGoodsExtended_tunnelsRestricted*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_tunnelsRestricted);
                                ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.tunnelsRestricted.value = _tmp_82->value;
                                
                                // Field name: limitedQuantity
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsExtended_limitedQuantity* _tmp_83 = (wind::cpp::ITS_Container::DangerousGoodsExtended_limitedQuantity*)buffer;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_limitedQuantity);
                                ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.limitedQuantity.value = _tmp_83->value;
                                
                                if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCodePresent) {
                                    // Field name: emergencyActionCode
                                    // Text
                                    
                                    wind::cpp::ITS_Container::DangerousGoodsExtended_emergencyActionCode* _tmp_84 = (wind::cpp::ITS_Container::DangerousGoodsExtended_emergencyActionCode*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_emergencyActionCode);
                                    
                                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCode.value.assign(_tmp_84->value);
                                }
                                
                                if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumberPresent) {
                                    // Field name: phoneNumber
                                    // Text
                                    
                                    wind::cpp::ITS_Container::PhoneNumber* _tmp_85 = (wind::cpp::ITS_Container::PhoneNumber*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::PhoneNumber);
                                    
                                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumber.value.assign(_tmp_85->value);
                                }
                                
                                if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyNamePresent) {
                                    // Field name: companyName
                                    // Text
                                    
                                    wind::cpp::ITS_Container::DangerousGoodsExtended_companyName* _tmp_86 = (wind::cpp::ITS_Container::DangerousGoodsExtended_companyName*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_companyName);
                                    
                                    ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyName.value.assign(_tmp_86->value);
                                }
                        }
                        
                        if(ros->denm.alacarte.stationaryVehicle.numberOfOccupantsPresent) {
                            // Field name: numberOfOccupants
                            // Value
                            wind::cpp::ITS_Container::NumberOfOccupants* _tmp_87 = (wind::cpp::ITS_Container::NumberOfOccupants*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::NumberOfOccupants);
                            ros->denm.alacarte.stationaryVehicle.numberOfOccupants.value = _tmp_87->value;
                        }
                        
                        if(ros->denm.alacarte.stationaryVehicle.vehicleIdentificationPresent) {
                            // Field name: vehicleIdentification
                                // VehicleIdentification  SEQUENCE
                                   //  wMInumber  WMInumber    OPTIONAL
                                   //  vDS        VDS          OPTIONAL
                                ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumberPresent = *(buffer++);
                                ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDSPresent = *(buffer++);
                                
                                if(ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumberPresent) {
                                    // Field name: wMInumber
                                    // Text
                                    
                                    wind::cpp::ITS_Container::WMInumber* _tmp_88 = (wind::cpp::ITS_Container::WMInumber*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::WMInumber);
                                    
                                    ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumber.value.assign(_tmp_88->value);
                                }
                                
                                if(ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDSPresent) {
                                    // Field name: vDS
                                    // Text
                                    
                                    wind::cpp::ITS_Container::VDS* _tmp_89 = (wind::cpp::ITS_Container::VDS*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::VDS);
                                    
                                    ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDS.value.assign(_tmp_89->value);
                                }
                        }
                        
                        if(ros->denm.alacarte.stationaryVehicle.energyStorageTypePresent) {
                            // Field name: energyStorageType
                            // BitString
                            wind::cpp::ITS_Container::EnergyStorageType* _tmp_90 = (wind::cpp::ITS_Container::EnergyStorageType*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::EnergyStorageType);
                            for(int p = 0; p < 7; p++) {
                                uint8_t tmp_p;
                                ros->denm.alacarte.stationaryVehicle.energyStorageType.values.push_back(tmp_p);
                                ros->denm.alacarte.stationaryVehicle.energyStorageType.values[p] = _tmp_90->values[p];
                            }
                        }
                }
        }
    
}
