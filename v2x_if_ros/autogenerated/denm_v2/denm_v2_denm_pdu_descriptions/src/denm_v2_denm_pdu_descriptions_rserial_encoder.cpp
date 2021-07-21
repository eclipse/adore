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
#include <denm_v2_denm_pdu_descriptions_rserial_encoder.h>

int wind::wind_ros::encode(const denm_v2_denm_pdu_descriptions::DENM::ConstPtr& ros, const char *buffer)
{
    const char *start = buffer;

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
        wind::cpp::ITS_Container::ItsPduHeader_protocolVersion* _tmp_1 =
            (wind::cpp::ITS_Container::ItsPduHeader_protocolVersion*)buffer;
        _tmp_1->value = ros->header.protocolVersion.value;
        buffer += sizeof(wind::cpp::ITS_Container::ItsPduHeader_protocolVersion);
        
        // Field name: messageID
        // Value
        wind::cpp::ITS_Container::ItsPduHeader_messageID* _tmp_2 =
            (wind::cpp::ITS_Container::ItsPduHeader_messageID*)buffer;
        _tmp_2->value = ros->header.messageID.value;
        buffer += sizeof(wind::cpp::ITS_Container::ItsPduHeader_messageID);
        
        // Field name: stationID
        // Value
        wind::cpp::ITS_Container::StationID* _tmp_3 =
            (wind::cpp::ITS_Container::StationID*)buffer;
        _tmp_3->value = ros->header.stationID.value;
        buffer += sizeof(wind::cpp::ITS_Container::StationID);
    
    // Field name: denm
        // DecentralizedEnvironmentalNotificationMessage  SEQUENCE
           //  management ManagementContainer   
           //  situation  SituationContainer    OPTIONAL
           //  location   LocationContainer     OPTIONAL
           //  alacarte   AlacarteContainer     OPTIONAL
        char* _tmp_4 = (char*)buffer;
        *_tmp_4 = (ros->denm.situationPresent? 1: 0);
        buffer++;
        
        char* _tmp_5 = (char*)buffer;
        *_tmp_5 = (ros->denm.locationPresent? 1: 0);
        buffer++;
        
        char* _tmp_6 = (char*)buffer;
        *_tmp_6 = (ros->denm.alacartePresent? 1: 0);
        buffer++;
        
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
            char* _tmp_7 = (char*)buffer;
            *_tmp_7 = (ros->denm.management.terminationPresent? 1: 0);
            buffer++;
            
            char* _tmp_8 = (char*)buffer;
            *_tmp_8 = (ros->denm.management.relevanceDistancePresent? 1: 0);
            buffer++;
            
            char* _tmp_9 = (char*)buffer;
            *_tmp_9 = (ros->denm.management.relevanceTrafficDirectionPresent? 1: 0);
            buffer++;
            
            char* _tmp_10 = (char*)buffer;
            *_tmp_10 = (ros->denm.management.transmissionIntervalPresent? 1: 0);
            buffer++;
            
            // Field name: actionID
                // ActionID  SEQUENCE
                   //  originatingStationID StationID        
                   //  sequenceNumber       SequenceNumber   
                // Field name: originatingStationID
                // Value
                wind::cpp::ITS_Container::StationID* _tmp_11 =
                    (wind::cpp::ITS_Container::StationID*)buffer;
                _tmp_11->value = ros->denm.management.actionID.originatingStationID.value;
                buffer += sizeof(wind::cpp::ITS_Container::StationID);
                
                // Field name: sequenceNumber
                // Value
                wind::cpp::ITS_Container::SequenceNumber* _tmp_12 =
                    (wind::cpp::ITS_Container::SequenceNumber*)buffer;
                _tmp_12->value = ros->denm.management.actionID.sequenceNumber.value;
                buffer += sizeof(wind::cpp::ITS_Container::SequenceNumber);
            
            // Field name: detectionTime
            // Value
            wind::cpp::ITS_Container::TimestampIts* _tmp_13 =
                (wind::cpp::ITS_Container::TimestampIts*)buffer;
            _tmp_13->value = ros->denm.management.detectionTime.value;
            buffer += sizeof(wind::cpp::ITS_Container::TimestampIts);
            
            // Field name: referenceTime
            // Value
            wind::cpp::ITS_Container::TimestampIts* _tmp_14 =
                (wind::cpp::ITS_Container::TimestampIts*)buffer;
            _tmp_14->value = ros->denm.management.referenceTime.value;
            buffer += sizeof(wind::cpp::ITS_Container::TimestampIts);
            
            if(ros->denm.management.terminationPresent) {
                // Field name: termination
                // Value
                wind::cpp::DENM_PDU_Descriptions::Termination* _tmp_15 =
                    (wind::cpp::DENM_PDU_Descriptions::Termination*)buffer;
                _tmp_15->value = ros->denm.management.termination.value;
                buffer += sizeof(wind::cpp::DENM_PDU_Descriptions::Termination);
            }
            
            // Field name: eventPosition
                // ReferencePosition  SEQUENCE
                   //  latitude                  Latitude               
                   //  longitude                 Longitude              
                   //  positionConfidenceEllipse PosConfidenceEllipse   
                   //  altitude                  Altitude               
                // Field name: latitude
                // Value
                wind::cpp::ITS_Container::Latitude* _tmp_16 =
                    (wind::cpp::ITS_Container::Latitude*)buffer;
                _tmp_16->value = ros->denm.management.eventPosition.latitude.value;
                buffer += sizeof(wind::cpp::ITS_Container::Latitude);
                
                // Field name: longitude
                // Value
                wind::cpp::ITS_Container::Longitude* _tmp_17 =
                    (wind::cpp::ITS_Container::Longitude*)buffer;
                _tmp_17->value = ros->denm.management.eventPosition.longitude.value;
                buffer += sizeof(wind::cpp::ITS_Container::Longitude);
                
                // Field name: positionConfidenceEllipse
                    // PosConfidenceEllipse  SEQUENCE
                       //  semiMajorConfidence  SemiAxisLength   
                       //  semiMinorConfidence  SemiAxisLength   
                       //  semiMajorOrientation HeadingValue     
                    // Field name: semiMajorConfidence
                    // Value
                    wind::cpp::ITS_Container::SemiAxisLength* _tmp_18 =
                        (wind::cpp::ITS_Container::SemiAxisLength*)buffer;
                    _tmp_18->value = ros->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence.value;
                    buffer += sizeof(wind::cpp::ITS_Container::SemiAxisLength);
                    
                    // Field name: semiMinorConfidence
                    // Value
                    wind::cpp::ITS_Container::SemiAxisLength* _tmp_19 =
                        (wind::cpp::ITS_Container::SemiAxisLength*)buffer;
                    _tmp_19->value = ros->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence.value;
                    buffer += sizeof(wind::cpp::ITS_Container::SemiAxisLength);
                    
                    // Field name: semiMajorOrientation
                    // Value
                    wind::cpp::ITS_Container::HeadingValue* _tmp_20 =
                        (wind::cpp::ITS_Container::HeadingValue*)buffer;
                    _tmp_20->value = ros->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation.value;
                    buffer += sizeof(wind::cpp::ITS_Container::HeadingValue);
                
                // Field name: altitude
                    // Altitude  SEQUENCE
                       //  altitudeValue      AltitudeValue        
                       //  altitudeConfidence AltitudeConfidence   
                    // Field name: altitudeValue
                    // Value
                    wind::cpp::ITS_Container::AltitudeValue* _tmp_21 =
                        (wind::cpp::ITS_Container::AltitudeValue*)buffer;
                    _tmp_21->value = ros->denm.management.eventPosition.altitude.altitudeValue.value;
                    buffer += sizeof(wind::cpp::ITS_Container::AltitudeValue);
                    
                    // Field name: altitudeConfidence
                    // Value
                    wind::cpp::ITS_Container::AltitudeConfidence* _tmp_22 =
                        (wind::cpp::ITS_Container::AltitudeConfidence*)buffer;
                    _tmp_22->value = ros->denm.management.eventPosition.altitude.altitudeConfidence.value;
                    buffer += sizeof(wind::cpp::ITS_Container::AltitudeConfidence);
            
            if(ros->denm.management.relevanceDistancePresent) {
                // Field name: relevanceDistance
                // Value
                wind::cpp::ITS_Container::RelevanceDistance* _tmp_23 =
                    (wind::cpp::ITS_Container::RelevanceDistance*)buffer;
                _tmp_23->value = ros->denm.management.relevanceDistance.value;
                buffer += sizeof(wind::cpp::ITS_Container::RelevanceDistance);
            }
            
            if(ros->denm.management.relevanceTrafficDirectionPresent) {
                // Field name: relevanceTrafficDirection
                // Value
                wind::cpp::ITS_Container::RelevanceTrafficDirection* _tmp_24 =
                    (wind::cpp::ITS_Container::RelevanceTrafficDirection*)buffer;
                _tmp_24->value = ros->denm.management.relevanceTrafficDirection.value;
                buffer += sizeof(wind::cpp::ITS_Container::RelevanceTrafficDirection);
            }
            
            // Field name: validityDuration
            // Value
            wind::cpp::ITS_Container::ValidityDuration* _tmp_25 =
                (wind::cpp::ITS_Container::ValidityDuration*)buffer;
            _tmp_25->value = ros->denm.management.validityDuration.value;
            buffer += sizeof(wind::cpp::ITS_Container::ValidityDuration);
            
            if(ros->denm.management.transmissionIntervalPresent) {
                // Field name: transmissionInterval
                // Value
                wind::cpp::ITS_Container::TransmissionInterval* _tmp_26 =
                    (wind::cpp::ITS_Container::TransmissionInterval*)buffer;
                _tmp_26->value = ros->denm.management.transmissionInterval.value;
                buffer += sizeof(wind::cpp::ITS_Container::TransmissionInterval);
            }
            
            // Field name: stationType
            // Value
            wind::cpp::ITS_Container::StationType* _tmp_27 =
                (wind::cpp::ITS_Container::StationType*)buffer;
            _tmp_27->value = ros->denm.management.stationType.value;
            buffer += sizeof(wind::cpp::ITS_Container::StationType);
        
        if(ros->denm.situationPresent) {
            // Field name: situation
                // SituationContainer  SEQUENCE
                   //  informationQuality InformationQuality   
                   //  eventType          CauseCode            
                   //  linkedCause        CauseCode            OPTIONAL
                   //  eventHistory       EventHistory         OPTIONAL
                char* _tmp_28 = (char*)buffer;
                *_tmp_28 = (ros->denm.situation.linkedCausePresent? 1: 0);
                buffer++;
                
                char* _tmp_29 = (char*)buffer;
                *_tmp_29 = (ros->denm.situation.eventHistoryPresent? 1: 0);
                buffer++;
                
                // Field name: informationQuality
                // Value
                wind::cpp::ITS_Container::InformationQuality* _tmp_30 =
                    (wind::cpp::ITS_Container::InformationQuality*)buffer;
                _tmp_30->value = ros->denm.situation.informationQuality.value;
                buffer += sizeof(wind::cpp::ITS_Container::InformationQuality);
                
                // Field name: eventType
                    // CauseCode  SEQUENCE
                       //  causeCode    CauseCodeType      
                       //  subCauseCode SubCauseCodeType   
                    // Field name: causeCode
                    // Value
                    wind::cpp::ITS_Container::CauseCodeType* _tmp_31 =
                        (wind::cpp::ITS_Container::CauseCodeType*)buffer;
                    _tmp_31->value = ros->denm.situation.eventType.causeCode.value;
                    buffer += sizeof(wind::cpp::ITS_Container::CauseCodeType);
                    
                    // Field name: subCauseCode
                    // Value
                    wind::cpp::ITS_Container::SubCauseCodeType* _tmp_32 =
                        (wind::cpp::ITS_Container::SubCauseCodeType*)buffer;
                    _tmp_32->value = ros->denm.situation.eventType.subCauseCode.value;
                    buffer += sizeof(wind::cpp::ITS_Container::SubCauseCodeType);
                
                if(ros->denm.situation.linkedCausePresent) {
                    // Field name: linkedCause
                        // CauseCode  SEQUENCE
                           //  causeCode    CauseCodeType      
                           //  subCauseCode SubCauseCodeType   
                        // Field name: causeCode
                        // Value
                        wind::cpp::ITS_Container::CauseCodeType* _tmp_33 =
                            (wind::cpp::ITS_Container::CauseCodeType*)buffer;
                        _tmp_33->value = ros->denm.situation.linkedCause.causeCode.value;
                        buffer += sizeof(wind::cpp::ITS_Container::CauseCodeType);
                        
                        // Field name: subCauseCode
                        // Value
                        wind::cpp::ITS_Container::SubCauseCodeType* _tmp_34 =
                            (wind::cpp::ITS_Container::SubCauseCodeType*)buffer;
                        _tmp_34->value = ros->denm.situation.linkedCause.subCauseCode.value;
                        buffer += sizeof(wind::cpp::ITS_Container::SubCauseCodeType);
                }
                
                if(ros->denm.situation.eventHistoryPresent) {
                    // Field name: eventHistory
                    // SequenceOf
                    // Data Type Byte
                    char* _tmp_35 = (char*)buffer;
                    *_tmp_35 = ros->denm.situation.eventHistory.count;
                    buffer++;
                    
                    int count_a = ros->denm.situation.eventHistory.count;
                    for(int a = 0; a < count_a; a++) {
                        
                            // EventPoint  SEQUENCE
                               //  eventPosition      DeltaReferencePosition   
                               //  eventDeltaTime     PathDeltaTime            OPTIONAL
                               //  informationQuality InformationQuality       
                            char* _tmp_36 = (char*)buffer;
                            *_tmp_36 = (ros->denm.situation.eventHistory.elements[a].eventDeltaTimePresent? 1: 0);
                            buffer++;
                            
                            // Field name: eventPosition
                                // DeltaReferencePosition  SEQUENCE
                                   //  deltaLatitude  DeltaLatitude    
                                   //  deltaLongitude DeltaLongitude   
                                   //  deltaAltitude  DeltaAltitude    
                                // Field name: deltaLatitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLatitude* _tmp_37 =
                                    (wind::cpp::ITS_Container::DeltaLatitude*)buffer;
                                _tmp_37->value = ros->denm.situation.eventHistory.elements[a].eventPosition.deltaLatitude.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLatitude);
                                
                                // Field name: deltaLongitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLongitude* _tmp_38 =
                                    (wind::cpp::ITS_Container::DeltaLongitude*)buffer;
                                _tmp_38->value = ros->denm.situation.eventHistory.elements[a].eventPosition.deltaLongitude.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLongitude);
                                
                                // Field name: deltaAltitude
                                // Value
                                wind::cpp::ITS_Container::DeltaAltitude* _tmp_39 =
                                    (wind::cpp::ITS_Container::DeltaAltitude*)buffer;
                                _tmp_39->value = ros->denm.situation.eventHistory.elements[a].eventPosition.deltaAltitude.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaAltitude);
                            
                            if(ros->denm.situation.eventHistory.elements[a].eventDeltaTimePresent) {
                                // Field name: eventDeltaTime
                                // Value
                                wind::cpp::ITS_Container::PathDeltaTime* _tmp_40 =
                                    (wind::cpp::ITS_Container::PathDeltaTime*)buffer;
                                _tmp_40->value = ros->denm.situation.eventHistory.elements[a].eventDeltaTime.value;
                                buffer += sizeof(wind::cpp::ITS_Container::PathDeltaTime);
                            }
                            
                            // Field name: informationQuality
                            // Value
                            wind::cpp::ITS_Container::InformationQuality* _tmp_41 =
                                (wind::cpp::ITS_Container::InformationQuality*)buffer;
                            _tmp_41->value = ros->denm.situation.eventHistory.elements[a].informationQuality.value;
                            buffer += sizeof(wind::cpp::ITS_Container::InformationQuality);
                            
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
                char* _tmp_42 = (char*)buffer;
                *_tmp_42 = (ros->denm.location.eventSpeedPresent? 1: 0);
                buffer++;
                
                char* _tmp_43 = (char*)buffer;
                *_tmp_43 = (ros->denm.location.eventPositionHeadingPresent? 1: 0);
                buffer++;
                
                char* _tmp_44 = (char*)buffer;
                *_tmp_44 = (ros->denm.location.roadTypePresent? 1: 0);
                buffer++;
                
                if(ros->denm.location.eventSpeedPresent) {
                    // Field name: eventSpeed
                        // Speed  SEQUENCE
                           //  speedValue      SpeedValue        
                           //  speedConfidence SpeedConfidence   
                        // Field name: speedValue
                        // Value
                        wind::cpp::ITS_Container::SpeedValue* _tmp_45 =
                            (wind::cpp::ITS_Container::SpeedValue*)buffer;
                        _tmp_45->value = ros->denm.location.eventSpeed.speedValue.value;
                        buffer += sizeof(wind::cpp::ITS_Container::SpeedValue);
                        
                        // Field name: speedConfidence
                        // Value
                        wind::cpp::ITS_Container::SpeedConfidence* _tmp_46 =
                            (wind::cpp::ITS_Container::SpeedConfidence*)buffer;
                        _tmp_46->value = ros->denm.location.eventSpeed.speedConfidence.value;
                        buffer += sizeof(wind::cpp::ITS_Container::SpeedConfidence);
                }
                
                if(ros->denm.location.eventPositionHeadingPresent) {
                    // Field name: eventPositionHeading
                        // Heading  SEQUENCE
                           //  headingValue      HeadingValue        
                           //  headingConfidence HeadingConfidence   
                        // Field name: headingValue
                        // Value
                        wind::cpp::ITS_Container::HeadingValue* _tmp_47 =
                            (wind::cpp::ITS_Container::HeadingValue*)buffer;
                        _tmp_47->value = ros->denm.location.eventPositionHeading.headingValue.value;
                        buffer += sizeof(wind::cpp::ITS_Container::HeadingValue);
                        
                        // Field name: headingConfidence
                        // Value
                        wind::cpp::ITS_Container::HeadingConfidence* _tmp_48 =
                            (wind::cpp::ITS_Container::HeadingConfidence*)buffer;
                        _tmp_48->value = ros->denm.location.eventPositionHeading.headingConfidence.value;
                        buffer += sizeof(wind::cpp::ITS_Container::HeadingConfidence);
                }
                
                // Field name: traces
                // SequenceOf
                // Data Type Byte
                char* _tmp_49 = (char*)buffer;
                *_tmp_49 = ros->denm.location.traces.count;
                buffer++;
                
                int count_b = ros->denm.location.traces.count;
                for(int b = 0; b < count_b; b++) {
                    
                    // SequenceOf
                    // Data Type Byte
                    char* _tmp_50 = (char*)buffer;
                    *_tmp_50 = ros->denm.location.traces.elements[b].count;
                    buffer++;
                    
                    int count_c = ros->denm.location.traces.elements[b].count;
                    for(int c = 0; c < count_c; c++) {
                        
                            // PathPoint  SEQUENCE
                               //  pathPosition  DeltaReferencePosition   
                               //  pathDeltaTime PathDeltaTime            OPTIONAL
                            char* _tmp_51 = (char*)buffer;
                            *_tmp_51 = (ros->denm.location.traces.elements[b].elements[c].pathDeltaTimePresent? 1: 0);
                            buffer++;
                            
                            // Field name: pathPosition
                                // DeltaReferencePosition  SEQUENCE
                                   //  deltaLatitude  DeltaLatitude    
                                   //  deltaLongitude DeltaLongitude   
                                   //  deltaAltitude  DeltaAltitude    
                                // Field name: deltaLatitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLatitude* _tmp_52 =
                                    (wind::cpp::ITS_Container::DeltaLatitude*)buffer;
                                _tmp_52->value = ros->denm.location.traces.elements[b].elements[c].pathPosition.deltaLatitude.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLatitude);
                                
                                // Field name: deltaLongitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLongitude* _tmp_53 =
                                    (wind::cpp::ITS_Container::DeltaLongitude*)buffer;
                                _tmp_53->value = ros->denm.location.traces.elements[b].elements[c].pathPosition.deltaLongitude.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLongitude);
                                
                                // Field name: deltaAltitude
                                // Value
                                wind::cpp::ITS_Container::DeltaAltitude* _tmp_54 =
                                    (wind::cpp::ITS_Container::DeltaAltitude*)buffer;
                                _tmp_54->value = ros->denm.location.traces.elements[b].elements[c].pathPosition.deltaAltitude.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaAltitude);
                            
                            if(ros->denm.location.traces.elements[b].elements[c].pathDeltaTimePresent) {
                                // Field name: pathDeltaTime
                                // Value
                                wind::cpp::ITS_Container::PathDeltaTime* _tmp_55 =
                                    (wind::cpp::ITS_Container::PathDeltaTime*)buffer;
                                _tmp_55->value = ros->denm.location.traces.elements[b].elements[c].pathDeltaTime.value;
                                buffer += sizeof(wind::cpp::ITS_Container::PathDeltaTime);
                            }
                            
                    }
                    
                }
                
                if(ros->denm.location.roadTypePresent) {
                    // Field name: roadType
                    // Value
                    wind::cpp::ITS_Container::RoadType* _tmp_56 =
                        (wind::cpp::ITS_Container::RoadType*)buffer;
                    _tmp_56->value = ros->denm.location.roadType.value;
                    buffer += sizeof(wind::cpp::ITS_Container::RoadType);
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
                char* _tmp_57 = (char*)buffer;
                *_tmp_57 = (ros->denm.alacarte.lanePositionPresent? 1: 0);
                buffer++;
                
                char* _tmp_58 = (char*)buffer;
                *_tmp_58 = (ros->denm.alacarte.impactReductionPresent? 1: 0);
                buffer++;
                
                char* _tmp_59 = (char*)buffer;
                *_tmp_59 = (ros->denm.alacarte.externalTemperaturePresent? 1: 0);
                buffer++;
                
                char* _tmp_60 = (char*)buffer;
                *_tmp_60 = (ros->denm.alacarte.roadWorksPresent? 1: 0);
                buffer++;
                
                char* _tmp_61 = (char*)buffer;
                *_tmp_61 = (ros->denm.alacarte.positioningSolutionPresent? 1: 0);
                buffer++;
                
                char* _tmp_62 = (char*)buffer;
                *_tmp_62 = (ros->denm.alacarte.stationaryVehiclePresent? 1: 0);
                buffer++;
                
                if(ros->denm.alacarte.lanePositionPresent) {
                    // Field name: lanePosition
                    // Value
                    wind::cpp::ITS_Container::LanePosition* _tmp_63 =
                        (wind::cpp::ITS_Container::LanePosition*)buffer;
                    _tmp_63->value = ros->denm.alacarte.lanePosition.value;
                    buffer += sizeof(wind::cpp::ITS_Container::LanePosition);
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
                        wind::cpp::ITS_Container::HeightLonCarr* _tmp_64 =
                            (wind::cpp::ITS_Container::HeightLonCarr*)buffer;
                        _tmp_64->value = ros->denm.alacarte.impactReduction.heightLonCarrLeft.value;
                        buffer += sizeof(wind::cpp::ITS_Container::HeightLonCarr);
                        
                        // Field name: heightLonCarrRight
                        // Value
                        wind::cpp::ITS_Container::HeightLonCarr* _tmp_65 =
                            (wind::cpp::ITS_Container::HeightLonCarr*)buffer;
                        _tmp_65->value = ros->denm.alacarte.impactReduction.heightLonCarrRight.value;
                        buffer += sizeof(wind::cpp::ITS_Container::HeightLonCarr);
                        
                        // Field name: posLonCarrLeft
                        // Value
                        wind::cpp::ITS_Container::PosLonCarr* _tmp_66 =
                            (wind::cpp::ITS_Container::PosLonCarr*)buffer;
                        _tmp_66->value = ros->denm.alacarte.impactReduction.posLonCarrLeft.value;
                        buffer += sizeof(wind::cpp::ITS_Container::PosLonCarr);
                        
                        // Field name: posLonCarrRight
                        // Value
                        wind::cpp::ITS_Container::PosLonCarr* _tmp_67 =
                            (wind::cpp::ITS_Container::PosLonCarr*)buffer;
                        _tmp_67->value = ros->denm.alacarte.impactReduction.posLonCarrRight.value;
                        buffer += sizeof(wind::cpp::ITS_Container::PosLonCarr);
                        
                        // Field name: positionOfPillars
                        // SequenceOf
                        // Data Type Byte
                        char* _tmp_68 = (char*)buffer;
                        *_tmp_68 = ros->denm.alacarte.impactReduction.positionOfPillars.count;
                        buffer++;
                        
                        int count_d = ros->denm.alacarte.impactReduction.positionOfPillars.count;
                        for(int d = 0; d < count_d; d++) {
                            
                            // Value
                            wind::cpp::ITS_Container::PosPillar* _tmp_69 =
                                (wind::cpp::ITS_Container::PosPillar*)buffer;
                            _tmp_69->value = ros->denm.alacarte.impactReduction.positionOfPillars.elements[d].value;
                            buffer += sizeof(wind::cpp::ITS_Container::PosPillar);
                        }
                        
                        // Field name: posCentMass
                        // Value
                        wind::cpp::ITS_Container::PosCentMass* _tmp_70 =
                            (wind::cpp::ITS_Container::PosCentMass*)buffer;
                        _tmp_70->value = ros->denm.alacarte.impactReduction.posCentMass.value;
                        buffer += sizeof(wind::cpp::ITS_Container::PosCentMass);
                        
                        // Field name: wheelBaseVehicle
                        // Value
                        wind::cpp::ITS_Container::WheelBaseVehicle* _tmp_71 =
                            (wind::cpp::ITS_Container::WheelBaseVehicle*)buffer;
                        _tmp_71->value = ros->denm.alacarte.impactReduction.wheelBaseVehicle.value;
                        buffer += sizeof(wind::cpp::ITS_Container::WheelBaseVehicle);
                        
                        // Field name: turningRadius
                        // Value
                        wind::cpp::ITS_Container::TurningRadius* _tmp_72 =
                            (wind::cpp::ITS_Container::TurningRadius*)buffer;
                        _tmp_72->value = ros->denm.alacarte.impactReduction.turningRadius.value;
                        buffer += sizeof(wind::cpp::ITS_Container::TurningRadius);
                        
                        // Field name: posFrontAx
                        // Value
                        wind::cpp::ITS_Container::PosFrontAx* _tmp_73 =
                            (wind::cpp::ITS_Container::PosFrontAx*)buffer;
                        _tmp_73->value = ros->denm.alacarte.impactReduction.posFrontAx.value;
                        buffer += sizeof(wind::cpp::ITS_Container::PosFrontAx);
                        
                        // Field name: positionOfOccupants
                        // BitString
                        wind::cpp::ITS_Container::PositionOfOccupants* _tmp_74 =
                            (wind::cpp::ITS_Container::PositionOfOccupants*)buffer;
                        buffer += sizeof(wind::cpp::ITS_Container::PositionOfOccupants);
                        for(int e = 0; e < 20; e++)
                            _tmp_74->values[e] = 
                                ros->denm.alacarte.impactReduction.positionOfOccupants.values[e];
                        
                        // Field name: vehicleMass
                        // Value
                        wind::cpp::ITS_Container::VehicleMass* _tmp_75 =
                            (wind::cpp::ITS_Container::VehicleMass*)buffer;
                        _tmp_75->value = ros->denm.alacarte.impactReduction.vehicleMass.value;
                        buffer += sizeof(wind::cpp::ITS_Container::VehicleMass);
                        
                        // Field name: requestResponseIndication
                        // Value
                        wind::cpp::ITS_Container::RequestResponseIndication* _tmp_76 =
                            (wind::cpp::ITS_Container::RequestResponseIndication*)buffer;
                        _tmp_76->value = ros->denm.alacarte.impactReduction.requestResponseIndication.value;
                        buffer += sizeof(wind::cpp::ITS_Container::RequestResponseIndication);
                }
                
                if(ros->denm.alacarte.externalTemperaturePresent) {
                    // Field name: externalTemperature
                    // Value
                    wind::cpp::ITS_Container::Temperature* _tmp_77 =
                        (wind::cpp::ITS_Container::Temperature*)buffer;
                    _tmp_77->value = ros->denm.alacarte.externalTemperature.value;
                    buffer += sizeof(wind::cpp::ITS_Container::Temperature);
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
                        char* _tmp_78 = (char*)buffer;
                        *_tmp_78 = (ros->denm.alacarte.roadWorks.lightBarSirenInUsePresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_79 = (char*)buffer;
                        *_tmp_79 = (ros->denm.alacarte.roadWorks.closedLanesPresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_80 = (char*)buffer;
                        *_tmp_80 = (ros->denm.alacarte.roadWorks.restrictionPresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_81 = (char*)buffer;
                        *_tmp_81 = (ros->denm.alacarte.roadWorks.speedLimitPresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_82 = (char*)buffer;
                        *_tmp_82 = (ros->denm.alacarte.roadWorks.incidentIndicationPresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_83 = (char*)buffer;
                        *_tmp_83 = (ros->denm.alacarte.roadWorks.recommendedPathPresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_84 = (char*)buffer;
                        *_tmp_84 = (ros->denm.alacarte.roadWorks.startingPointSpeedLimitPresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_85 = (char*)buffer;
                        *_tmp_85 = (ros->denm.alacarte.roadWorks.trafficFlowRulePresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_86 = (char*)buffer;
                        *_tmp_86 = (ros->denm.alacarte.roadWorks.referenceDenmsPresent? 1: 0);
                        buffer++;
                        
                        if(ros->denm.alacarte.roadWorks.lightBarSirenInUsePresent) {
                            // Field name: lightBarSirenInUse
                            // BitString
                            wind::cpp::ITS_Container::LightBarSirenInUse* _tmp_87 =
                                (wind::cpp::ITS_Container::LightBarSirenInUse*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::LightBarSirenInUse);
                            for(int f = 0; f < 2; f++)
                                _tmp_87->values[f] = 
                                    ros->denm.alacarte.roadWorks.lightBarSirenInUse.values[f];
                        }
                        
                        if(ros->denm.alacarte.roadWorks.closedLanesPresent) {
                            // Field name: closedLanes
                                // ClosedLanes  SEQUENCE
                                   //  innerhardShoulderStatus HardShoulderStatus   OPTIONAL
                                   //  outerhardShoulderStatus HardShoulderStatus   OPTIONAL
                                   //  drivingLaneStatus       DrivingLaneStatus    OPTIONAL
                                char* _tmp_88 = (char*)buffer;
                                *_tmp_88 = (ros->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatusPresent? 1: 0);
                                buffer++;
                                
                                char* _tmp_89 = (char*)buffer;
                                *_tmp_89 = (ros->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatusPresent? 1: 0);
                                buffer++;
                                
                                char* _tmp_90 = (char*)buffer;
                                *_tmp_90 = (ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent? 1: 0);
                                buffer++;
                                
                                if(ros->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatusPresent) {
                                    // Field name: innerhardShoulderStatus
                                    // Value
                                    wind::cpp::ITS_Container::HardShoulderStatus* _tmp_91 =
                                        (wind::cpp::ITS_Container::HardShoulderStatus*)buffer;
                                    _tmp_91->value = ros->denm.alacarte.roadWorks.closedLanes.innerhardShoulderStatus.value;
                                    buffer += sizeof(wind::cpp::ITS_Container::HardShoulderStatus);
                                }
                                
                                if(ros->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatusPresent) {
                                    // Field name: outerhardShoulderStatus
                                    // Value
                                    wind::cpp::ITS_Container::HardShoulderStatus* _tmp_92 =
                                        (wind::cpp::ITS_Container::HardShoulderStatus*)buffer;
                                    _tmp_92->value = ros->denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatus.value;
                                    buffer += sizeof(wind::cpp::ITS_Container::HardShoulderStatus);
                                }
                                
                                if(ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent) {
                                    // Field name: drivingLaneStatus
                                    // BitString
                                    wind::cpp::ITS_Container::DrivingLaneStatus* _tmp_93 =
                                        (wind::cpp::ITS_Container::DrivingLaneStatus*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::DrivingLaneStatus);
                                    for(int g = 0; g < 13; g++)
                                        _tmp_93->values[g] = 
                                            ros->denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values[g];
                                }
                        }
                        
                        if(ros->denm.alacarte.roadWorks.restrictionPresent) {
                            // Field name: restriction
                            // SequenceOf
                            // Data Type Byte
                            char* _tmp_94 = (char*)buffer;
                            *_tmp_94 = ros->denm.alacarte.roadWorks.restriction.count;
                            buffer++;
                            
                            int count_h = ros->denm.alacarte.roadWorks.restriction.count;
                            for(int h = 0; h < count_h; h++) {
                                
                                // Value
                                wind::cpp::ITS_Container::StationType* _tmp_95 =
                                    (wind::cpp::ITS_Container::StationType*)buffer;
                                _tmp_95->value = ros->denm.alacarte.roadWorks.restriction.elements[h].value;
                                buffer += sizeof(wind::cpp::ITS_Container::StationType);
                            }
                        }
                        
                        if(ros->denm.alacarte.roadWorks.speedLimitPresent) {
                            // Field name: speedLimit
                            // Value
                            wind::cpp::ITS_Container::SpeedLimit* _tmp_96 =
                                (wind::cpp::ITS_Container::SpeedLimit*)buffer;
                            _tmp_96->value = ros->denm.alacarte.roadWorks.speedLimit.value;
                            buffer += sizeof(wind::cpp::ITS_Container::SpeedLimit);
                        }
                        
                        if(ros->denm.alacarte.roadWorks.incidentIndicationPresent) {
                            // Field name: incidentIndication
                                // CauseCode  SEQUENCE
                                   //  causeCode    CauseCodeType      
                                   //  subCauseCode SubCauseCodeType   
                                // Field name: causeCode
                                // Value
                                wind::cpp::ITS_Container::CauseCodeType* _tmp_97 =
                                    (wind::cpp::ITS_Container::CauseCodeType*)buffer;
                                _tmp_97->value = ros->denm.alacarte.roadWorks.incidentIndication.causeCode.value;
                                buffer += sizeof(wind::cpp::ITS_Container::CauseCodeType);
                                
                                // Field name: subCauseCode
                                // Value
                                wind::cpp::ITS_Container::SubCauseCodeType* _tmp_98 =
                                    (wind::cpp::ITS_Container::SubCauseCodeType*)buffer;
                                _tmp_98->value = ros->denm.alacarte.roadWorks.incidentIndication.subCauseCode.value;
                                buffer += sizeof(wind::cpp::ITS_Container::SubCauseCodeType);
                        }
                        
                        if(ros->denm.alacarte.roadWorks.recommendedPathPresent) {
                            // Field name: recommendedPath
                            // SequenceOf
                            // Data Type Byte
                            char* _tmp_99 = (char*)buffer;
                            *_tmp_99 = ros->denm.alacarte.roadWorks.recommendedPath.count;
                            buffer++;
                            
                            int count_i = ros->denm.alacarte.roadWorks.recommendedPath.count;
                            for(int i = 0; i < count_i; i++) {
                                
                                    // ReferencePosition  SEQUENCE
                                       //  latitude                  Latitude               
                                       //  longitude                 Longitude              
                                       //  positionConfidenceEllipse PosConfidenceEllipse   
                                       //  altitude                  Altitude               
                                    // Field name: latitude
                                    // Value
                                    wind::cpp::ITS_Container::Latitude* _tmp_100 =
                                        (wind::cpp::ITS_Container::Latitude*)buffer;
                                    _tmp_100->value = ros->denm.alacarte.roadWorks.recommendedPath.elements[i].latitude.value;
                                    buffer += sizeof(wind::cpp::ITS_Container::Latitude);
                                    
                                    // Field name: longitude
                                    // Value
                                    wind::cpp::ITS_Container::Longitude* _tmp_101 =
                                        (wind::cpp::ITS_Container::Longitude*)buffer;
                                    _tmp_101->value = ros->denm.alacarte.roadWorks.recommendedPath.elements[i].longitude.value;
                                    buffer += sizeof(wind::cpp::ITS_Container::Longitude);
                                    
                                    // Field name: positionConfidenceEllipse
                                        // PosConfidenceEllipse  SEQUENCE
                                           //  semiMajorConfidence  SemiAxisLength   
                                           //  semiMinorConfidence  SemiAxisLength   
                                           //  semiMajorOrientation HeadingValue     
                                        // Field name: semiMajorConfidence
                                        // Value
                                        wind::cpp::ITS_Container::SemiAxisLength* _tmp_102 =
                                            (wind::cpp::ITS_Container::SemiAxisLength*)buffer;
                                        _tmp_102->value = ros->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMajorConfidence.value;
                                        buffer += sizeof(wind::cpp::ITS_Container::SemiAxisLength);
                                        
                                        // Field name: semiMinorConfidence
                                        // Value
                                        wind::cpp::ITS_Container::SemiAxisLength* _tmp_103 =
                                            (wind::cpp::ITS_Container::SemiAxisLength*)buffer;
                                        _tmp_103->value = ros->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMinorConfidence.value;
                                        buffer += sizeof(wind::cpp::ITS_Container::SemiAxisLength);
                                        
                                        // Field name: semiMajorOrientation
                                        // Value
                                        wind::cpp::ITS_Container::HeadingValue* _tmp_104 =
                                            (wind::cpp::ITS_Container::HeadingValue*)buffer;
                                        _tmp_104->value = ros->denm.alacarte.roadWorks.recommendedPath.elements[i].positionConfidenceEllipse.semiMajorOrientation.value;
                                        buffer += sizeof(wind::cpp::ITS_Container::HeadingValue);
                                    
                                    // Field name: altitude
                                        // Altitude  SEQUENCE
                                           //  altitudeValue      AltitudeValue        
                                           //  altitudeConfidence AltitudeConfidence   
                                        // Field name: altitudeValue
                                        // Value
                                        wind::cpp::ITS_Container::AltitudeValue* _tmp_105 =
                                            (wind::cpp::ITS_Container::AltitudeValue*)buffer;
                                        _tmp_105->value = ros->denm.alacarte.roadWorks.recommendedPath.elements[i].altitude.altitudeValue.value;
                                        buffer += sizeof(wind::cpp::ITS_Container::AltitudeValue);
                                        
                                        // Field name: altitudeConfidence
                                        // Value
                                        wind::cpp::ITS_Container::AltitudeConfidence* _tmp_106 =
                                            (wind::cpp::ITS_Container::AltitudeConfidence*)buffer;
                                        _tmp_106->value = ros->denm.alacarte.roadWorks.recommendedPath.elements[i].altitude.altitudeConfidence.value;
                                        buffer += sizeof(wind::cpp::ITS_Container::AltitudeConfidence);
                                    
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
                                wind::cpp::ITS_Container::DeltaLatitude* _tmp_107 =
                                    (wind::cpp::ITS_Container::DeltaLatitude*)buffer;
                                _tmp_107->value = ros->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaLatitude.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLatitude);
                                
                                // Field name: deltaLongitude
                                // Value
                                wind::cpp::ITS_Container::DeltaLongitude* _tmp_108 =
                                    (wind::cpp::ITS_Container::DeltaLongitude*)buffer;
                                _tmp_108->value = ros->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaLongitude.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaLongitude);
                                
                                // Field name: deltaAltitude
                                // Value
                                wind::cpp::ITS_Container::DeltaAltitude* _tmp_109 =
                                    (wind::cpp::ITS_Container::DeltaAltitude*)buffer;
                                _tmp_109->value = ros->denm.alacarte.roadWorks.startingPointSpeedLimit.deltaAltitude.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DeltaAltitude);
                        }
                        
                        if(ros->denm.alacarte.roadWorks.trafficFlowRulePresent) {
                            // Field name: trafficFlowRule
                            // Value
                            wind::cpp::ITS_Container::TrafficRule* _tmp_110 =
                                (wind::cpp::ITS_Container::TrafficRule*)buffer;
                            _tmp_110->value = ros->denm.alacarte.roadWorks.trafficFlowRule.value;
                            buffer += sizeof(wind::cpp::ITS_Container::TrafficRule);
                        }
                        
                        if(ros->denm.alacarte.roadWorks.referenceDenmsPresent) {
                            // Field name: referenceDenms
                            // SequenceOf
                            // Data Type Byte
                            char* _tmp_111 = (char*)buffer;
                            *_tmp_111 = ros->denm.alacarte.roadWorks.referenceDenms.count;
                            buffer++;
                            
                            int count_j = ros->denm.alacarte.roadWorks.referenceDenms.count;
                            for(int j = 0; j < count_j; j++) {
                                
                                    // ActionID  SEQUENCE
                                       //  originatingStationID StationID        
                                       //  sequenceNumber       SequenceNumber   
                                    // Field name: originatingStationID
                                    // Value
                                    wind::cpp::ITS_Container::StationID* _tmp_112 =
                                        (wind::cpp::ITS_Container::StationID*)buffer;
                                    _tmp_112->value = ros->denm.alacarte.roadWorks.referenceDenms.elements[j].originatingStationID.value;
                                    buffer += sizeof(wind::cpp::ITS_Container::StationID);
                                    
                                    // Field name: sequenceNumber
                                    // Value
                                    wind::cpp::ITS_Container::SequenceNumber* _tmp_113 =
                                        (wind::cpp::ITS_Container::SequenceNumber*)buffer;
                                    _tmp_113->value = ros->denm.alacarte.roadWorks.referenceDenms.elements[j].sequenceNumber.value;
                                    buffer += sizeof(wind::cpp::ITS_Container::SequenceNumber);
                                    
                            }
                        }
                }
                
                if(ros->denm.alacarte.positioningSolutionPresent) {
                    // Field name: positioningSolution
                    // Value
                    wind::cpp::ITS_Container::PositioningSolutionType* _tmp_114 =
                        (wind::cpp::ITS_Container::PositioningSolutionType*)buffer;
                    _tmp_114->value = ros->denm.alacarte.positioningSolution.value;
                    buffer += sizeof(wind::cpp::ITS_Container::PositioningSolutionType);
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
                        char* _tmp_115 = (char*)buffer;
                        *_tmp_115 = (ros->denm.alacarte.stationaryVehicle.stationarySincePresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_116 = (char*)buffer;
                        *_tmp_116 = (ros->denm.alacarte.stationaryVehicle.stationaryCausePresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_117 = (char*)buffer;
                        *_tmp_117 = (ros->denm.alacarte.stationaryVehicle.carryingDangerousGoodsPresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_118 = (char*)buffer;
                        *_tmp_118 = (ros->denm.alacarte.stationaryVehicle.numberOfOccupantsPresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_119 = (char*)buffer;
                        *_tmp_119 = (ros->denm.alacarte.stationaryVehicle.vehicleIdentificationPresent? 1: 0);
                        buffer++;
                        
                        char* _tmp_120 = (char*)buffer;
                        *_tmp_120 = (ros->denm.alacarte.stationaryVehicle.energyStorageTypePresent? 1: 0);
                        buffer++;
                        
                        if(ros->denm.alacarte.stationaryVehicle.stationarySincePresent) {
                            // Field name: stationarySince
                            // Value
                            wind::cpp::ITS_Container::StationarySince* _tmp_121 =
                                (wind::cpp::ITS_Container::StationarySince*)buffer;
                            _tmp_121->value = ros->denm.alacarte.stationaryVehicle.stationarySince.value;
                            buffer += sizeof(wind::cpp::ITS_Container::StationarySince);
                        }
                        
                        if(ros->denm.alacarte.stationaryVehicle.stationaryCausePresent) {
                            // Field name: stationaryCause
                                // CauseCode  SEQUENCE
                                   //  causeCode    CauseCodeType      
                                   //  subCauseCode SubCauseCodeType   
                                // Field name: causeCode
                                // Value
                                wind::cpp::ITS_Container::CauseCodeType* _tmp_122 =
                                    (wind::cpp::ITS_Container::CauseCodeType*)buffer;
                                _tmp_122->value = ros->denm.alacarte.stationaryVehicle.stationaryCause.causeCode.value;
                                buffer += sizeof(wind::cpp::ITS_Container::CauseCodeType);
                                
                                // Field name: subCauseCode
                                // Value
                                wind::cpp::ITS_Container::SubCauseCodeType* _tmp_123 =
                                    (wind::cpp::ITS_Container::SubCauseCodeType*)buffer;
                                _tmp_123->value = ros->denm.alacarte.stationaryVehicle.stationaryCause.subCauseCode.value;
                                buffer += sizeof(wind::cpp::ITS_Container::SubCauseCodeType);
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
                                char* _tmp_124 = (char*)buffer;
                                *_tmp_124 = (ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCodePresent? 1: 0);
                                buffer++;
                                
                                char* _tmp_125 = (char*)buffer;
                                *_tmp_125 = (ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumberPresent? 1: 0);
                                buffer++;
                                
                                char* _tmp_126 = (char*)buffer;
                                *_tmp_126 = (ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyNamePresent? 1: 0);
                                buffer++;
                                
                                // Field name: dangerousGoodsType
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsBasic* _tmp_127 =
                                    (wind::cpp::ITS_Container::DangerousGoodsBasic*)buffer;
                                _tmp_127->value = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.dangerousGoodsType.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsBasic);
                                
                                // Field name: unNumber
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsExtended_unNumber* _tmp_128 =
                                    (wind::cpp::ITS_Container::DangerousGoodsExtended_unNumber*)buffer;
                                _tmp_128->value = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.unNumber.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_unNumber);
                                
                                // Field name: elevatedTemperature
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsExtended_elevatedTemperature* _tmp_129 =
                                    (wind::cpp::ITS_Container::DangerousGoodsExtended_elevatedTemperature*)buffer;
                                _tmp_129->value = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.elevatedTemperature.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_elevatedTemperature);
                                
                                // Field name: tunnelsRestricted
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsExtended_tunnelsRestricted* _tmp_130 =
                                    (wind::cpp::ITS_Container::DangerousGoodsExtended_tunnelsRestricted*)buffer;
                                _tmp_130->value = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.tunnelsRestricted.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_tunnelsRestricted);
                                
                                // Field name: limitedQuantity
                                // Value
                                wind::cpp::ITS_Container::DangerousGoodsExtended_limitedQuantity* _tmp_131 =
                                    (wind::cpp::ITS_Container::DangerousGoodsExtended_limitedQuantity*)buffer;
                                _tmp_131->value = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.limitedQuantity.value;
                                buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_limitedQuantity);
                                
                                if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCodePresent) {
                                    // Field name: emergencyActionCode
                                    // Text
                                    
                                    wind::cpp::ITS_Container::DangerousGoodsExtended_emergencyActionCode* _tmp_132 = (wind::cpp::ITS_Container::DangerousGoodsExtended_emergencyActionCode*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_emergencyActionCode);
                                    
                                    for(int k = 0; k < 24; k++) {  // DangerousGoodsExtended_emergencyActionCode
                                        if(k < ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCode.value.length())
                                            _tmp_132->value[k] = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.emergencyActionCode.value.c_str()[k];
                                        else
                                            _tmp_132->value[k] = ' ';
                                    }
                                }
                                
                                if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumberPresent) {
                                    // Field name: phoneNumber
                                    // Text
                                    
                                    wind::cpp::ITS_Container::PhoneNumber* _tmp_133 = (wind::cpp::ITS_Container::PhoneNumber*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::PhoneNumber);
                                    
                                    for(int l = 0; l < 16; l++) {  // PhoneNumber
                                        if(l < ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumber.value.length())
                                            _tmp_133->value[l] = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.phoneNumber.value.c_str()[l];
                                        else
                                            _tmp_133->value[l] = ' ';
                                    }
                                }
                                
                                if(ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyNamePresent) {
                                    // Field name: companyName
                                    // Text
                                    
                                    wind::cpp::ITS_Container::DangerousGoodsExtended_companyName* _tmp_134 = (wind::cpp::ITS_Container::DangerousGoodsExtended_companyName*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::DangerousGoodsExtended_companyName);
                                    
                                    for(int m = 0; m < 24; m++) {  // DangerousGoodsExtended_companyName
                                        if(m < ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyName.value.length())
                                            _tmp_134->value[m] = ros->denm.alacarte.stationaryVehicle.carryingDangerousGoods.companyName.value.c_str()[m];
                                        else
                                            _tmp_134->value[m] = ' ';
                                    }
                                }
                        }
                        
                        if(ros->denm.alacarte.stationaryVehicle.numberOfOccupantsPresent) {
                            // Field name: numberOfOccupants
                            // Value
                            wind::cpp::ITS_Container::NumberOfOccupants* _tmp_135 =
                                (wind::cpp::ITS_Container::NumberOfOccupants*)buffer;
                            _tmp_135->value = ros->denm.alacarte.stationaryVehicle.numberOfOccupants.value;
                            buffer += sizeof(wind::cpp::ITS_Container::NumberOfOccupants);
                        }
                        
                        if(ros->denm.alacarte.stationaryVehicle.vehicleIdentificationPresent) {
                            // Field name: vehicleIdentification
                                // VehicleIdentification  SEQUENCE
                                   //  wMInumber  WMInumber    OPTIONAL
                                   //  vDS        VDS          OPTIONAL
                                char* _tmp_136 = (char*)buffer;
                                *_tmp_136 = (ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumberPresent? 1: 0);
                                buffer++;
                                
                                char* _tmp_137 = (char*)buffer;
                                *_tmp_137 = (ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDSPresent? 1: 0);
                                buffer++;
                                
                                if(ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumberPresent) {
                                    // Field name: wMInumber
                                    // Text
                                    
                                    wind::cpp::ITS_Container::WMInumber* _tmp_138 = (wind::cpp::ITS_Container::WMInumber*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::WMInumber);
                                    
                                    for(int n = 0; n < 3; n++) {  // WMInumber
                                        if(n < ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumber.value.length())
                                            _tmp_138->value[n] = ros->denm.alacarte.stationaryVehicle.vehicleIdentification.wMInumber.value.c_str()[n];
                                        else
                                            _tmp_138->value[n] = ' ';
                                    }
                                }
                                
                                if(ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDSPresent) {
                                    // Field name: vDS
                                    // Text
                                    
                                    wind::cpp::ITS_Container::VDS* _tmp_139 = (wind::cpp::ITS_Container::VDS*)buffer;
                                    buffer += sizeof(wind::cpp::ITS_Container::VDS);
                                    
                                    for(int o = 0; o < 6; o++) {  // VDS
                                        if(o < ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDS.value.length())
                                            _tmp_139->value[o] = ros->denm.alacarte.stationaryVehicle.vehicleIdentification.vDS.value.c_str()[o];
                                        else
                                            _tmp_139->value[o] = ' ';
                                    }
                                }
                        }
                        
                        if(ros->denm.alacarte.stationaryVehicle.energyStorageTypePresent) {
                            // Field name: energyStorageType
                            // BitString
                            wind::cpp::ITS_Container::EnergyStorageType* _tmp_140 =
                                (wind::cpp::ITS_Container::EnergyStorageType*)buffer;
                            buffer += sizeof(wind::cpp::ITS_Container::EnergyStorageType);
                            for(int p = 0; p < 7; p++)
                                _tmp_140->values[p] = 
                                    ros->denm.alacarte.stationaryVehicle.energyStorageType.values[p];
                        }
                }
        }
    

    return buffer - start;
}
