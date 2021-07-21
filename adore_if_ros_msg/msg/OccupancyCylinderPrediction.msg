# *******************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *  Daniel Heß
# ********************************************************************************
int32 trackingID                #internal tracking id corresponding to TrafficParticipantDetection::trackingID
int32 branchID                  #different optional behaviors may be predicted - this is an id for the branch, which has to be unique for a single trackingid
int32 predecessorID             #predecessorID_ references the branchID_ of a predecessor prediction. -1 for no predecessor
OccupancyCylinder[] occupancy   #prediction of occupied space at a given time
float32 confidence              #the confidence that an agent with trackingID will be enclosed in the given occupancy at future points of time
