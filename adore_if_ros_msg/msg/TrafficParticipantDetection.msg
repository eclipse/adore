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
TrafficParticipant data#information about traffic participant
int32 trackingID #internal tracking id, providing highest possible continuity
# geometric classification -> on ego surface, above ego surface, below ego surface, onRoad, onSidewalk, onTracks, onEmergencyLane, onParkingSpot, onPedestrianCrossing, onBikeLane,
# intent classification -> parked, parking, laneFollowing, turning, yieldingToEgo, yieldingToOther, slowingDown, crashed, waitingAtRedlightQueue, embarkingPassengers, waitingToPassLaneblock, waitingAtPedestrianCrossing, jaywalking
uint32 detection_by_sensor # bit array -> sensor type values: which sensors detected the object?

#sensor type bits
uint32 UNDEFINED = 1
uint32 RADAR = 2
uint32 LIDAR = 4
uint32 MONO_VIDEO = 8
uint32 STEREO_VISION = 16
uint32 SPERICAL_CAMERA = 32
uint32 NIGHT_VISION = 64
uint32 ULTRASONIC = 128
uint32 INDUCTIONLOOP = 256
uint32 PMD = 512
uint32 V2X_CAM = 1024
uint32 V2X_CPM = 2048
uint32 V2X_MCM = 4096
uint32 SPECULATIVE = 2147483648 # 2^31, not observed by any sensor but rather speculated to exist in occlusion area
