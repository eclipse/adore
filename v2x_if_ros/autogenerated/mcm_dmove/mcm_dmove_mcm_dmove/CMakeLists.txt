# 
# Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)
# Institute of Transportation Systems. (https://www.dlr.de/ts/)
# 
# Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# 
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0
# 
# 
# File automatically generated with DLR Wind v2 (2021)
# 
# This file belongs to the DLR Wind project de.dlr.ts.v2x:mcm_dmove:3.0
# 
# Module: MCM_DMove {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
#     wg1(1) en(302637) mcm(1) version(3)}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(mcm_dmove_mcm_dmove)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    its_container_v2_its_container
)
find_package(Boost)

add_message_files(
    FILES
    AdviceFollowed.msg
    AdviceID.msg
    AdviceLaneID.msg
    AdvicePosition.msg
    AdviceResponse.msg
    AdviceResponseList.msg
    BasicContainer.msg
    CarFollowingAdvice.msg
    DesiredBehaviour.msg
    DesiredTrajectory.msg
    DiffPosition.msg
    DiffTime.msg
    FollowingVehicle.msg
    GenerationDeltaTime.msg
    IntersectionID.msg
    IntersectionReferenceID.msg
    LaneAdvice.msg
    LaneAdviceReason.msg
    LaneChangeMoment.msg
    LaneChangePosition.msg
    LaneChangeSpeed.msg
    LateralDeviation.msg
    LeadingVehicle.msg
    MCM.msg
    ManeuverContainer.msg
    ManeuverCoordination.msg
    McmParameters.msg
    Millisecond.msg
    Minute.msg
    PlaceOfEndTransition.msg
    PlaceOfStartTransition.msg
    PlannedTrajectory.msg
    RequestID.msg
    RespectedDesiredTrajectoriesList.msg
    RespectedDesiredTrajectory.msg
    RoadRegulatorID.msg
    RoadSegmentID.msg
    RoadSegmentReferenceID.msg
    RsuManeuver.msg
    TargetAutomationLevel.msg
    TargetGap.msg
    TargetLane.msg
    TargetSpeed.msg
    TargetStationID.msg
    TimeDeviation.msg
    TimeOfTriggerTransition.msg
    TocAdvice.msg
    TocAdviceReason.msg
    ToleratedDistance.msg
    TrajectoryAdvice.msg
    TrajectoryAdvice_trajectory.msg
    TrajectoryPoint.msg
    TriggerTimeOfMRM.msg
    TriggerTimeOfToC.msg
    TriggeringPointOfToC.msg
    VehicleAdvice.msg
    VehicleAdviceList.msg
    VehicleCapabilities.msg
    VehicleManeuver.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    its_container_v2_its_container
)

##------------------  Receiver  ------------------##
add_executable(receiver
    src/receiver.cpp
    src/udp_receiver.cpp
    src/mcm_dmove_mcm_dmove_translator_wind2ros.cpp
    src/mcm_dmove_mcm_dmove_receiver.cpp
)
add_dependencies(receiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(receiver ${catkin_LIBRARIES})
target_include_directories(receiver PRIVATE include ${catkin_INCLUDE_DIRS})
install(TARGETS receiver RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
##------------------------------------------------##

##------------------   Sender   ------------------##
add_executable(sender
    src/sender.cpp
    src/udp_sender.cpp
    src/mcm_dmove_mcm_dmove_translator_ros2wind.cpp
    src/mcm_dmove_mcm_dmove_sender.cpp
)
add_dependencies(sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sender ${catkin_LIBRARIES})
target_include_directories(sender PRIVATE include ${catkin_INCLUDE_DIRS})
install(TARGETS sender RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
##------------------------------------------------##

catkin_package(
    INCLUDE_DIRS include  # This indicates that the folder "include" within the package folder is where exported headers go
    CATKIN_DEPENDS message_runtime std_msgs
    # CATKIN_DEPENDS message_runtime roscpp
)

