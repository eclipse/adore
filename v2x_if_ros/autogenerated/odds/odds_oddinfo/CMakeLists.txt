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
# This file belongs to the DLR Wind project de.dlr.ts.v2x:odds:12.0
# 
# Module: ODDInfo {}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(odds_oddinfo)

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
    Bearing.msg
    EnvironmentalConditions.msg
    GeoInformation.msg
    InfraBoundary.msg
    InfraCategory.msg
    InfraEmergencyVehiclePresence.msg
    InfraEvents.msg
    InfraI2V.msg
    InfraTrafficCondition.msg
    InfraTrafficLOS.msg
    InfraZone.msg
    IsConstructionAreaTraffic.msg
    IsDynamicSpeedDisplay.msg
    IsEmergencVehiclePresent.msg
    LightingCondition.msg
    ODDInfo.msg
    ODDMSG.msg
    ODDS.msg
    ObstacleInformation.msg
    OperationalConditions.msg
    PositionWithBearing.msg
    Section.msg
    SectionID.msg
    SectionObject.msg
    SectionObjectList.msg
    SignalizedIntersection.msg
    StaticInfrastructure.msg
    Status.msg
    TimeStamp.msg
    ValidityInformation.msg
    ValidityInformationConfidence.msg
    WeatherCondition.msg
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
    src/odds_oddinfo_translator_wind2ros.cpp
    src/odds_oddinfo_receiver.cpp
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
    src/odds_oddinfo_translator_ros2wind.cpp
    src/odds_oddinfo_sender.cpp
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

