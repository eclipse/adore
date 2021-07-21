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
# This file belongs to the DLR Wind project de.dlr.ts.v2x:cpm_v1_19:1.3
# 
# Module: CPM_PDU_Descriptions {itu-t(0) identified-organization(4) etsi(0) itsDomain(5) 
#     wg1(1) ts(103324) cpm(1) version1(1)}
# 
# For support contact v2x-ts@dlr.de
# 
# 
cmake_minimum_required(VERSION 2.8.3)
project(cpm_v1_19_cpm_pdu_descriptions)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    cam_v2_cam_pdu_descriptions
    cpm_v1_19_cpm_commondatatypes_descriptions
    cpm_v1_19_cpm_freespaceaddendum_descriptions
    cpm_v1_19_cpm_originatingstationdata_descriptions
    cpm_v1_19_cpm_perceivedobject_descriptions
    cpm_v1_19_cpm_sensorinformation_descriptions
    its_container_v2_its_container
)
find_package(Boost)

add_message_files(
    FILES
    CPM.msg
    CollectivePerceptionMessage.msg
    CpmContainerId.msg
    CpmManagementContainer.msg
    CpmParameters.msg
    CpmParameters_perceptionData.msg
    CpmPerceptionDataContainer.msg
    CpmStationDataContainer.msg
    CpmStationDataId.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    its_container_v2_its_container
    cam_v2_cam_pdu_descriptions
    cpm_v1_19_cpm_freespaceaddendum_descriptions
    cpm_v1_19_cpm_sensorinformation_descriptions
    cpm_v1_19_cpm_originatingstationdata_descriptions
    cpm_v1_19_cpm_commondatatypes_descriptions
    cpm_v1_19_cpm_perceivedobject_descriptions
)

##------------------  Receiver  ------------------##
add_executable(receiver
    src/receiver.cpp
    src/udp_receiver.cpp
    src/cpm_v1_19_cpm_pdu_descriptions_translator_wind2ros.cpp
    src/cpm_v1_19_cpm_pdu_descriptions_receiver.cpp
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
    src/cpm_v1_19_cpm_pdu_descriptions_translator_ros2wind.cpp
    src/cpm_v1_19_cpm_pdu_descriptions_sender.cpp
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

