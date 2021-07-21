#********************************************************************************
#* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
#*
#* This program and the accompanying materials are made available under the 
#* terms of the Eclipse Public License 2.0 which is available at
#* http://www.eclipse.org/legal/epl-2.0.
#*
#* SPDX-License-Identifier: EPL-2.0 
#*
#* Contributors: 
#*   Thomas Lobig
#********************************************************************************


add_subdirectory("view" "${CMAKE_CURRENT_BINARY_DIR}/view")
add_library(adore::view ALIAS adore_view)
add_subdirectory("mad" "${CMAKE_CURRENT_BINARY_DIR}/mad")
add_library(adore::mad ALIAS adore_mad)
add_subdirectory("params" "${CMAKE_CURRENT_BINARY_DIR}/params")
add_library(adore::params ALIAS adore_params)
add_subdirectory("env" "${CMAKE_CURRENT_BINARY_DIR}/env")
add_library(adore::env ALIAS adore_env)
add_subdirectory("fun" "${CMAKE_CURRENT_BINARY_DIR}/fun")
add_library(adore::fun ALIAS adore_fun)
add_subdirectory("if_xodr" "${CMAKE_CURRENT_BINARY_DIR}/if_xodr")
add_library(adore::if_xodr ALIAS adore_if_xodr)
add_subdirectory("if_r2s" "${CMAKE_CURRENT_BINARY_DIR}/if_r2s")
add_library(adore::if_r2s ALIAS adore_if_r2s)
add_subdirectory("sim" "${CMAKE_CURRENT_BINARY_DIR}/sim")
add_library(adore::sim ALIAS adore_sim)
add_subdirectory("apps" "${CMAKE_CURRENT_BINARY_DIR}/apps")
add_library(adore::apps ALIAS adore_apps)