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
#*   Daniel Heß
#********************************************************************************

if(BUILD_adore_TESTING) #why not always build tests?
# adding test cases for the library
  find_package(Catch2 CONFIG)
  
  # cases correspond to .cpp files in folder test
  add_executable(test_adore_fun) 
  target_sources(test_adore_fun PRIVATE 
    test_adore_fun.cpp
    test_setpointrequestswath.cpp
  )
  target_link_libraries(test_adore_fun PRIVATE fun Catch2::Catch2)
  target_compile_features(test_adore_fun PRIVATE cxx_std_17)

  include(CTest)
  include(Catch)
  catch_discover_tests(test_adore_fun)

endif()