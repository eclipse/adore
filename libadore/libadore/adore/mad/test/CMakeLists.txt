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

if(BUILD_adore_TESTING) 
# adding test cases for the library
  # cases correspond to .cpp files in folder test
  set(adore_MAD_TEST_CASES
    test_alfunction
    test_adoremath
    test_CubicPiecewise_function
    test_centerandlanewidth
  )

  find_package(Catch2)
  include(CTest)
  include(Catch)

  foreach(testcase ${adore_MAD_TEST_CASES})
    add_executable(${testcase}) 
    target_sources(${testcase} PRIVATE ${testcase}.cpp)
    target_link_libraries(
        ${testcase} 
          PRIVATE 
            mad 
            Catch2::Catch2
            qpoases::qpoases
    )
    target_compile_features(${testcase} PRIVATE cxx_std_17)
    catch_discover_tests(${testcase})
  endforeach()
endif()
