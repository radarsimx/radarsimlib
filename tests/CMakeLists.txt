# CMakeLists.txt for RadarSim C Wrapper Tests
#
# This CMake file configures the build for C wrapper unit tests
# for the RadarSim library.
#
# Dependencies:
# - Google Test framework
# - RadarSim C library (radarsimc)
# - Standard C++ libraries
#
# Usage:
# Include this in the main CMakeLists.txt or build separately:
# mkdir build && cd build
# cmake .. -DBUILD_CWRAPPER_TESTS=ON
# make
#
# Copyright (C) 2023 - PRESENT  radarsimx.com

cmake_minimum_required(VERSION 3.14)

# Option to build C wrapper tests
option(BUILD_CWRAPPER_TESTS "Build C wrapper tests" OFF)

if(BUILD_CWRAPPER_TESTS)
    # Find Google Test
    find_package(GTest REQUIRED)
    
    # Include directories
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/includes
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/radarsimcpp/includes
        ${GTEST_INCLUDE_DIRS}
    )
    
    # Define test source files
    set(CWRAPPER_TEST_SOURCES
        test_c_wrapper_radarsim.cpp
        test_c_wrapper_basic.cpp
        test_c_wrapper_error_handling.cpp
    )
    
    # Create test executables
    foreach(TEST_SOURCE ${CWRAPPER_TEST_SOURCES})
        # Extract test name from filename
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
        
        # Create executable
        add_executable(${TEST_NAME} ${TEST_SOURCE})
        
        # Link libraries
        target_link_libraries(${TEST_NAME}
            PRIVATE
            radarsimc           # The C library being tested
            GTest::GTest
            GTest::Main
            ${CMAKE_THREAD_LIBS_INIT}
        )
        
        # Set C++ standard
        set_target_properties(${TEST_NAME} PROPERTIES
            CXX_STANDARD 20
            CXX_STANDARD_REQUIRED ON
        )
        
        # Add compiler flags for better error reporting
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            target_compile_options(${TEST_NAME} PRIVATE -Wall -Wextra -Wpedantic)
        elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
            target_compile_options(${TEST_NAME} PRIVATE /W4)
        endif()
        
        # Register test with CTest
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
        
        # Set test properties
        set_tests_properties(${TEST_NAME} PROPERTIES
            TIMEOUT 300  # 5 minutes timeout
            LABELS "cwrapper;unit_test"
        )
    endforeach()
    
    # Create a combined test executable
    add_executable(cwrapper_tests_all ${CWRAPPER_TEST_SOURCES})
    
    target_link_libraries(cwrapper_tests_all
        PRIVATE
        radarsimc
        GTest::GTest
        GTest::Main
        ${CMAKE_THREAD_LIBS_INIT}
    )
    
    set_target_properties(cwrapper_tests_all PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
    )
    
    # Add to CTest
    add_test(NAME cwrapper_tests_all COMMAND cwrapper_tests_all)
    set_tests_properties(cwrapper_tests_all PROPERTIES
        TIMEOUT 600  # 10 minutes for all tests
        LABELS "cwrapper;unit_test;integration"
    )
    
    # Create custom target for running only C wrapper tests
    add_custom_target(test_cwrapper
        COMMAND ${CMAKE_CTEST_COMMAND} -L cwrapper --output-on-failure
        DEPENDS cwrapper_tests_all
        COMMENT "Running C wrapper tests"
    )
    
    # Print configuration info
    message(STATUS "C wrapper tests configured:")
    message(STATUS "  Test sources: ${CWRAPPER_TEST_SOURCES}")
    message(STATUS "  Google Test: ${GTEST_LIBRARIES}")
    message(STATUS "  Use 'make test_cwrapper' to run C wrapper tests only")
    
endif()

# Add option for coverage reporting (if available)
option(ENABLE_COVERAGE "Enable coverage reporting" OFF)

if(ENABLE_COVERAGE AND BUILD_CWRAPPER_TESTS)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        # Add coverage flags
        set(COVERAGE_FLAGS "--coverage")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COVERAGE_FLAGS}")
        
        # Find coverage tools
        find_program(GCOV_PATH gcov)
        find_program(LCOV_PATH lcov)
        find_program(GENHTML_PATH genhtml)
        
        if(GCOV_PATH AND LCOV_PATH AND GENHTML_PATH)
            # Add custom target for coverage report
            add_custom_target(coverage_cwrapper
                COMMAND ${LCOV_PATH} --directory . --capture --output-file coverage.info
                COMMAND ${LCOV_PATH} --remove coverage.info '/usr/*' --output-file coverage.info
                COMMAND ${LCOV_PATH} --list coverage.info
                COMMAND ${GENHTML_PATH} -o coverage_html coverage.info
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                COMMENT "Generating coverage report for C wrapper tests"
            )
            
            message(STATUS "Coverage reporting enabled for C wrapper tests")
            message(STATUS "  Use 'make coverage_cwrapper' to generate coverage report")
        else()
            message(WARNING "Coverage tools not found. Coverage reporting disabled.")
        endif()
    else()
        message(WARNING "Coverage reporting only supported with GCC or Clang")
    endif()
endif()
