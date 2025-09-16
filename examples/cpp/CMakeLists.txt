cmake_minimum_required(VERSION 3.16)
project(RadarSimExample
    VERSION 1.0.0
    DESCRIPTION "RadarSim C/C++ Wrapper Library Example"
    LANGUAGES CXX C)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set C standard for legacy compatibility
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Build configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler-specific options
if(MSVC)
    add_compile_options(/W4)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the RadarSim library
set(RADARSIM_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/radarsimlib")

# Find the library file
if(WIN32)
    set(RADARSIM_LIB_NAME "radarsimc")
    set(RADARSIM_DLL_NAME "radarsimc.dll")

    # Only search in the radarsimlib release folder
    find_library(RADARSIM_LIBRARY
        NAMES ${RADARSIM_LIB_NAME}
        PATHS "${RADARSIM_LIB_DIR}"
        NO_DEFAULT_PATH
    )

    # Find the DLL for runtime
    find_file(RADARSIM_DLL
        NAMES ${RADARSIM_DLL_NAME}
        PATHS "${RADARSIM_LIB_DIR}"
        NO_DEFAULT_PATH
    )

    # Find the header file
    find_file(RADARSIM_HEADER
        NAMES "radarsim.h"
        PATHS "${RADARSIM_LIB_DIR}"
        NO_DEFAULT_PATH
    )
else()
    set(RADARSIM_LIB_NAME "radarsimc")

    # Only search in the radarsimlib release folder
    find_library(RADARSIM_LIBRARY
        NAMES ${RADARSIM_LIB_NAME}
        PATHS "${RADARSIM_LIB_DIR}"
        NO_DEFAULT_PATH
    )

    # Find the header file
    find_file(RADARSIM_HEADER
        NAMES "radarsim.h"
        PATHS "${RADARSIM_LIB_DIR}"
        NO_DEFAULT_PATH
    )
endif()

# Set include directory based on found header
if(RADARSIM_HEADER)
    get_filename_component(RADARSIM_INCLUDE_DIR "${RADARSIM_HEADER}" DIRECTORY)
    include_directories("${RADARSIM_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "RadarSim header file (radarsim.h) not found in ${RADARSIM_LIB_DIR}")
endif()

# Check if library was found
if(NOT RADARSIM_LIBRARY)
    message(FATAL_ERROR "RadarSim library not found in ${RADARSIM_LIB_DIR}. Please ensure the library is built using the build script.")
endif()

message(STATUS "Found RadarSim library: ${RADARSIM_LIBRARY}")
message(STATUS "Found RadarSim header: ${RADARSIM_HEADER}")

if(WIN32 AND RADARSIM_DLL)
    message(STATUS "Found RadarSim DLL: ${RADARSIM_DLL}")
endif()

# Create the executable from C++ source
add_executable(example_radarsim example_radarsim.cpp)

# Alternative: Create C version executable (commented out)
# add_executable(example_radarsim_c example_radarsim.c)

# Link libraries
target_link_libraries(example_radarsim
    ${RADARSIM_LIBRARY}
)

# Link math library on Unix systems
if(UNIX)
    target_link_libraries(example_radarsim m)
endif()

# Set output directory
set_target_properties(example_radarsim PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Copy DLL to output directory on Windows
if(WIN32 AND RADARSIM_DLL)
    add_custom_command(TARGET example_radarsim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${RADARSIM_DLL}"
        "$<TARGET_FILE_DIR:example_radarsim>"
        COMMENT "Copying RadarSim DLL to output directory"
    )
endif()

# Installation
install(TARGETS example_radarsim
    RUNTIME DESTINATION bin
)

if(WIN32 AND RADARSIM_DLL)
    install(FILES "${RADARSIM_DLL}"
        DESTINATION bin
    )
endif()

# Print build information
message(STATUS "")
message(STATUS "RadarSim Example Configuration:")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "  C Standard: C${CMAKE_C_STANDARD}")
message(STATUS "  RadarSim Library: ${RADARSIM_LIBRARY}")
message(STATUS "  RadarSim Header: ${RADARSIM_HEADER}")
message(STATUS "  Include Directory: ${RADARSIM_INCLUDE_DIR}")

if(WIN32 AND RADARSIM_DLL)
    message(STATUS "  RadarSim DLL: ${RADARSIM_DLL}")
endif()

message(STATUS "")

# Add a custom target for running the example
add_custom_target(run_example
    COMMAND example_radarsim
    DEPENDS example_radarsim
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    COMMENT "Running RadarSim example"
    USES_TERMINAL
)
