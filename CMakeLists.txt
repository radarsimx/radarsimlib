cmake_minimum_required(VERSION 3.14)

# Config for radarsimc
project(radarsimc LANGUAGES CXX)

# Options
option(GPU_BUILD "Enable GPU/CUDA build" OFF)
option(GTEST "Enable Google Test" OFF)
option(FREETIER "Enable free tier build" OFF)

# force GPU build
# set(GPU_BUILD OFF)

# bypass CUDA compiler check
# set(CMAKE_CUDA_COMPILER_WORKS 1)
if(GPU_BUILD)
    # add _CUDA_ micro to the compiler
    add_definitions(-D_CUDA_)
endif()

if(FREETIER)
    add_definitions(-D_FREETIER_)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") # Windows
    include_directories("./src/includes"
        "./src/radarsimcpp/includes"
        "./src/radarsimcpp/hdf5-lib-build/libs/lib_win_x86_64/include")
    link_directories("${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5-lib-build/libs/lib_win_x86_64/lib")
    set(_hdf5_libs
        libhdf5
        libhdf5_cpp
        libhdf5_hl
        libhdf5_hl_cpp
        shlwapi
    )
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux") # Linux
    include_directories("./src/includes"
        "./src/radarsimcpp/includes"
        "./src/radarsimcpp/hdf5-lib-build/libs/lib_linux_x86_64/include")
    link_directories("${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5-lib-build/libs/lib_linux_x86_64/lib")
    set(_hdf5_libs
        hdf5
        hdf5_cpp
        hdf5_hl
        hdf5_hl_cpp
    )
endif()

if(GPU_BUILD)
    enable_language(CUDA)

    # add source files, ".cpp" and ".cu"
    file(GLOB SOURCES "./src/*.cu" "./src/*.cpp" "./src/radarsimcpp/src/*.cu" "./src/radarsimcpp/src/*.cpp")

    # use nvcc to compile all the *.cpp files
    set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CUDA)

    # build executable
    add_library(radarsimc SHARED ${SOURCES})

    # set CUDA building properties
    set_target_properties(radarsimc PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_ARCHITECTURES all-major)

    # add CUDA includes path
    include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

    # add _CUDA_ micro to the compiler
    target_compile_definitions(radarsimc PRIVATE _CUDA_ _CUDATEST_)

    # for MSVC, remove default libs
    if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        set_target_properties(radarsimc PROPERTIES LINK_FLAGS "/NODEFAULTLIB:\"LIBCMT\"")
    endif()

    target_link_libraries(radarsimc PUBLIC ${_hdf5_libs})
else()
    # add source files, ".cpp"
    file(GLOB SOURCES "./src/*.cpp" "./src/radarsimcpp/src/*.cpp")

    # build library
    add_library(radarsimc SHARED ${SOURCES})
    target_link_libraries(radarsimc PUBLIC ${_hdf5_libs})

    # add_executable(radarsimc ${SOURCES})
endif()

# Google Test setup for C wrapper
if(GTEST)
    message(STATUS "Enabling Google Test for RadarSim C wrapper")

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    )

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Include test directory if it exists
    if(EXISTS "${CMAKE_SOURCE_DIR}/tests")
        # Collect test sources from tests directory
        file(GLOB_RECURSE C_TEST_SOURCES "tests/*.cpp" "tests/*.c")
        
        if(C_TEST_SOURCES)
            # Create test executable
            add_executable(radarsim_c_test ${C_TEST_SOURCES})

            # Link with gtest and main library
            target_link_libraries(radarsim_c_test
                PRIVATE
                gtest_main
                radarsimc
                ${_hdf5_libs}
            )

            # Add include directories
            target_include_directories(radarsim_c_test 
                PRIVATE 
                "./src/includes"
                "./src/radarsimcpp/includes"
            )

            # Enable testing and add test
            enable_testing()
            add_test(NAME radarsim_c_test COMMAND radarsim_c_test)
        endif()
    endif()
endif()
