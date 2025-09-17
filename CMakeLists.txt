cmake_minimum_required(VERSION 3.14)

# Config for radarsimc
project(radarsimc LANGUAGES CXX)

# Avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP in CMake 3.24:
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
    cmake_policy(SET CMP0135 NEW)
endif()

# Options
option(GPU_BUILD "Enable GPU/CUDA build" OFF)
option(GTEST "Enable Google Test" OFF)

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
        "./src/radarsimcpp/hdf5-lib-build/libs/lib_linux_gcc11_x86_64/include")
    link_directories("${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5-lib-build/libs/lib_linux_gcc11_x86_64/lib")
    set(_hdf5_libs
        hdf5
        hdf5_cpp
        hdf5_hl
        hdf5_hl_cpp
    )
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # macOS
    include_directories("./src/includes"
        "./src/radarsimcpp/includes")
    
    # Set HDF5 paths based on architecture
    if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm64")
        include_directories("./src/radarsimcpp/hdf5-lib-build/libs/lib_macos_arm64/include")
        link_directories("${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5-lib-build/libs/lib_macos_arm64/lib")
    elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
        include_directories("./src/radarsimcpp/hdf5-lib-build/libs/lib_macos_x86_64/include")
        link_directories("${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5-lib-build/libs/lib_macos_x86_64/lib")
    else()
        # Fallback to x86_64 for unknown architectures
        include_directories("./src/radarsimcpp/hdf5-lib-build/libs/lib_macos_x86_64/include")
        link_directories("${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5-lib-build/libs/lib_macos_x86_64/lib")
    endif()
    
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

# Google Test setup
if(GTEST)
    message(STATUS "Enabling Google Test")

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    )

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Collect test sources
    file(GLOB_RECURSE TEST_SOURCES "tests/*.cpp")

    # Create test executable
    add_executable(radarsimc_test ${TEST_SOURCES})

    # Link with gtest and main library
    target_link_libraries(radarsimc_test
        PRIVATE
        gtest_main
        radarsimc
    )

    # GPU-specific test configuration
    if(GPU_BUILD)
        # Add CUDA includes
        target_include_directories(radarsimc_test PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

        # Set CUDA properties for tests
        set_target_properties(radarsimc_test PROPERTIES
            CUDA_SEPARABLE_COMPILATION ON
            CUDA_ARCHITECTURES "75;86") # Limited architectures for tests

        # Mark test sources as CUDA
        set_source_files_properties(${TEST_SOURCES} PROPERTIES LANGUAGE CUDA)

        # Add CUDA-specific compile definitions
        target_compile_definitions(radarsimc_test PRIVATE _CUDATEST_ _CUDA_)

        # Handle MSVC-specific linking for tests
        if(MSVC)
            # Use proper linker flags for CUDA test builds to avoid runtime library conflicts
            set_target_properties(radarsimc_test PROPERTIES
                LINK_FLAGS "/NODEFAULTLIB:LIBCMT")
        endif()
    endif()

    # Enable testing and add test
    enable_testing()
    add_test(NAME radarsimc_test COMMAND radarsimc_test)
endif()
