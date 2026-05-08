cmake_minimum_required(VERSION 3.18) # Increased for better CUDA support

# Avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP in CMake 3.24:
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
    cmake_policy(SET CMP0135 NEW)
endif()

# Options
option(GPU_BUILD "Enable GPU/CUDA build" OFF)
option(GTEST "Enable Google Test" OFF)
option(CMAKE_CUDA_COMPILER_WORKS "Bypass CUDA compiler check" OFF)
option(ENABLE_LICENSE "Enable license verification (requires mbedTLS)" OFF)

# Config for radarsimc
project(radarsimc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Instructs CMake to export all symbols for Windows builds
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Add cmake modules path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/src/radarsimcpp/cmake")

# Add global project include directories
include_directories("./src/includes" "./src/radarsimcpp/includes")

# Function to setup OpenMP for macOS with Homebrew
function(setup_openmp_macos)
    if(APPLE)
        # Try to find OpenMP with Homebrew paths
        set(OpenMP_ROOT_HINTS
            /opt/homebrew/opt/libomp # Apple Silicon
            /usr/local/opt/libomp # Intel Mac
        )

        foreach(hint_path ${OpenMP_ROOT_HINTS})
            if(EXISTS ${hint_path})
                set(OpenMP_ROOT ${hint_path})
                set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
                set(OpenMP_CXX_LIB_NAMES "omp")
                set(OpenMP_omp_LIBRARY ${hint_path}/lib/libomp.dylib)
                message(STATUS "Found OpenMP at: ${hint_path}")
                break()
            endif()
        endforeach()

        # Export to parent scope
        set(OpenMP_ROOT ${OpenMP_ROOT} PARENT_SCOPE)
        set(OpenMP_CXX_FLAGS ${OpenMP_CXX_FLAGS} PARENT_SCOPE)
        set(OpenMP_CXX_LIB_NAMES ${OpenMP_CXX_LIB_NAMES} PARENT_SCOPE)
        set(OpenMP_omp_LIBRARY ${OpenMP_omp_LIBRARY} PARENT_SCOPE)
    endif()
endfunction()

# Function to setup HDF5 for different platforms
function(setup_hdf5_for_platform)
    set(HDF5_BASE "${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5-lib-build/libs")

    if(WIN32)
        set(HDF5_INCLUDE_DIR "${HDF5_BASE}/include" "${HDF5_BASE}/include_win_x86_64")
        set(HDF5_LIB_DIR "${HDF5_BASE}/lib_win_x86_64")
        set(HDF5_LIBS libhdf5 libhdf5_cpp libhdf5_hl libhdf5_hl_cpp shlwapi)
    elseif(UNIX AND NOT APPLE)
        set(HDF5_INCLUDE_DIR "${HDF5_BASE}/include" "${HDF5_BASE}/include_linux_x86_64")
        set(HDF5_LIB_DIR "${HDF5_BASE}/lib_linux_x86_64")
        set(HDF5_LIBS
            "${HDF5_BASE}/lib_linux_x86_64/libhdf5.a"
            "${HDF5_BASE}/lib_linux_x86_64/libhdf5_cpp.a"
            "${HDF5_BASE}/lib_linux_x86_64/libhdf5_hl.a"
            "${HDF5_BASE}/lib_linux_x86_64/libhdf5_hl_cpp.a")
    elseif(APPLE)
        if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm64")
            set(HDF5_INCLUDE_DIR "${HDF5_BASE}/include" "${HDF5_BASE}/include_macos_arm64")
            set(HDF5_LIB_DIR "${HDF5_BASE}/lib_macos_arm64")
        else()
            set(HDF5_INCLUDE_DIR "${HDF5_BASE}/include" "${HDF5_BASE}/include_macos_x86_64")
            set(HDF5_LIB_DIR "${HDF5_BASE}/lib_macos_x86_64")
        endif()

        set(HDF5_LIBS hdf5 hdf5_cpp hdf5_hl hdf5_hl_cpp)
    endif()

    # Export to parent scope
    set(HDF5_INCLUDE_DIR ${HDF5_INCLUDE_DIR} PARENT_SCOPE)
    set(HDF5_LIB_DIR ${HDF5_LIB_DIR} PARENT_SCOPE)
    set(HDF5_LIBS ${HDF5_LIBS} PARENT_SCOPE)
endfunction()

# Setup HDF5
setup_hdf5_for_platform()

# Setup License Verification (optional)
# mbedTLS is built from source via FetchContent to ensure static linking with
# -fPIC on all platforms, avoiding runtime shared library dependencies.
if(ENABLE_LICENSE)
    message(STATUS "License verification enabled - building mbedTLS from source")

    include(FetchContent)

    # Disable mbedTLS components we don't need
    set(ENABLE_TESTING OFF CACHE BOOL "Disable mbedTLS tests" FORCE)
    set(ENABLE_PROGRAMS OFF CACHE BOOL "Disable mbedTLS programs" FORCE)
    set(GEN_FILES OFF CACHE BOOL "Disable mbedTLS file generation" FORCE)

    # Build only static mbedTLS libraries
    set(USE_STATIC_MBEDTLS_LIBRARY ON CACHE BOOL "Build static mbedTLS" FORCE)
    set(USE_SHARED_MBEDTLS_LIBRARY OFF CACHE BOOL "Don't build shared mbedTLS" FORCE)

    # Ensure PIC is enabled (required for linking static libs into shared lib)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)

    FetchContent_Declare(
        mbedtls
        URL https://github.com/Mbed-TLS/mbedtls/releases/download/mbedtls-4.1.0/mbedtls-4.1.0.tar.bz2
    )
    FetchContent_MakeAvailable(mbedtls)

    add_compile_definitions(RADARSIMCPP_ENABLE_LICENSE)
    message(STATUS "mbedTLS will be built from source with static linking")
endif()

# Collect source files
if(GPU_BUILD)
    file(GLOB SOURCES "./src/*.cu" "./src/*.cpp" "./src/radarsimcpp/src/*.cu" "./src/radarsimcpp/src/*.cpp")
else()
    file(GLOB SOURCES "./src/*.cpp" "./src/radarsimcpp/src/*.cpp")
endif()

# Build library
add_library(radarsimc SHARED ${SOURCES})

# Set target HDF5 include directories
target_include_directories(radarsimc PUBLIC ${HDF5_INCLUDE_DIR})

# Link mbedTLS if license verification is enabled
if(ENABLE_LICENSE)
    target_link_libraries(radarsimc PUBLIC mbedtls mbedx509 tfpsacrypto)

    # On Windows, mbedTLS entropy functions require bcrypt library
    if(WIN32)
        target_link_libraries(radarsimc PUBLIC bcrypt)
        # Suppress MSVCRTD conflict warning when mixing debug/release libs
        target_link_options(radarsimc PUBLIC /NODEFAULTLIB:MSVCRTD)
    endif()

    message(STATUS "License verification linked with mbedTLS (static)")
endif()

# Link HDF5 libraries - use full paths for Linux, link directories for other platforms
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(radarsimc PUBLIC ${HDF5_LIBS})

    # Use whole-archive to force inclusion of all symbols from static libraries
    set(HDF5_LINUX_LIB_DIR "${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5-lib-build/libs/lib_linux_x86_64")
    target_link_options(radarsimc PRIVATE
        "LINKER:--whole-archive"
        "LINKER:${HDF5_LINUX_LIB_DIR}/libhdf5.a"
        "LINKER:${HDF5_LINUX_LIB_DIR}/libhdf5_cpp.a"
        "LINKER:${HDF5_LINUX_LIB_DIR}/libhdf5_hl.a"
        "LINKER:${HDF5_LINUX_LIB_DIR}/libhdf5_hl_cpp.a"
        "LINKER:--no-whole-archive"
    )
else()
    target_link_directories(radarsimc PUBLIC ${HDF5_LIB_DIR})
    target_link_libraries(radarsimc PUBLIC ${HDF5_LIBS})
endif()

if(GPU_BUILD)
    message(STATUS "Building for GPU with CUDA support")

    # Enable CUDA language
    enable_language(CUDA)

    # Set CUDA properties
    set_target_properties(radarsimc PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_ARCHITECTURES "75;80;86;87;89;90;100;103;110;120;121")

    # Mark source files as CUDA
    set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CUDA)

    # Add CUDA includes path
    target_include_directories(radarsimc PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

    # Add CUDA-specific compile definitions
    target_compile_definitions(radarsimc PRIVATE _CUDA_)

    # Handle MSVC-specific linking
    if(MSVC)
        # Use standard conforming preprocessor to avoid CUDA compilation warnings
        target_compile_options(radarsimc PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>)

        # Use proper linker flags for CUDA builds to avoid runtime library conflicts
        set_target_properties(radarsimc PROPERTIES
            MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
            LINK_FLAGS "/NODEFAULTLIB:LIBCMT")
    endif()

else()
    message(STATUS "Building for CPU with OpenMP support")

    # Setup OpenMP for macOS if needed
    setup_openmp_macos()

    # Find OpenMP - handle macOS case where OpenMP might not be available
    find_package(OpenMP)

    if(OpenMP_CXX_FOUND)
        message(STATUS "OpenMP found and will be used for parallelization")
        target_link_libraries(radarsimc PRIVATE OpenMP::OpenMP_CXX)

        # Set compiler-specific optimization flags
        if(MSVC)
            target_compile_options(radarsimc PRIVATE /openmp)
        elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            target_compile_options(radarsimc PRIVATE -fopenmp -O3)
        endif()
    else()
        message(WARNING "OpenMP not found. Building without OpenMP support.")
        message(WARNING "For macOS: Consider installing OpenMP via Homebrew:")
        message(WARNING "  brew install libomp")
        message(WARNING "  export OpenMP_ROOT=$(brew --prefix libomp)")
        message(WARNING "  Then re-run cmake")

        # Set optimization flags without OpenMP
        if(MSVC)
            target_compile_options(radarsimc PRIVATE /O2)
        elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            target_compile_options(radarsimc PRIVATE -O3)
        endif()
    endif()
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
            # Use standard conforming preprocessor to avoid CUDA compilation warnings
            target_compile_options(radarsimc_test PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>)

            # Use proper linker flags for CUDA test builds to avoid runtime library conflicts
            set_target_properties(radarsimc_test PROPERTIES
                MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
                LINK_FLAGS "/NODEFAULTLIB:LIBCMT")
        endif()
    endif()

    # Enable testing and add test
    enable_testing()
    add_test(NAME radarsimc_test COMMAND radarsimc_test)
endif()
