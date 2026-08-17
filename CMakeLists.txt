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

# Prebuilt dependencies
#
# HDF5 and mbedTLS are never compiled here. They are built once per release in
# the radarsimx-deps repository and consumed as static libraries, resolved by
# setup_prebuilt_dep() from radarsimcpp's cmake/PrebuiltDeps.cmake. That module
# owns everything platform-specific about locating them - the platform triplet,
# the .a/.lib naming, and the deps-submodule-or-download decision - so this file
# only has to say which libraries it wants.
#
# See src/radarsimcpp/cmake/deps_manifest.cmake for the pinned versions and
# checksums, and src/radarsimcpp/deps/README.md for how they are produced.
include(PrebuiltDeps)

# shlwapi supplies the path helpers HDF5's Windows VFD calls into.
set(HDF5_SYSTEM_LIBS "")
if(WIN32)
    set(HDF5_SYSTEM_LIBS shlwapi)
endif()

setup_prebuilt_dep(hdf5
    LIBS hdf5 hdf5_cpp hdf5_hl hdf5_hl_cpp
    SYSTEM_LIBS ${HDF5_SYSTEM_LIBS}
    TARGET radarsimx::hdf5
)

# Setup License Verification (optional)
if(ENABLE_LICENSE)
    message(STATUS "License verification enabled - using prebuilt mbedTLS")

    # bcrypt backs mbedTLS's entropy source on Windows.
    set(MBEDTLS_SYSTEM_LIBS "")
    if(WIN32)
        set(MBEDTLS_SYSTEM_LIBS bcrypt)
    endif()

    # In dependency order: mbedtls needs mbedx509, which needs tfpsacrypto.
    setup_prebuilt_dep(mbedtls
        LIBS mbedtls mbedx509 tfpsacrypto
        SYSTEM_LIBS ${MBEDTLS_SYSTEM_LIBS}
        TARGET radarsimx::mbedtls
    )

    add_compile_definitions(RADARSIMCPP_ENABLE_LICENSE)
endif()

# Collect source files
if(GPU_BUILD)
    file(GLOB SOURCES "./src/*.cu" "./src/*.cpp" "./src/radarsimcpp/src/*.cu" "./src/radarsimcpp/src/*.cpp")
else()
    file(GLOB SOURCES "./src/*.cpp" "./src/radarsimcpp/src/*.cpp")
endif()

# Build library
add_library(radarsimc SHARED ${SOURCES})

# Export only what radarsim.h marks EXPORTED.
#
# ELF resolves a shared library's own calls through the PLT against the entire
# process, so every symbol this library exports is one another module can
# interpose. Statically linking mbedTLS does not prevent that: with default
# visibility its symbols land in the dynamic symbol table, and a process that
# also loads a system libmbedcrypto silently redirects our license verification
# into it. Different mbedTLS versions disagree on enough details to reject a
# valid license. The same hazard applies to HDF5, which --whole-archive below
# pulls in wholesale.
#
# Windows keeps CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS; its imports are resolved per
# module, so it has no equivalent exposure.
set_target_properties(radarsimc PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    CUDA_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)

# Visibility flags only reach sources this project compiles. mbedTLS and HDF5
# arrive as prebuilt static archives, so localize their symbols at link time.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(radarsimc PRIVATE "LINKER:--exclude-libs,ALL")
endif()

# Link the prebuilt dependencies. Both imported targets carry their own include
# directories and absolute library paths, so there is no link_directories search
# to get wrong.
target_link_libraries(radarsimc PUBLIC radarsimx::hdf5)

# /NODEFAULTLIB entries accumulated below and applied once. LINK_FLAGS is a
# single string property, so setting it twice drops the earlier value - which is
# how a GPU build with ENABLE_LICENSE used to lose the MSVCRTD exclusion.
set(RADARSIMC_LINK_FLAGS "")

# Link mbedTLS if license verification is enabled
if(ENABLE_LICENSE)
    target_link_libraries(radarsimc PUBLIC radarsimx::mbedtls)

    if(WIN32)
        # The prebuilt libraries are compiled against the release CRT (/MD) - see
        # deps/cmake/baseline.cmake - so a Debug build pulls in both MSVCRT and
        # MSVCRTD. Drop the debug one.
        list(APPEND RADARSIMC_LINK_FLAGS "/NODEFAULTLIB:MSVCRTD")
    endif()

    message(STATUS "License verification linked with mbedTLS (static)")
endif()

# radarsimc is a shared library and its consumers call HDF5 directly, so the
# HDF5 symbols have to survive into the .so even though nothing in radarsimc's
# own objects references them. --whole-archive is GNU ld specific; MSVC and ld64
# keep them via CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS and two-level namespaces.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(HDF5_WHOLE_ARCHIVE "LINKER:--whole-archive")
    foreach(hdf5_lib IN LISTS HDF5_LIBRARIES)
        list(APPEND HDF5_WHOLE_ARCHIVE "LINKER:${hdf5_lib}")
    endforeach()
    list(APPEND HDF5_WHOLE_ARCHIVE "LINKER:--no-whole-archive")

    target_link_options(radarsimc PRIVATE ${HDF5_WHOLE_ARCHIVE})
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
        set_target_properties(radarsimc PROPERTIES MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
        list(APPEND RADARSIMC_LINK_FLAGS "/NODEFAULTLIB:LIBCMT")
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

# Apply the accumulated /NODEFAULTLIB flags. LINK_FLAGS is used instead of
# target_link_options so the flags land in the VS linker section only and are
# not passed to cl.exe.
if(RADARSIMC_LINK_FLAGS)
    list(JOIN RADARSIMC_LINK_FLAGS " " RADARSIMC_LINK_FLAGS_STR)
    set_target_properties(radarsimc PROPERTIES LINK_FLAGS "${RADARSIMC_LINK_FLAGS_STR}")
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

    # Collect test sources. CONFIGURE_DEPENDS re-runs the glob at build time so
    # a newly added tests/*.cpp is picked up without a manual re-configure.
    file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "tests/*.cpp")

    # Create test executable
    add_executable(radarsimc_test ${TEST_SOURCES})

    # tests/test_helpers.hpp is included as "test_helpers.hpp"
    target_include_directories(radarsimc_test PRIVATE "${CMAKE_SOURCE_DIR}/tests")

    # Link with gtest and main library. gtest (not gtest_main) - the runner
    # provides its own main() in tests/test_main.cpp.
    target_link_libraries(radarsimc_test
        PRIVATE
        gtest
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

    # Enable testing and register every TEST() as its own CTest case, so a
    # failure names the individual test instead of the whole binary.
    # PRE_TEST defers discovery to `ctest` time, which keeps the build from
    # depending on the test executable being runnable (relevant for GPU builds
    # and cross-compiled trees).
    enable_testing()
    include(GoogleTest)
    gtest_discover_tests(radarsimc_test
        DISCOVERY_MODE PRE_TEST
        DISCOVERY_TIMEOUT 60
        PROPERTIES TIMEOUT 300
    )
endif()
