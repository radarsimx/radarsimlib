cmake_minimum_required(VERSION 3.14)

# ## Config for radarsimc
project(radarsimc LANGUAGES CXX)

# ## force GPU build
# set(GPU_BUILD OFF)

# ## bypass CUDA compiler check
# set(CMAKE_CUDA_COMPILER_WORKS 1)
if(GPU_BUILD)
    # ## add _CUDA_ micro to the compiler
    add_definitions(-D_CUDA_)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") # Windows
    include_directories("./src/includes"
        "./src/radarsimcpp/includes"
        "./src/radarsimcpp/hdf5/lib_win_x86_64/include")
    link_directories(${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5/lib_win_x86_64/lib)
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
        "./src/radarsimcpp/hdf5/lib_linux_x86_64/include")
    link_directories(${CMAKE_SOURCE_DIR}/src/radarsimcpp/hdf5/lib_linux_x86_64/lib)
    set(_hdf5_libs
        hdf5
        hdf5_cpp
        hdf5_hl
        hdf5_hl_cpp
    )
endif()

if(GPU_BUILD)
    enable_language(CUDA)

    # ## add source files, ".cpp" and ".cu"
    file(GLOB SOURCES "./src/*.cu" "./src/*.cpp" "./src/radarsimcpp/src/*.cu" "./src/radarsimcpp/src/*.cpp")

    # ## use nvcc to compile all the *.cpp files
    set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CUDA)

    # ## build executable
    add_library(radarsimc SHARED ${SOURCES})

    # ## set CUDA building properties
    set_target_properties(radarsimc PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_ARCHITECTURES all-major)

    # ## add CUDA includes path
    include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

    # ## add _CUDA_ micro to the compiler
    target_compile_definitions(radarsimc PRIVATE _CUDA_ _CUDATEST_)

    # ## for MSVC, remove default libs
    if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        set_target_properties(radarsimc PROPERTIES LINK_FLAGS "/NODEFAULTLIB:\"LIBCMT\"")
    endif()

    target_link_libraries(radarsimc PUBLIC ${_hdf5_libs})
else()
    # ## add source files, ".cpp"
    file(GLOB SOURCES "./src/*.cpp" "./src/radarsimcpp/src/*.cpp")

    # ## build executable
    add_library(radarsimc SHARED ${SOURCES})
    target_link_libraries(radarsimc PUBLIC ${_hdf5_libs})

    # add_executable(radarsimc ${SOURCES})
endif()
