cmake_minimum_required(VERSION 3.8)

# ## Config for radarsim
project(radarsim LANGUAGES CXX)

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

# ## header files in "./includes" and "../includes" folders
include_directories("./src/includes" "./src/radarsimcpp/includes")

if(GPU_BUILD)
    enable_language(CUDA)

    # ## add source files, ".cpp" and ".cu"
    file(GLOB SOURCES "./src/*.cu" "./src/*.cpp" "./src/radarsimcpp/src/*.cu" "./src/radarsimcpp/src/*.cpp")

    # ## use nvcc to compile all the *.cpp files
    set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CUDA)

    # ## build executable
    add_library(radarsim SHARED ${SOURCES})

    # ## set CUDA building properties
    set_target_properties(radarsim PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_ARCHITECTURES "50;61;86")

    # ## add CUDA includes path
    include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

    # ## add _CUDA_ micro to the compiler
    target_compile_definitions(radarsim PRIVATE _CUDA_ _CUDATEST_)

    # ## for MSVC, remove default libs
    if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        # target_compile_options(radarsim PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
        # --generate-line-info
        # --use_fast_math
        # --relocatable-device-code=true
        # >)
        set_target_properties(radarsim PROPERTIES LINK_FLAGS "/NODEFAULTLIB:\"LIBCMT\"")
    endif()
else()
    # ## add source files, ".cpp"
    file(GLOB SOURCES "./src/*.cpp" "./src/radarsimcpp/src/*.cpp")

    # ## build executable
    add_library(radarsim SHARED ${SOURCES})
    # add_executable(radarsim ${SOURCES})

    # require OpenMP for CPU build
    # find_package(OpenMP REQUIRED)

    # if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC") # MSVC
    #     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp")
    # elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU") # gcc/g++
    #     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
    # else() # other compiler (not tested)
    #     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
    # endif()

    # link with OpenMP library
    # target_link_libraries(radarsim PRIVATE OpenMP::OpenMP_CXX)
endif()
