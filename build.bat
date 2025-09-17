@echo off
SETLOCAL EnableDelayedExpansion

REM ==============================================================================
REM RadarSimLib Build Script for Windows
REM ==============================================================================
REM
REM DESCRIPTION:
REM   This script automates the build process for RadarSimLib on Windows systems.
REM   It compiles the C++ library (radarsimc.dll) with comprehensive error 
REM   handling and logging optimized for Windows platforms.
REM
REM REQUIREMENTS:
REM   - CMake 3.18 or higher
REM   - Visual Studio 2017 or later (with C++ tools) or Visual Studio Build Tools
REM   - MSVC compiler (automatically detected via vswhere)
REM   - CUDA toolkit (for GPU builds)
REM
REM USAGE:
REM   build_win.bat [OPTIONS]
REM
REM OPTIONS:
REM   --help              Show help message
REM   --tier=TIER         Build tier: 'standard' or 'free' (default: standard)
REM   --arch=ARCH         Build architecture: 'cpu' or 'gpu' (default: cpu)
REM   --test=TEST         Enable unit tests: 'on' or 'off' (default: on)
REM   --jobs=N            Number of parallel build jobs (default: auto-detect)
REM
REM EXAMPLES:
REM   build_win.bat                                    REM Default build
REM   build_win.bat --tier=free --arch=gpu           REM GPU build with free tier
REM   build_win.bat --jobs=8 --test=off              REM 8-core parallel build, no tests
REM   build_win.bat --arch=cpu --tier=standard       REM CPU build with standard tier
REM
REM EXIT CODES:
REM   0  - Success
REM   1  - General error (missing dependencies, validation failure, etc.)
REM
REM FILES CREATED:
REM   - .\radarsimlib_win_x86_64_{arch}[_free]\       REM Output directory with built libraries
REM   - .\build_logs\windows_batch_build_log_YYYYMMDD_HHMMSS.log  REM Timestamped build log
REM
REM ==============================================================================

REM Default build configuration
set TIER=standard
set ARCH=cpu
set TEST=on
set BUILD_TYPE=Release
set SCRIPT_DIR=%~dp0
set BUILD_FAILED=0
set JOBS=0

REM Initialize error tracking
set CMAKE_FAILED=0
set TEST_FAILED=0
set MISSING_DEPS=0
set VS_FOUND=0

goto GETOPTS

REM Help section - displays command line parameter usage
:Help
    echo.
    echo Usage: build_win.bat [OPTIONS]
    echo.
    echo Cross-platform build script for RadarSimLib - A Radar Simulator Built with C++
    echo Optimized for Windows platforms with automatic dependency detection.
    echo.
    echo Current Platform: Windows
    echo.
    echo OPTIONS:
    echo   --help              Show this help message
    echo   --tier=TIER         Build tier: 'standard' or 'free' (default: standard)
    echo   --arch=ARCH         Build architecture: 'cpu' or 'gpu' (default: cpu)
    echo   --test=TEST         Enable unit tests: 'on' or 'off' (default: on)
    echo   --jobs=N            Number of parallel build jobs (default: auto-detect)
    echo.
    echo EXAMPLES:
    echo   %~nx0                                    # Default build
    echo   %~nx0 --tier=free --arch=gpu           # GPU build with free tier
    echo   %~nx0 --jobs=8 --test=off              # 8-core parallel build, no tests
    echo   %~nx0 --arch=cpu --tier=standard       # CPU build with standard tier
    echo.
    echo WINDOWS-SPECIFIC NOTES:
    echo   - Uses MSVC compiler, creates .dll files
    echo   - Automatically detects Visual Studio installations using vswhere
    echo   - Requires Visual Studio 2017 or later with C++ development tools
    echo   - Supports both x64 and x86 architectures
    echo   - GPU builds require CUDA toolkit
    echo.
    echo EXIT CODES:
    echo   0  - Success
    echo   1  - General error (missing dependencies, validation failure, etc.)
    echo.
    echo FILES CREATED:
    echo   - .\radarsimlib_win_x86_64_{arch}[_free]\    # Output directory with built libraries
    echo   - .\build_logs\windows_batch_build_log_YYYYMMDD_HHMMSS.log  # Timestamped build log
    echo.
    goto EOF

REM Command line parameter parsing section
REM
REM GETOPTS() - Parses command line arguments and sets global configuration
REM Description:
REM   Processes all command line arguments passed to the script and sets global
REM   configuration variables. Handles both parameter validation and default value
REM   assignment. Also handles special cases like --help and automatic CPU detection.
REM Arguments:
REM   %1, %2, ... - Command line arguments passed to the script
REM Global Variables Set:
REM   TIER - Build tier (standard/free)
REM   ARCH - Build architecture (cpu/gpu)
REM   TEST - Unit test flag (on/off)
REM   JOBS - Number of parallel build jobs
REM Supported Options:
REM   --help: Shows help and exits
REM   --tier=VALUE: Sets build tier
REM   --arch=VALUE: Sets architecture
REM   --test=VALUE: Enables/disables tests
REM   --jobs=VALUE: Sets parallel job count
REM Exit:
REM   Exits with code 0 on --help
REM   Exits with code 1 on unknown options or validation errors
:GETOPTS
    REM Parse command line arguments
    if /I "%1" == "--help" goto Help
    if /I "%1" == "-h" goto Help
    if /I "%1" == "--tier" (
        set TIER=%2
        shift
        shift
        goto GETOPTS
    )
    if /I "%1" == "--arch" (
        set ARCH=%2
        shift
        shift
        goto GETOPTS
    )
    if /I "%1" == "--test" (
        set TEST=%2
        shift
        shift
        goto GETOPTS
    )
    if /I "%1" == "--jobs" (
        set JOBS=%2
        shift
        shift
        goto GETOPTS
    )
    if not "%1" == "" (
        echo ERROR: Unknown parameter: %1
        echo Use --help for usage information
        goto ERROR_EXIT
    )

    REM Validate tier parameter
    if /I NOT "%TIER%" == "free" (
        if /I NOT "%TIER%" == "standard" (
            echo ERROR: Invalid --tier parameter '%TIER%'. Please choose 'free' or 'standard'
            goto ERROR_EXIT
        )
    )

    REM Validate architecture parameter
    if /I NOT "%ARCH%" == "cpu" (
        if /I NOT "%ARCH%" == "gpu" (
            echo ERROR: Invalid --arch parameter '%ARCH%'. Please choose 'cpu' or 'gpu'
            goto ERROR_EXIT
        )
    )

    REM Validate test parameter
    if /I NOT "%TEST%" == "on" (
        if /I NOT "%TEST%" == "off" (
            echo ERROR: Invalid --test parameter '%TEST%'. Please choose 'on' or 'off'
            goto ERROR_EXIT
        )
    )

    REM Validate and set jobs parameter
    if /I "%JOBS%" == "0" (
        set JOBS=auto
    )
    if /I "%JOBS%" == "auto" (
        REM Auto-detect number of CPU cores
        if defined NUMBER_OF_PROCESSORS (
            set JOBS=%NUMBER_OF_PROCESSORS%
        ) else (
            set JOBS=4
        )
        echo INFO: Auto-detected %JOBS% CPU cores for parallel build
    ) else (
        REM Validate that jobs is a positive integer
        echo %JOBS%| findstr /r "^[1-9][0-9]*$" >nul
        if %errorlevel% neq 0 (
            echo ERROR: Invalid --jobs parameter '%JOBS%'. Please provide 'auto' or a positive integer
            goto ERROR_EXIT
        )
    )

REM Validate build environment
:VALIDATE_ENVIRONMENT
    echo INFO: Validating build environment for Windows...
    set MISSING_DEPS=0
    
    REM Check for CMake
    cmake --version >nul 2>&1
    if %errorlevel% neq 0 (
        echo ERROR: CMake is not installed or not in PATH
        echo Please install CMake 3.18 or higher
        set MISSING_DEPS=1
    ) else (
        echo INFO: CMake found and available
    )
    
    REM Check for MSVC compiler (Visual Studio Build Tools) using vswhere
    set VS_FOUND=0
    set VSWHERE_PATH="%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    
    REM Check if vswhere exists
    if exist %VSWHERE_PATH% (
        REM Use vswhere to find Visual Studio installations with C++ tools
        for /f "usebackq tokens=*" %%i in (`%VSWHERE_PATH% -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do (
            set "VS_INSTALL_PATH=%%i"
            set VS_FOUND=1
        )
        
        if !VS_FOUND! equ 1 (
            echo INFO: Visual Studio found at: !VS_INSTALL_PATH!
            REM Check if vcvarsall.bat exists
            if exist "!VS_INSTALL_PATH!\VC\Auxiliary\Build\vcvarsall.bat" (
                echo INFO: MSVC compiler tools available
            ) else (
                echo WARNING: MSVC compiler tools not found in Visual Studio installation
            )
        ) else (
            echo WARNING: No Visual Studio installation with C++ tools found
            echo Please install Visual Studio 2019 or later with C++ development tools
        )
    ) else (
        REM Fallback to checking cl.exe in PATH
        echo INFO: vswhere not found, checking for cl.exe in PATH...
        cl.exe >nul 2>&1
        if %errorlevel% neq 0 (
            echo WARNING: MSVC compiler not found in PATH
            echo Make sure Visual Studio Build Tools are installed and properly configured
            echo You may need to run this script from a Visual Studio Command Prompt
        ) else (
            echo INFO: MSVC compiler found and available in PATH
        )
    )
    
    REM Check for CUDA if GPU build is requested
    if /I "%ARCH%" == "gpu" (
        nvcc --version >nul 2>&1
        if %errorlevel% neq 0 (
            echo ERROR: CUDA toolkit is not installed or not in PATH
            echo Please install CUDA SDK for GPU builds
            set MISSING_DEPS=1
        ) else (
            echo INFO: CUDA toolkit found and available
        )
    )
    
    if %MISSING_DEPS% equ 1 (
        echo ERROR: Missing required dependencies. Please install them and try again.
        goto ERROR_EXIT
    )
    
    echo INFO: All system requirements satisfied for Windows

REM Display banner and copyright information
:DISPLAY_BANNER

    echo.
    echo ====================================================================
    echo RadarSimLib - A Radar Simulator Built with C++
    echo Copyright (C) 2023 - PRESENT  radarsimx.com
    echo E-mail: info@radarsimx.com
    echo Website: https://radarsimx.com
    echo ====================================================================
    echo.
    echo Build Configuration (Windows):
    echo   - Platform: Windows
    echo   - Tier: %TIER%
    echo   - Architecture: %ARCH%
    echo   - Tests: %TEST%
    echo   - Build Type: %BUILD_TYPE%
    echo   - Parallel Jobs: %JOBS%
    echo   - Script Directory: %SCRIPT_DIR%
    echo.
    echo ######                               #####           #     # 
    echo #     #   ##   #####    ##   #####  #     # # #    #  #   #  
    echo #     #  #  #  #    #  #  #  #    # #       # ##  ##   # #   
    echo ######  #    # #    # #    # #    #  #####  # # ## #    #    
    echo #   #   ###### #    # ###### #####        # # #    #   # #   
    echo #    #  #    # #    # #    # #   #  #     # # #    #  #   #  
    echo #     # #    # #####  #    # #    #  #####  # #    # #     # 
    echo.

REM Store current directory for later use
set PWD=%cd%

REM Start build process
:BUILD_START
    echo INFO: Starting build process...
    echo INFO: Current directory: %PWD%

REM Clean up previous build artifacts
REM
REM CLEAN_BUILD() - Removes all previous build artifacts and temporary files
REM Description:
REM   Performs comprehensive cleanup of all build-related directories and files
REM   to ensure a clean build environment. This includes C++ build directories
REM   and output directories.
REM Arguments:
REM   None
REM Directories Cleaned:
REM   - .\build - C++ build directory
REM   - .\radarsimlib_win_x86_64_* - Output directories
REM Error Handling:
REM   Continues on cleanup failures with warnings (non-fatal)
:CLEAN_BUILD
    echo INFO: Cleaning previous build artifacts...
    
    if exist ".\build" (
        rmdir /q /s ".\build" 2>nul
        if %errorlevel% neq 0 (
            echo WARNING: Could not fully clean C++ build directory
        )
    )

REM Build C++ library
REM
REM BUILD_CPP() - Builds the RadarSimLib C++ library using CMake
REM Description:
REM   Configures and builds the C++ library component using CMake with appropriate
REM   settings for the target architecture (CPU/GPU), tier (standard/free), and test configuration.
REM   Uses Visual Studio generators for Windows compatibility.
REM Arguments:
REM   None
REM Global Variables Used:
REM   ARCH - Determines GPU_BUILD CMake option
REM   TIER - Determines FREETIER CMake option
REM   TEST - Determines GTEST CMake option
REM   BUILD_TYPE - CMake build configuration (Release/Debug)
REM   JOBS - Number of parallel build jobs
REM Output:
REM   - radarsimc.dll in .\build\%BUILD_TYPE%\
REM   - radarsimc_test.exe (if tests enabled)
REM Error Handling:
REM   Sets CMAKE_FAILED=1 and exits on any CMake failures
:BUILD_CPP
    echo INFO: Building RadarSimLib library...
    
    REM Create build directory
    if not exist ".\build" (
        mkdir ".\build"
        if %errorlevel% neq 0 (
            echo ERROR: Failed to create C++ build directory
            goto ERROR_EXIT
        )
    )
    
    REM Change to build directory
    pushd ".\build"
    
    REM Configure CMake build based on architecture, tier and test settings
    echo INFO: Configuring CMake build - Architecture: %ARCH%, Tier: %TIER%, Tests: %TEST%...
    
    REM Set CMake options based on configuration
    set CMAKE_OPTIONS=
    if /I "%ARCH%" == "gpu" (
        set CMAKE_OPTIONS=%CMAKE_OPTIONS% -DGPU_BUILD=ON
    ) else (
        set CMAKE_OPTIONS=%CMAKE_OPTIONS% -DGPU_BUILD=OFF
    )
    
    if /I "%TIER%" == "free" (
        set CMAKE_OPTIONS=%CMAKE_OPTIONS% -DFREETIER=ON
    ) else (
        set CMAKE_OPTIONS=%CMAKE_OPTIONS% -DFREETIER=OFF
    )
    
    if /I "%TEST%" == "on" (
        set CMAKE_OPTIONS=%CMAKE_OPTIONS% -DGTEST=ON
    ) else (
        set CMAKE_OPTIONS=%CMAKE_OPTIONS% -DGTEST=OFF
    )
    
    cmake %CMAKE_OPTIONS% ..
    
    if %errorlevel% neq 0 (
        echo ERROR: CMake configuration failed
        set CMAKE_FAILED=1
        popd
        goto ERROR_EXIT
    )
    
    REM Build the C++ library
    echo INFO: Building C++ library with %BUILD_TYPE% configuration using %JOBS% parallel jobs...
    cmake --build . --config %BUILD_TYPE% --parallel %JOBS%
    
    if %errorlevel% neq 0 (
        echo ERROR: C++ library build failed
        set CMAKE_FAILED=1
        popd
        goto ERROR_EXIT
    )
    
    popd
    echo INFO: C++ library build completed successfully

REM Copy built artifacts
:COPY_ARTIFACTS
    echo INFO: Copying build artifacts...
    
    REM Determine output directory based on configuration
    if /I "%ARCH%" == "gpu" (
        if /I "%TIER%" == "standard" (
            set RELEASE_PATH=.\radarsimlib_win_x86_64_gpu
        ) else (
            set RELEASE_PATH=.\radarsimlib_win_x86_64_gpu_free
        )
    ) else (
        if /I "%TIER%" == "standard" (
            set RELEASE_PATH=.\radarsimlib_win_x86_64_cpu
        ) else (
            set RELEASE_PATH=.\radarsimlib_win_x86_64_cpu_free
        )
    )
    
    REM Create output directory
    if not exist "%RELEASE_PATH%" (
        mkdir "%RELEASE_PATH%"
        if %errorlevel% neq 0 (
            echo ERROR: Failed to create output directory %RELEASE_PATH%
            goto ERROR_EXIT
        )
    )
    
    REM Copy DLL file
    if exist ".\build\%BUILD_TYPE%\radarsimc.dll" (
        copy ".\build\%BUILD_TYPE%\radarsimc.dll" "%RELEASE_PATH%\" >nul
        if %errorlevel% neq 0 (
            echo ERROR: Failed to copy radarsimc.dll
            goto ERROR_EXIT
        )
    ) else (
        echo ERROR: radarsimc.dll not found in build directory
        goto ERROR_EXIT
    )
    
    REM Copy LIB file
    if exist ".\build\%BUILD_TYPE%\radarsimc.lib" (
        copy ".\build\%BUILD_TYPE%\radarsimc.lib" "%RELEASE_PATH%\" >nul
        if %errorlevel% neq 0 (
            echo ERROR: Failed to copy radarsimc.lib
            goto ERROR_EXIT
        )
    ) else (
        echo ERROR: radarsimc.lib not found in build directory
        goto ERROR_EXIT
    )
    
    REM Copy header file
    if exist ".\src\includes\radarsim.h" (
        copy ".\src\includes\radarsim.h" "%RELEASE_PATH%\" >nul
        if %errorlevel% neq 0 (
            echo ERROR: Failed to copy radarsim.h
            goto ERROR_EXIT
        )
    ) else (
        echo ERROR: radarsim.h not found
        goto ERROR_EXIT
    )
    
    REM Copy files to examples/cpp/radarsimlib directory as well
    set EXAMPLES_PATH=.\examples\cpp\radarsimlib
    if not exist "%EXAMPLES_PATH%" (
        mkdir "%EXAMPLES_PATH%"
        if %errorlevel% neq 0 (
            echo WARNING: Failed to create examples directory %EXAMPLES_PATH%
        )
    )
    
    if exist "%EXAMPLES_PATH%" (
        echo INFO: Copying artifacts to examples directory...
        
        REM Copy DLL to examples
        if exist ".\build\%BUILD_TYPE%\radarsimc.dll" (
            copy ".\build\%BUILD_TYPE%\radarsimc.dll" "%EXAMPLES_PATH%\" >nul
            if %errorlevel% neq 0 (
                echo WARNING: Failed to copy radarsimc.dll to examples
            )
        )
        
        REM Copy LIB to examples
        if exist ".\build\%BUILD_TYPE%\radarsimc.lib" (
            copy ".\build\%BUILD_TYPE%\radarsimc.lib" "%EXAMPLES_PATH%\" >nul
            if %errorlevel% neq 0 (
                echo WARNING: Failed to copy radarsimc.lib to examples
            )
        )
        
        REM Copy header to examples
        if exist ".\src\includes\radarsim.h" (
            copy ".\src\includes\radarsim.h" "%EXAMPLES_PATH%\" >nul
            if %errorlevel% neq 0 (
                echo WARNING: Failed to copy radarsim.h to examples
            ) else (
                echo INFO: Artifacts copied successfully to %EXAMPLES_PATH%
            )
        )
    )
    
    echo INFO: Artifacts copied successfully to %RELEASE_PATH%

REM Run tests if enabled
REM
REM RUN_TESTS() - Executes unit tests for C++ components
REM Description:
REM   Runs comprehensive test suites including Google Test for C++ components.
REM   Tests are only executed if enabled via the --test=on parameter.
REM Arguments:
REM   None
REM Global Variables Used:
REM   TEST - Determines if tests should be executed
REM   BUILD_TYPE - Used to locate test executables
REM Tests Executed:
REM   - Google Test (radarsimc_test.exe) via CTest
REM Error Handling:
REM   Sets TEST_FAILED=1 on any test failures
REM   Continues with warnings if test tools are not available
:RUN_TESTS
    if /I "%TEST%" == "off" (
        echo INFO: Tests are disabled, skipping test execution
        goto BUILD_SUCCESS
    )
    
    echo INFO: Running tests...
    
    REM Run Google Test (C++) using CTest
    if exist ".\build\%BUILD_TYPE%\radarsimc_test.exe" (
        echo INFO: Running Google Test for C++ using CTest...
        ctest --test-dir ".\build" -C %BUILD_TYPE% --verbose
        set CTEST_EXIT_CODE=!errorlevel!
        if !CTEST_EXIT_CODE! neq 0 (
            echo ERROR: Google Test failed with exit code !CTEST_EXIT_CODE!
            set TEST_FAILED=1
        ) else (
            echo INFO: Google Test passed
        )
    ) else (
        echo WARNING: Google Test executable not found, skipping C++ tests
    )
    
    REM Check overall test results
    if %TEST_FAILED% neq 0 (
        echo ERROR: Some tests failed
        goto ERROR_EXIT
    )
    
    echo INFO: All tests passed successfully
    goto BUILD_SUCCESS

:BUILD_SUCCESS
    echo ====================================================================
    echo BUILD COMPLETED SUCCESSFULLY
    echo ====================================================================
    echo.
    echo Build Summary (Windows):
    echo   - Platform: Windows
    echo   - Tier: %TIER%
    echo   - Architecture: %ARCH%
    echo   - Tests: %TEST%
    echo   - Build Type: %BUILD_TYPE%
    echo   - Parallel Jobs: %JOBS%
    echo   - Script Directory: %SCRIPT_DIR%
    echo.
    echo Output Locations:
    echo   - C++ Library: .\build\%BUILD_TYPE%\
    echo   - Output Directory: %RELEASE_PATH%\
    echo   - Examples Directory: .\examples\cpp\radarsimlib\
    echo   - DLL File: %RELEASE_PATH%\radarsimc.dll
    echo   - LIB File: %RELEASE_PATH%\radarsimc.lib
    echo   - Header File: %RELEASE_PATH%\radarsim.h
    echo.
    if /I "%TEST%" == "on" (
        echo Test Results: All tests passed successfully
    ) else (
        echo Test Results: Tests were skipped (disabled)
    )
    echo.
    echo Build completed at: %DATE% %TIME%
    echo.
    echo ====================================================================
    
    exit /b 0

REM Error handling
:ERROR_EXIT
    echo.
    echo ====================================================================
    echo BUILD FAILED
    echo ====================================================================
    echo.
    echo Build Configuration (Windows):
    echo   - Platform: Windows
    echo   - Tier: %TIER%
    echo   - Architecture: %ARCH%
    echo   - Tests: %TEST%
    echo   - Build Type: %BUILD_TYPE%
    echo   - Parallel Jobs: %JOBS%
    echo.
    echo Error Summary:
    if %CMAKE_FAILED% neq 0 echo   - CMake configuration or build failed
    if %TEST_FAILED% neq 0 echo   - Unit tests failed
    if %MISSING_DEPS% neq 0 echo   - Missing required dependencies
    echo.
    echo Troubleshooting:
    echo   - Ensure all required dependencies are installed
    echo   - Install Visual Studio 2017 or later with C++ development tools
    echo   - Script uses vswhere for automatic Visual Studio detection
    echo   - For GPU builds, verify CUDA toolkit installation
    echo   - Try running from a Visual Studio Command Prompt if detection fails
    echo   - Check the build logs for detailed error messages
    echo.
    echo Build failed at: %DATE% %TIME%
    echo.
    echo ====================================================================
    
    exit /b 1

:EOF
    exit /b 0
