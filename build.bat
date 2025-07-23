@echo off
SETLOCAL EnableDelayedExpansion

REM ==============================================================================
REM RadarSimLib Build Script for Windows
REM ==============================================================================
REM
REM DESCRIPTION:
REM   This script automates the build process for RadarSimLib on Windows systems.
REM   It compiles the C++ library (radarsimc.dll) using CMake with comprehensive
REM   error handling, dependency validation, and testing support optimized for
REM   Windows platforms.
REM
REM REQUIREMENTS:
REM   - CMake 3.14 or higher
REM   - Visual Studio 2017 or later (with C++ tools) or Visual Studio Build Tools
REM   - MSVC compiler (automatically detected via vswhere)
REM   - CUDA toolkit (for GPU builds)
REM   - Google Test (automatically downloaded when tests enabled)
REM
REM USAGE:
REM   build_win.bat [OPTIONS]
REM
REM OPTIONS:
REM   --help              Show help message
REM   --tier=TIER         Build tier: 'standard' or 'free' (default: standard)
REM   --arch=ARCH         Build architecture: 'cpu' or 'gpu' (default: cpu)
REM   --test=TEST         Enable unit tests: 'on' or 'off' (default: off)
REM   --jobs=N            Number of parallel build jobs (default: auto-detect)
REM
REM EXAMPLES:
REM   build_win.bat                                    REM Default build
REM   build_win.bat --tier=free --arch=gpu           REM GPU build with free tier
REM   build_win.bat --jobs=8 --test=on               REM 8-core parallel build with tests
REM   build_win.bat --arch=cpu --tier=standard       REM CPU build with standard tier
REM
REM EXIT CODES:
REM   0  - Success
REM   1  - General error (missing dependencies, validation failure, etc.)
REM
REM FILES CREATED:
REM   - .\radarsimlib_win_x86_64_[arch][_tier]\       REM Output directory with built library
REM   - .\build\Release\radarsimc.dll                 REM Main library file
REM   - .\build\Release\radarsim_c_test.exe           REM Test executable (if tests enabled)
REM
REM ==============================================================================

REM Default build configuration
set TIER=standard
set ARCH=cpu
set TEST=off
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
    echo Cross-platform build script for RadarSimLib - A Radar Simulation C++ Library
    echo Optimized for Windows platforms with automatic dependency detection.
    echo.
    echo Current Platform: Windows
    echo.
    echo OPTIONS:
    echo   --help              Show this help message
    echo   --tier=TIER         Build tier: 'standard' or 'free' (default: standard)
    echo   --arch=ARCH         Build architecture: 'cpu' or 'gpu' (default: cpu)
    echo   --test=TEST         Enable unit tests: 'on' or 'off' (default: off)
    echo   --jobs=N            Number of parallel build jobs (default: auto-detect)
    echo.
    echo EXAMPLES:
    echo   %~nx0                                    # Default build
    echo   %~nx0 --tier=free --arch=gpu           # GPU build with free tier
    echo   %~nx0 --jobs=8 --test=on               # 8-core parallel build with tests
    echo   %~nx0 --arch=cpu --tier=standard       # CPU build with standard tier
    echo.
    echo WINDOWS-SPECIFIC NOTES:
    echo   - Uses MSVC compiler, creates .dll files
    echo   - Automatically detects Visual Studio installations using vswhere
    echo   - Requires Visual Studio 2017 or later with C++ development tools
    echo   - Supports both x64 and x86 architectures
    echo   - GPU builds require CUDA toolkit
    echo   - Tests use Google Test framework (automatically downloaded)
    echo.
    echo EXIT CODES:
    echo   0  - Success
    echo   1  - General error (missing dependencies, validation failure, etc.)
    echo.
    echo FILES CREATED:
    echo   - .\radarsimlib_win_x86_64_[arch][_tier]\   # Output directory with built library
    echo   - .\build\Release\radarsimc.dll             # Main library file
    echo   - .\build\Release\radarsim_c_test.exe       # Test executable (if tests enabled)
    echo.
    goto EOF

REM Command line parameter parsing section
:GETOPTS
    REM Parse command line arguments using the fixed parameter parsing
    if /I "%1" == "--help" goto Help
    if /I "%1" == "-h" goto Help
    if /I "%1" == "--tier" (
        set TIER=%~2
        shift
        shift
        goto GETOPTS
    )
    if /I "%1" == "--arch" (
        set ARCH=%~2
        shift
        shift
        goto GETOPTS
    )
    if /I "%1" == "--test" (
        set TEST=%~2
        shift
        shift
        goto GETOPTS
    )
    if /I "%1" == "--jobs" (
        set JOBS=%~2
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
        echo Please install CMake 3.14 or higher
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
    echo RadarSimLib - A Radar Simulation C++ Library
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
:CLEAN_BUILD
    echo INFO: Cleaning previous build artifacts...
    
    if exist ".\build" (
        rmdir /q /s ".\build" 2>nul
        if %errorlevel% neq 0 (
            echo WARNING: Could not fully clean build directory
        )
    )

REM Build C++ library
:BUILD_CPP
    echo INFO: Building RadarSimLib C++ library...
    
    REM Create build directory
    if not exist ".\build" (
        mkdir ".\build"
        if %errorlevel% neq 0 (
            echo ERROR: Failed to create build directory
            goto ERROR_EXIT
        )
    )
    
    REM Change to build directory
    pushd ".\build"
    
    REM Configure CMake build based on architecture, tier, and test settings
    echo INFO: Configuring CMake build - Architecture: %ARCH%, Tier: %TIER%, Tests: %TEST%...
    
    set CMAKE_OPTIONS=-DCMAKE_BUILD_TYPE=%BUILD_TYPE%
    
    if /I "%TIER%" == "free" (
        set CMAKE_OPTIONS=!CMAKE_OPTIONS! -DFREETIER=ON
        echo INFO: Configuring for FREE tier build
    ) else (
        set CMAKE_OPTIONS=!CMAKE_OPTIONS! -DFREETIER=OFF
        echo INFO: Configuring for STANDARD tier build
    )
    
    if /I "%ARCH%" == "gpu" (
        set CMAKE_OPTIONS=!CMAKE_OPTIONS! -DGPU_BUILD=ON
        echo INFO: Configuring for GPU build
    ) else (
        set CMAKE_OPTIONS=!CMAKE_OPTIONS! -DGPU_BUILD=OFF
        echo INFO: Configuring for CPU build
    )
    
    if /I "%TEST%" == "on" (
        set CMAKE_OPTIONS=!CMAKE_OPTIONS! -DGTEST=ON
        echo INFO: Configuring with Google Test enabled
    ) else (
        set CMAKE_OPTIONS=!CMAKE_OPTIONS! -DGTEST=OFF
    )
    
    echo INFO: CMake options: !CMAKE_OPTIONS!
    cmake !CMAKE_OPTIONS! ..
    
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

REM Run tests if enabled
:RUN_TESTS
    if /I "%TEST%" == "off" (
        echo INFO: Tests are disabled, skipping test execution
        goto COPY_ARTIFACTS
    )
    
    echo INFO: Running tests...
    
    REM Run tests using CTest
    pushd ".\build"
    ctest -C %BUILD_TYPE% --verbose --output-on-failure
    if %errorlevel% neq 0 (
        echo ERROR: Tests failed
        set TEST_FAILED=1
        popd
        goto ERROR_EXIT
    ) else (
        echo INFO: All tests passed successfully
    )
    popd

REM Copy built artifacts
:COPY_ARTIFACTS
    echo INFO: Copying build artifacts...
    
    REM Determine release path based on architecture and tier
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
    
    REM Clean and create release directory
    if exist "%RELEASE_PATH%" (
        rmdir /q /s "%RELEASE_PATH%" 2>nul
    )
    mkdir "%RELEASE_PATH%"
    if %errorlevel% neq 0 (
        echo ERROR: Failed to create release directory: %RELEASE_PATH%
        goto ERROR_EXIT
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
    
    REM Copy additional library files if they exist
    if exist ".\build\%BUILD_TYPE%\radarsimc.lib" (
        copy ".\build\%BUILD_TYPE%\radarsimc.lib" "%RELEASE_PATH%\" >nul
    )
    if exist ".\build\%BUILD_TYPE%\radarsimc.exp" (
        copy ".\build\%BUILD_TYPE%\radarsimc.exp" "%RELEASE_PATH%\" >nul
    )
    
    echo INFO: Artifacts copied successfully to: %RELEASE_PATH%

REM Build completion
:BUILD_SUCCESS
    echo.
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
    echo   - Main Library: .\build\%BUILD_TYPE%\radarsimc.dll
    echo   - Release Package: %RELEASE_PATH%\
    echo   - Header File: %RELEASE_PATH%\radarsim.h
    if /I "%TEST%" == "on" (
        echo   - Test Executable: .\build\%BUILD_TYPE%\radarsim_c_test.exe
    )
    echo.
    if /I "%TEST%" == "on" (
        echo Test Results: All tests passed successfully
    ) else (
        echo Test Results: Tests were skipped - disabled
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
