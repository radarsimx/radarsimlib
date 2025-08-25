@echo off
REM
REM Test runner script for RadarSim C Wrapper tests (Windows)
REM
REM This script provides convenient commands for running the C wrapper tests
REM with various options and configurations on Windows systems.
REM
REM Usage:
REM   run_tests.bat [options]
REM
REM Options:
REM   /build       Build tests before running
REM   /clean       Clean build directory before building
REM   /coverage    Enable coverage reporting
REM   /verbose     Run tests with verbose output
REM   /filter:X    Run only tests matching filter X
REM   /help        Show this help message
REM
REM Examples:
REM   run_tests.bat /build /verbose
REM   run_tests.bat /filter:*TransmitterTest*
REM   run_tests.bat /coverage
REM
REM Copyright (C) 2023 - PRESENT  radarsimx.com

setlocal enabledelayedexpansion

REM Default configuration
set BUILD_TESTS=false
set CLEAN_BUILD=false
set ENABLE_COVERAGE=false
set VERBOSE_OUTPUT=false
set TEST_FILTER=
set BUILD_DIR=build
set CMAKE_GENERATOR=

REM Script directory
set SCRIPT_DIR=%~dp0
set ROOT_DIR=%SCRIPT_DIR%..

REM Parse command line arguments
:parse_args
if "%~1"=="" goto end_parse
if /i "%~1"=="/build" (
    set BUILD_TESTS=true
    shift
    goto parse_args
)
if /i "%~1"=="/clean" (
    set CLEAN_BUILD=true
    shift
    goto parse_args
)
if /i "%~1"=="/coverage" (
    set ENABLE_COVERAGE=true
    shift
    goto parse_args
)
if /i "%~1"=="/verbose" (
    set VERBOSE_OUTPUT=true
    shift
    goto parse_args
)
if /i "%~1"=="/help" (
    goto show_help
)
if "%~1:~0,8%"=="/filter:" (
    set TEST_FILTER=%~1:~8%
    shift
    goto parse_args
)
echo [ERROR] Unknown option: %~1
goto show_help

:end_parse

REM Main execution
echo [INFO] RadarSim C Wrapper Test Runner (Windows)
echo [INFO] =======================================

REM Check dependencies
echo [INFO] Checking dependencies...
where cmake >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] CMake is not installed or not in PATH
    exit /b 1
)

REM Detect generator
where ninja >nul 2>&1
if %errorlevel% equ 0 (
    set CMAKE_GENERATOR=-G Ninja
    echo [INFO] Using Ninja generator
) else (
    echo [INFO] Using default Visual Studio generator
)

REM Clean build directory
if "%CLEAN_BUILD%"=="true" (
    echo [INFO] Cleaning build directory...
    if exist "%BUILD_DIR%" (
        rmdir /s /q "%BUILD_DIR%"
        echo [SUCCESS] Build directory cleaned
    )
)

REM Configure and build
if "%BUILD_TESTS%"=="true" (
    echo [INFO] Configuring CMake...
    
    if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
    cd /d "%BUILD_DIR%"
    
    set CMAKE_ARGS=-DBUILD_CWRAPPER_TESTS=ON
    
    if "%ENABLE_COVERAGE%"=="true" (
        set CMAKE_ARGS=!CMAKE_ARGS! -DENABLE_COVERAGE=ON
        echo [INFO] Coverage reporting enabled
    )
    
    cmake !CMAKE_ARGS! %CMAKE_GENERATOR% "%ROOT_DIR%"
    if %errorlevel% neq 0 (
        echo [ERROR] CMake configuration failed
        exit /b 1
    )
    echo [SUCCESS] CMake configuration completed
    
    echo [INFO] Building C wrapper tests...
    cmake --build . --target cwrapper_tests_all --config Release
    if %errorlevel% neq 0 (
        echo [ERROR] Build failed
        exit /b 1
    )
    echo [SUCCESS] Build completed successfully
)

REM Run tests
echo [INFO] Running C wrapper tests...

if not exist "%BUILD_DIR%" (
    echo [ERROR] Build directory not found. Use /build option first.
    exit /b 1
)

cd /d "%BUILD_DIR%"

REM Check for test executable
set TEST_EXE=Release\cwrapper_tests_all.exe
if not exist "%TEST_EXE%" (
    set TEST_EXE=cwrapper_tests_all.exe
)
if not exist "%TEST_EXE%" (
    echo [ERROR] Test executable not found. Use /build option first.
    exit /b 1
)

REM Prepare test command
set TEST_CMD=%TEST_EXE%

if not "%TEST_FILTER%"=="" (
    set TEST_CMD=!TEST_CMD! --gtest_filter=%TEST_FILTER%
    echo [INFO] Running tests with filter: %TEST_FILTER%
)

if "%VERBOSE_OUTPUT%"=="true" (
    set TEST_CMD=!TEST_CMD! --gtest_verbose
    echo [INFO] Running tests with verbose output
)

REM Run the tests
echo [INFO] Executing: !TEST_CMD!
!TEST_CMD!
set TEST_RESULT=%errorlevel%

if %TEST_RESULT% equ 0 (
    echo [SUCCESS] All tests passed!
) else (
    echo [ERROR] Some tests failed (exit code: %TEST_RESULT%)
    exit /b %TEST_RESULT%
)

REM Generate coverage report
if "%ENABLE_COVERAGE%"=="true" (
    echo [INFO] Generating coverage report...
    cmake --build . --target coverage_cwrapper --config Release
    if %errorlevel% equ 0 (
        echo [SUCCESS] Coverage report generated in coverage_html\
        REM Try to open coverage report
        if exist "coverage_html\index.html" (
            echo [INFO] Opening coverage report in browser...
            start coverage_html\index.html
        )
    ) else (
        echo [WARNING] Coverage report generation failed
    )
)

echo [SUCCESS] Test execution completed successfully!
goto end

:show_help
echo RadarSim C Wrapper Test Runner (Windows)
echo.
echo Usage: %~nx0 [options]
echo.
echo Options:
echo   /build       Build tests before running
echo   /clean       Clean build directory before building
echo   /coverage    Enable coverage reporting
echo   /verbose     Run tests with verbose output
echo   /filter:X    Run only tests matching filter X
echo   /help        Show this help message
echo.
echo Examples:
echo   %~nx0 /build /verbose
echo   %~nx0 /filter:*TransmitterTest*
echo   %~nx0 /coverage
echo.
exit /b 0

:end
endlocal
