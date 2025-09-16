@echo off
REM Build script for RadarSim C Example using CMake
REM This script creates a build directory and compiles the example

echo Building RadarSim C Example...
echo ============================

REM Create build directory
if not exist build mkdir build
cd build

REM Configure with CMake
echo Configuring with CMake...
cmake ..

REM Check if configuration was successful
if %ERRORLEVEL% neq 0 (
    echo ERROR: CMake configuration failed!
    echo.
    echo Trying with MinGW Makefiles...
    cmake .. -G "MinGW Makefiles"
    if %ERRORLEVEL% neq 0 (
        echo ERROR: CMake configuration failed with both generators!
        echo Please ensure you have either Visual Studio or MinGW installed.
        pause
        exit /b 1
    )
)

REM Build the project
echo Building the project...
cmake --build . --config Release

REM Check if build was successful
if %ERRORLEVEL% neq 0 (
    echo ERROR: Build failed!
    pause
    exit /b 1
)

echo.
echo Build completed successfully!
echo Executable location: build\bin\Release\example_radarsim.exe (VS) or build\bin\example_radarsim.exe (MinGW)
echo.
echo To run the example:
echo   cd build
echo   cmake --build . --target run_example
echo.
echo Or run directly:
echo   .\build\bin\Release\example_radarsim.exe   (VS)
echo   .\build\bin\example_radarsim.exe           (MinGW)
echo.

cd ..
