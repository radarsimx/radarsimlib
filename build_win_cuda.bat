@ECHO OFF

ECHO Automatic build script of radarsimc_entry for Windows
ECHO:
ECHO ----------
ECHO Copyright (C) 2023 - PRESENT  radarsimx.com
ECHO E-mail: info@radarsimx.com
ECHO Website: https://radarsimx.com
ECHO:
ECHO  *******                  **                   ******** **             **     **
ECHO /**////**                /**                  **////// //             //**   **
ECHO /**   /**   ******       /**  ******   ******/**        ** **********  //** **
ECHO /*******   //////**   ****** //////** //**//*/*********/**//**//**//**  //***
ECHO /**///**    *******  **///**  *******  /** / ////////**/** /** /** /**   **/**
ECHO /**  //**  **////** /**  /** **////**  /**          /**/** /** /** /**  ** //**
ECHO /**   //**//********//******//********/***    ******** /** *** /** /** **   //**
ECHO //     //  ////////  //////  //////// ///    ////////  // ///  //  // //     //
ECHO:

SET pwd=%cd%

ECHO ## Clean old build files ##
RMDIR /Q/S .\build
RMDIR /Q/S .\radarsimlib

ECHO ## Building radarsimlib with GPU ##

@REM go to the build folder
MD ".\build"
CD ".\build"

@REM MSVC needs to set the build type using '--config Relesae' 
cmake -DGPU_BUILD=ON -DFREETIER=ON ..
cmake --build . --config Release

CD %pwd%

MD ".\radarsimlib"
COPY .\build\Release\radarsimc.dll .\radarsimlib\
COPY .\src\includes\radarsim.h .\radarsimlib\

ECHO ## Build completed ##
