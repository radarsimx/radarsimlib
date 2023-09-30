@ECHO OFF

ECHO Automatic build script of radarsimc_entry for Windows
ECHO:
ECHO ----------
ECHO Copyright (C) 2023 - PRESENT  radarsimx.com
ECHO E-mail: info@radarsimx.com
ECHO Website: https://radarsimx.com
ECHO:
ECHO " ____           _            ____  _          __  __"
ECHO "|  _ \ __ _  __| | __ _ _ __/ ___|(_)_ __ ___ \ \/ /"
ECHO "| |_) / _` |/ _` |/ _` | '__\___ \| | '_ ` _ \ \  /"
ECHO "|  _ < (_| | (_| | (_| | |   ___) | | | | | | |/  \"
ECHO "|_| \_\__,_|\__,_|\__,_|_|  |____/|_|_| |_| |_/_/\_\"
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
cmake -DGPU_BUILD=ON ..
cmake --build . --config Release

CD %pwd%

MD ".\radarsimlib"
COPY .\build\Release\radarsimc.dll .\radarsimlib\
COPY .\src\includes\radarsim.h .\radarsimlib\

ECHO ## Build completed ##
