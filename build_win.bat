@ECHO OFF

set TIER=standard
set ARCH=cpu

goto GETOPTS

:Help
ECHO:
ECHO Usages:
ECHO    --help	Show the usages of the parameters
ECHO    --tier	Build tier, choose 'standard' or 'free'. Default is 'standard'
ECHO    --arch	Build architecture, choose 'cpu' or 'gpu'. Default is 'cpu'
ECHO:
goto EOF

:GETOPTS
if /I "%1" == "--help" goto Help
if /I "%1" == "--tier" set TIER=%2 & shift
if /I "%1" == "--arch" set ARCH=%2 & shift
shift
if not "%1" == "" goto GETOPTS

if /I NOT %TIER% == free (
    if /I NOT %TIER% == standard (
        ECHO ERROR: Invalid --tier parameters, please choose 'free' or 'standard'
        goto EOF
    )
)

if /I NOT %ARCH% == cpu (
    if /I NOT %ARCH% == gpu (
        ECHO ERROR: Invalid --arch parameters, please choose 'cpu' or 'gpu'
        goto EOF
    )
)

ECHO Automatic build script of radarsimlib for Windows
ECHO:
ECHO ----------
ECHO Copyright (C) 2023 - PRESENT  radarsimx.com
ECHO E-mail: info@radarsimx.com
ECHO Website: https://radarsimx.com
ECHO:
ECHO  ######                               #####           #     # 
ECHO  #     #   ##   #####    ##   #####  #     # # #    #  #   #  
ECHO  #     #  #  #  #    #  #  #  #    # #       # ##  ##   # #   
ECHO  ######  #    # #    # #    # #    #  #####  # # ## #    #    
ECHO  #   #   ###### #    # ###### #####        # # #    #   # #   
ECHO  #    #  #    # #    # #    # #   #  #     # # #    #  #   #  
ECHO  #     # #    # #####  #    # #    #  #####  # #    # #     # 
ECHO:

SET pwd=%cd%

ECHO ## Clean old build files ##
RMDIR /Q/S .\build
RMDIR /Q/S .\radarsimlib

@REM go to the build folder
MD ".\build"
CD ".\build"

@REM MSVC needs to set the build type using '--config Relesae'
if /I %ARCH% == gpu (
    if /I %TIER% == standard (
        ECHO ## Build standard GPU verion ##
        cmake -DGPU_BUILD=ON -DFREETIER=OFF ..
    ) else if /I %TIER% == free (
        ECHO ## Build freetier GPU verion ##
        cmake -DGPU_BUILD=ON -DFREETIER=ON ..
    )
) else if /I %ARCH% == cpu (
    if /I %TIER% == standard (
        ECHO ## Build standard CPU verion ##
        cmake -DGPU_BUILD=OFF -DFREETIER=OFF ..
    ) else if /I %TIER% == free (
        ECHO ## Build freetier CPU verion ##
        cmake -DGPU_BUILD=OFF -DFREETIER=ON ..
    )
)
cmake --build . --config Release

CD %pwd%

MD ".\radarsimlib"
COPY .\build\Release\radarsimc.dll .\radarsimlib\
COPY .\src\includes\radarsim.h .\radarsimlib\

ECHO ## Build completed ##
