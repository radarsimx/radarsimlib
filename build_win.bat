@ECHO OFF

ECHO Automatic build script of radarsimc_entry for Windows
ECHO:
ECHO ----------
ECHO RadarSimPy - A Radar Simulator Built with Python
ECHO Copyright (C) 2018 - PRESENT  Zhengyu Peng
ECHO E-mail: zpeng.me@gmail.com
ECHO Website: https://zpeng.me
ECHO:
ECHO `                      `
ECHO -:.                  -#:
ECHO -//:.              -###:
ECHO -////:.          -#####:
ECHO -/:.://:.      -###++##:
ECHO ..   `://:-  -###+. :##:
ECHO        `:/+####+.   :##:
ECHO .::::::::/+###.     :##:
ECHO .////-----+##:    `:###:
ECHO  `-//:.   :##:  `:###/.
ECHO    `-//:. :##:`:###/.
ECHO      `-//:+######/.
ECHO        `-/+####/.
ECHO          `+##+.
ECHO           :##:
ECHO           :##:
ECHO           :##:
ECHO           :##:
ECHO           :##:
ECHO            .+:

SET pwd=%cd%

ECHO ## Clean old build files ##
RMDIR /Q/S .\build
RMDIR /Q/S .\radarsimlib

ECHO ## Building radarsimlib with CPU ##

@REM go to the build folder
MD ".\build"
CD ".\build"

@REM MSVC needs to set the build type using '--config Relesae' 
cmake -DGPU_BUILD=OFF ..
cmake --build . --config Release

CD %pwd%

MD ".\radarsimlib"
COPY .\build\Release\radarsimc.dll .\radarsimlib\
COPY .\src\includes\radarsim.h .\radarsimlib\

ECHO ## Build completed ##
