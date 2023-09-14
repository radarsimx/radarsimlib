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

ECHO ## Building radarsimc_entry with GPU ##

@REM go to the build folder
MD ".\build"
CD ".\build"

@REM MSVC needs to set the build type using '--config Relesae' 
cmake -DGPU_BUILD=ON ..
cmake --build . --config Debug

CD %pwd%

ECHO ## Build completed ##
