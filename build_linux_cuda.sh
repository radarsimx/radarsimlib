#!/bin/sh

echo "Automatic build script of radarsimc_entry for Linux"
echo ""
echo "----------"
echo "Copyright (C) 2023 - PRESENT  radarsimx.com"
echo "E-mail: info@radarsimx.com"
echo "Website: https://radarsimx.com"
echo ""
echo " ____           _            ____  _          __  __ "
echo "|  _ \ __ _  __| | __ _ _ __/ ___|(_)_ __ ___ \ \/ / "
echo "| |_) / _' |/ _' |/ _' | '__\___ \| | '_ ' _ \ \  /  "
echo "|  _ < (_| | (_| | (_| | |   ___) | | | | | | |/  \  "
echo "|_| \_\__,_|\__,_|\__,_|_|  |____/|_|_| |_| |_/_/\_\ "

workpath=$(pwd)

echo "## Clean old build files ##"
rm -rf ./build

echo "## Building radarsimlib with GPU ##"
mkdir ./build 
cd ./build

cmake -DCMAKE_BUILD_TYPE=Release -DGPU_BUILD=ON ..
cmake --build .

cd $workpath

echo "## Build completed ##"
