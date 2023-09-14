#!/bin/sh

echo "Automatic build script of radarsimc_entry for Linux"
echo ""
echo "----------"
echo "RadarSimPy - A Radar Simulator Built with Python"
echo "Copyright (C) 2018 - PRESENT  Zhengyu Peng"
echo "E-mail: zpeng.me@gmail.com"
echo "Website: https://zpeng.me"
echo ""
echo "'                      '"
echo "-:.                  -#:"
echo "-//:.              -###:"
echo "-////:.          -#####:"
echo "-/:.://:.      -###++##:"
echo "..   '://:-  -###+. :##:"
echo "       ':/+####+.   :##:"
echo ".::::::::/+###.     :##:"
echo ".////-----+##:    ':###:"
echo " '-//:.   :##:  ':###/."
echo "   '-//:. :##:':###/."
echo "     '-//:+######/."
echo "       '-/+####/."
echo "         '+##+."
echo "          :##:"
echo "          :##:"
echo "          :##:"
echo "          :##:"
echo "          :##:"
echo "           .+:"

workpath=$(pwd)

echo "## Clean old build files ##"
rm -rf ./build

echo "## Building radarsimc_entry with GPU ##"
mkdir ./build 
cd ./build

cmake -DCMAKE_BUILD_TYPE=Debug -DGPU_BUILD=ON ..
cmake --build .

cd $workpath

echo "## Build completed ##"
