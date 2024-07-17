#!/bin/bash

Help()
{
   # Display Help
   echo
   echo "Usages:"
   echo
   echo "Syntax: build_linux.sh --tier=[standard|free] --arch=[cpu|gpu] --test=[on|off]"
   echo "options:"
   echo "   --help	Show the usages of the parameters"
   echo "   --tier	Build tier, choose 'standard' or 'free'. Default is 'standard'"
   echo "   --arch	Build architecture, choose 'cpu' or 'gpu'. Default is 'cpu'"
   echo "   --test	Enable or disable unit test, choose 'on' or 'off'. Default is 'on'"
   echo
}

TIER="standard"
ARCH="cpu"

for i in "$@"; do
  case $i in
    --help*)
      Help
      exit;;
    --tier=*)
      TIER="${i#*=}"
      shift # past argument
      ;;
    --arch=*)
      ARCH="${i#*=}"
      shift # past argument
      ;;
    --*)
      echo "Unknown option $1"
      exit 1
      ;;
    *)
      ;;
  esac
done

if [ "${TIER,,}" != "standard" ] && [ "${TIER,,}" != "free" ]; then
    echo "ERROR: Invalid --tier parameters, please choose 'free' or 'standard'"
    exit 1
fi

if [ "${ARCH,,}" != "cpu" ] && [ "${ARCH,,}" != "gpu" ]; then
    echo "ERROR: Invalid --arch parameters, please choose 'cpu' or 'gpu'"
    exit 1
fi

echo "Automatic build script of radarsimlib for Linux"
echo ""
echo "----------"
echo "Copyright (C) 2023 - PRESENT  radarsimx.com"
echo "E-mail: info@radarsimx.com"
echo "Website: https://radarsimx.com"
echo ""
echo "██████╗  █████╗ ██████╗  █████╗ ██████╗ ███████╗██╗███╗   ███╗██╗  ██╗"
echo "██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔════╝██║████╗ ████║╚██╗██╔╝"
echo "██████╔╝███████║██║  ██║███████║██████╔╝███████╗██║██╔████╔██║ ╚███╔╝ "
echo "██╔══██╗██╔══██║██║  ██║██╔══██║██╔══██╗╚════██║██║██║╚██╔╝██║ ██╔██╗ "
echo "██║  ██║██║  ██║██████╔╝██║  ██║██║  ██║███████║██║██║ ╚═╝ ██║██╔╝ ██╗"
echo "╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚═╝╚═╝     ╚═╝╚═╝  ╚═╝"

workpath=$(pwd)

echo "## Clean old build files ##"
rm -rf ./build

mkdir ./build 
cd ./build

if [ "${ARCH,,}" == "gpu" ]; then
    if [ "${TIER,,}" == "standard" ]; then
        echo "## Build standard GPU verion ##"
        release_path="./radarsimlib_linux_x86_64_gpu"
        cmake -DCMAKE_BUILD_TYPE=Release -DGPU_BUILD=ON -DFREETIER=OFF ..
    elif [ "${TIER,,}" == "free" ]; then
        echo "## Build freetier GPU verion ##"
        release_path="./radarsimlib_linux_x86_64_gpu_free"
        cmake -DCMAKE_BUILD_TYPE=Release -DGPU_BUILD=ON -DFREETIER=ON ..
    fi
elif [ "${ARCH,,}" == "cpu" ]; then
    if [ "${TIER,,}" == "standard" ]; then
        echo "## Build standard CPU verion ##"
        release_path="./radarsimlib_linux_x86_64_cpu"
        cmake -DCMAKE_BUILD_TYPE=Release -DGPU_BUILD=OFF -DFREETIER=OFF ..
    elif [ "${TIER,,}" == "free" ]; then
        echo "## Build freetier CPU verion ##"
        release_path="./radarsimlib_linux_x86_64_cpu_free"
        cmake -DCMAKE_BUILD_TYPE=Release -DGPU_BUILD=OFF -DFREETIER=ON ..
    fi
fi
cmake --build .

cd $workpath

rm -rf ${release_path}
mkdir ${release_path}
cp ./build/libradarsimc.so ${release_path}
cp ./src/includes/radarsim.h ${release_path}

echo "## Build completed ##"
