#!/bin/bash
# Build script for RadarSim C Example using CMake
# This script creates a build directory and compiles the example

echo "Building RadarSim C Example..."
echo "============================"

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Check if configuration was successful
if [ $? -ne 0 ]; then
    echo "ERROR: CMake configuration failed!"
    echo "Please ensure you have CMake and a C compiler installed."
    exit 1
fi

# Build the project
echo "Building the project..."
cmake --build . --config Release

# Check if build was successful
if [ $? -ne 0 ]; then
    echo "ERROR: Build failed!"
    exit 1
fi

echo ""
echo "Build completed successfully!"
echo "Executable location: build/bin/example_radarsim"
echo ""
echo "To run the example:"
echo "  cd build"
echo "  cmake --build . --target run_example"
echo ""
echo "Or run directly:"
echo "  ./build/bin/example_radarsim"
echo ""

cd ..
