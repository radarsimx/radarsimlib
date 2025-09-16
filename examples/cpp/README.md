# RadarSim C++ Example

This directory contains a complete C++ example demonstrating the RadarSim C wrapper library.

## Features Demonstrated

- **FMCW Radar Configuration**: 77 GHz automotive radar with 4 GHz bandwidth
- **C++ Modern Features**: STL containers, RAII, exception handling
- **Antenna Pattern Setup**: Configurable gain patterns for transmitter and receiver
- **Target Management**: Point scatterers and 3D mesh targets
- **Radar Simulation**: Complete baseband signal generation
- **RCS Simulation**: Radar cross section calculation using Physical Optics
- **LiDAR Simulation**: Point cloud generation using ray tracing
- **Memory Management**: RAII-based resource cleanup and error handling

## Building

### Prerequisites

- CMake 3.16 or higher
- C++11-compatible compiler (GCC, Clang, MSVC)
- RadarSim library (should be built and available)

### Quick Build (Windows)

```cmd
# Run the build script
build.bat
```

### Quick Build (Linux/macOS)

```bash
# Make the script executable and run it
chmod +x build.sh
./build.sh
```

### Manual Build

```bash
# Create build directory
mkdir build
cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build . --config Release

# Run
cmake --build . --target run_example
```

### Visual Studio

```cmd
# Generate Visual Studio solution
mkdir build
cd build
cmake .. -G "Visual Studio 16 2019" -A x64

# Open in Visual Studio
start RadarSimExample.sln
```

## Running

After building, you can run the example in several ways:

### Using CMake target
```bash
cd build
cmake --build . --target run_example
```

### Direct execution
```bash
# Windows (Visual Studio)
.\build\bin\Release\example_radarsim.exe

# Windows (MinGW) or Linux/macOS
./build/bin/example_radarsim
```

## Expected Output

The example will output progress information and simulation results:

```text
RadarSim C++ Wrapper Library Example
====================================

RadarSim Library Version: 14.0.0

1. Creating Transmitter...
   ✓ Transmitter created with 1 channels
2. Creating Receiver...
   ✓ Receiver created with 1 channels
3. Creating Radar System...
   ✓ Radar system created successfully
4. Creating Target Scenario...
   ✓ Target scenario created (2 point targets + 1 mesh target)
5. Running Radar Simulation...
   Buffer size: 128000 samples (128 pulses × 1 channels × 1000 samples/pulse)
   ✓ Radar simulation completed
   Max amplitude: 1.234567e-03, RMS amplitude: 2.345678e-05
6. Running RCS Simulation...
   ✓ RCS simulation completed
   Maximum RCS: 4.567890e-02 m²
7. Running LiDAR Simulation...
   ✓ LiDAR simulation completed
   Point cloud contains 15 points
   First point: (150.12, -29.87, 2.03) at distance 153.45 m

✓ All simulations completed successfully!

8. Cleaning up...
   ✓ Cleanup completed (automatic with RAII)

Example completed successfully!
```

## Configuration

The example is configured for a typical automotive radar scenario:

- **Frequency**: 77 GHz center frequency
- **Bandwidth**: 4 GHz (FMCW)
- **Pulse Duration**: 100 microseconds
- **Number of Pulses**: 128
- **Sampling Rate**: 10 MHz
- **Platform Height**: 1.5 meters

### Targets:
1. **Point Target 1**: 100m range, approaching at 20 m/s, 10 dBsm RCS
2. **Point Target 2**: 200m range, 50m cross-range, moving laterally at 10 m/s, 5 dBsm RCS
3. **Mesh Target**: Triangular plate at 150m range, slowly rotating

## Customization

You can modify the example by changing parameters in `example_radarsim.cpp`:

### Radar Parameters

```cpp
const double center_freq = 77e9;      // Change frequency
const double bandwidth = 4e9;         // Change bandwidth
const int num_pulses = 128;           // Change number of pulses
const float tx_power = 20.0f;         // Change transmit power
```

### Target Parameters

```cpp
// Modify target locations, velocities, and RCS values
std::vector<float> target1_location = {100.0f, 0.0f, 1.0f};
std::vector<float> target1_speed = {-20.0f, 0.0f, 0.0f};
float target1_rcs = 10.0f;
```

### Simulation Quality

```cpp
// Adjust simulation quality vs speed tradeoff
Run_RadarSimulator(radar, targets, 
                   3,     // Quality level (1-5)
                   1.0f,  // Ray density
                   ray_filter.data(), bb_real.data(), bb_imag.data());
```

## Troubleshooting

### Library Not Found
If you get library loading errors:
1. Ensure the RadarSim library is built
2. Check that the library path in CMakeLists.txt is correct
3. Verify the DLL/shared library is in the correct location

### Compilation Errors

- Ensure you have a C++11-compatible compiler
- Check that CMake version is 3.16 or higher
- Verify include paths point to the correct header location

### Runtime Errors
- Check that all required DLLs are in PATH (Windows)
- Verify library architecture matches (x64 vs x86)
- Ensure sufficient memory for large simulations

## Performance Notes

- Ray density affects simulation accuracy and speed
- Higher quality levels provide better results but take longer
- Use range filtering to limit computation to areas of interest
- Pre-allocate buffers to avoid runtime allocation overhead

## Support

For questions about this example or the RadarSim library:
- Website: https://radarsimx.com
- Email: info@radarsimx.com
