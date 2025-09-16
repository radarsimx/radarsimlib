# RadarSim C Wrapper Library Usage Examples

This directory contains comprehensive examples demonstrating how to use the RadarSim C wrapper library for radar simulation.

## Library Overview

The RadarSim C wrapper library provides a C-compatible interface for advanced radar simulation capabilities, including:

- **Transmitter/Receiver Configuration**: Set up radar systems with custom waveforms and antenna patterns
- **Target Management**: Add point targets and complex 3D mesh targets
- **Radar Simulation**: Generate realistic baseband signals with Doppler effects
- **RCS Simulation**: Calculate radar cross sections using Physical Optics
- **LiDAR Simulation**: Generate point clouds using ray tracing
- **Interference Simulation**: Model radar-to-radar interference
- **Automatic Memory Management**: Built-in cleanup system for safe resource handling

## Examples Provided

### 1. C Example (`example_radarsim.c`)

A complete C implementation demonstrating all major features:

```c
// Key steps in the example:
t_Transmitter *tx = Create_Transmitter(freq, freq_time, samples, ...);
Add_Txchannel(location, polarization, antenna_pattern, ...);

t_Receiver *rx = Create_Receiver(sampling_rate, gains, ...);
Add_Rxchannel(location, polarization, antenna_pattern, ...);

t_Radar *radar = Create_Radar(tx, rx, timing_params, ...);

t_Targets *targets = Init_Targets();
Add_Point_Target(location, velocity, rcs, phase, targets);
Add_Mesh_Target(mesh_data, material_props, targets);

Run_RadarSimulator(radar, targets, quality_params, output_buffers);
```

**Features Demonstrated:**
- FMCW radar configuration (77 GHz automotive radar)
- Antenna pattern setup with gain patterns
- Multiple target types (point scatterers and mesh targets)
- Complete radar simulation with baseband output
- RCS calculation for mesh targets
- LiDAR point cloud generation
- Proper resource cleanup

### 2. Python Example (`example_radarsim.py`)

A Python wrapper using ctypes to interface with the C library:

```python
# Python usage example:
radar_sim = RadarSimWrapper()
version = radar_sim.get_version()

# Create waveform and radar configuration
freq, time = radar_sim.create_fmcw_waveform(77e9, 4e9, 100e-6, 1000)
# ... configure transmitter, receiver, targets
bb_real, bb_imag = radar_sim.run_example()
```

**Features Demonstrated:**
- Dynamic library loading with ctypes
- Type-safe function signatures
- NumPy integration for efficient data handling
- Automatic cleanup management
- Optional matplotlib visualization

## Building and Running

### Prerequisites

- **Windows**: Visual Studio 2019+ or MinGW-w64
- **Linux/macOS**: GCC or Clang
- **Python**: Python 3.6+ with NumPy (optional: matplotlib for visualization)

### Building the C Example

#### Option 1: Using the provided Makefile
```bash
# Copy and rename the example Makefile
copy Makefile.example Makefile

# Build the example
make

# Run the example
make run
```

#### Option 2: Manual compilation
```bash
# Windows (MinGW)
gcc -std=c99 -Wall -O2 -Isrc/includes -o example_radarsim.exe example_radarsim.c -Lradarsimlib_win_x86_64_cpu -lradarsimc -lm

# Linux
gcc -std=c99 -Wall -O2 -Isrc/includes -o example_radarsim example_radarsim.c -L./lib -lradarsimc -lm

# Make sure the library is in your PATH or LD_LIBRARY_PATH
```

### Running the Python Example

```bash
# Install required packages
pip install numpy matplotlib  # matplotlib is optional

# Run the example
python example_radarsim.py
```

## Library API Reference

### Core Data Types

```c
typedef struct s_Transmitter t_Transmitter;  // Opaque transmitter handle
typedef struct s_Receiver t_Receiver;        // Opaque receiver handle
typedef struct s_Radar t_Radar;              // Opaque radar system handle
typedef struct s_Targets t_Targets;          // Opaque target management handle
```

### Key Functions

#### Transmitter Management
```c
t_Transmitter* Create_Transmitter(double *freq, double *freq_time, int waveform_size,
                                 double *freq_offset, double *pulse_start_time, 
                                 int num_pulses, float tx_power);
int Add_Txchannel(float *location, float *polarization, float *antenna_pattern, ...);
void Free_Transmitter(t_Transmitter *tx);
```

#### Receiver Management
```c
t_Receiver* Create_Receiver(float fs, float rf_gain, float resistor, 
                           float baseband_gain, float baseband_bw);
int Add_Rxchannel(float *location, float *polarization, float *antenna_pattern, ...);
void Free_Receiver(t_Receiver *rx);
```

#### Radar System
```c
t_Radar* Create_Radar(t_Transmitter *tx, t_Receiver *rx, double *frame_times,
                     int num_frames, float *platform_params, ...);
void Free_Radar(t_Radar *radar);
```

#### Target Management
```c
t_Targets* Init_Targets();
int Add_Point_Target(float *location, float *velocity, float rcs, float phase, t_Targets *targets);
int Add_Mesh_Target(float *vertices, int *triangles, int num_triangles, 
                   float *material_props, ..., t_Targets *targets);
void Free_Targets(t_Targets *targets);
```

#### Simulation Engines
```c
void Run_RadarSimulator(t_Radar *radar, t_Targets *targets, int quality_level,
                       float ray_density, int *range_filter, 
                       double *output_real, double *output_imag);

int Run_RcsSimulator(t_Targets *targets, double *incident_dirs, double *observation_dirs,
                    int num_directions, double *polarizations, double frequency,
                    double density, double *rcs_results);

int Run_LidarSimulator(t_Targets *targets, double *phi_angles, double *theta_angles,
                      int num_rays, double *sensor_location, double *point_cloud, ...);
```

## Configuration Examples

### FMCW Radar (Automotive)
```c
// 77 GHz automotive radar with 4 GHz bandwidth
const double center_freq = 77e9;
const double bandwidth = 4e9;
const double pulse_duration = 100e-6;
const float tx_power = 20.0f;  // 20 dBm
```

### Pulse Radar (Maritime)
```c
// X-band maritime radar
const double center_freq = 9.4e9;
const double pulse_duration = 1e-6;   // 1 microsecond pulse
const float tx_power = 50.0f;         // 50 dBm high power
```

### Antenna Patterns
```c
// Create directional antenna pattern
for (int i = 0; i < num_angles; i++) {
    angles[i] = (float)i / (num_angles - 1) * 2.0f * M_PI - M_PI;
    // Cosine-squared pattern for directivity
    pattern[i] = max_gain_db * cos(angles[i] / 2.0f) * cos(angles[i] / 2.0f);
}
```

## Free Tier Limitations

The free tier has the following limitations:
- **1 transmitter channel maximum**
- **1 receiver channel maximum** 
- **2 point targets maximum**
- **2 mesh targets maximum**
- **8 triangular faces per mesh maximum**

## Memory Management

The library provides automatic cleanup registration for safety:

```c
// Objects are automatically registered for cleanup
t_Transmitter *tx = Create_Transmitter(...);  // Auto-registered

// Manual cleanup is recommended
Free_Transmitter(tx);  // Explicit cleanup

// Force cleanup of all objects (called automatically on DLL unload)
Force_Cleanup_All();
```

## Error Handling

```c
// Check return values for object creation
t_Transmitter *tx = Create_Transmitter(...);
if (!tx) {
    printf("ERROR: Failed to create transmitter\\n");
    return -1;
}

// Check return codes for configuration functions
int result = Add_Txchannel(...);
if (result != 0) {
    printf("ERROR: Failed to add transmitter channel\\n");
    return -1;
}
```

## Performance Tips

1. **Buffer Pre-allocation**: Always pre-allocate output buffers to the correct size
2. **Ray Density**: Higher density = more accurate but slower simulation
3. **Quality Level**: Use level 3-4 for good balance of speed vs accuracy
4. **Range Filtering**: Use ray filters to limit computation to areas of interest
5. **Memory Management**: Call Free_* functions explicitly for better performance

## Troubleshooting

### Library Loading Issues
- Ensure the DLL/shared library is in your PATH or library search path
- Check that all dependencies are available
- Verify architecture match (x64 vs x86)

### Simulation Errors
- Verify all input arrays are properly allocated and sized
- Check that antenna patterns cover the full angular range
- Ensure target parameters are within reasonable physical limits

### Performance Issues
- Reduce ray density for faster simulation
- Lower quality level for initial testing
- Use range filtering to limit computation area

## Support and Documentation

For additional support and detailed API documentation:
- Website: https://radarsimx.com
- Email: info@radarsimx.com

## License

Copyright (C) 2023 - PRESENT radarsimx.com
See LICENSE file for full license terms.
