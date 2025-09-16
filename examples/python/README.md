# RadarSim Python Example

This directory contains a Python example demonstrating how to use the RadarSim C library from Python using ctypes.

## Features

- **Dynamic Library Loading**: Automatic discovery and loading of RadarSim library
- **Type-Safe Interface**: Proper ctypes function signatures for all API calls
- **NumPy Integration**: Efficient array handling and data processing
- **Error Handling**: Comprehensive error checking and custom exceptions
- **Automatic Cleanup**: Resource management with automatic cleanup on exit
- **Optional Visualization**: Matplotlib integration for result plotting

## Prerequisites

- Python 3.6 or higher
- NumPy
- RadarSim library (compiled and available)
- Optional: matplotlib (for visualization)

## Installation

```bash
# Install required packages
pip install numpy

# Optional: for visualization
pip install matplotlib
```

## Usage

### Basic Usage

```python
from example_radarsim import RadarSimWrapper

# Create wrapper instance
radar_sim = RadarSimWrapper()

# Get library version
version = radar_sim.get_version()
print(f"RadarSim Version: {version[0]}.{version[1]}.{version[2]}")

# Run complete example
bb_real, bb_imag = radar_sim.run_example()
```

### Custom Configuration

```python
import numpy as np
from example_radarsim import RadarSimWrapper

radar_sim = RadarSimWrapper()

# Create custom waveform
freq, time = radar_sim.create_fmcw_waveform(
    center_freq=77e9,    # 77 GHz
    bandwidth=4e9,       # 4 GHz bandwidth
    duration=100e-6,     # 100 microseconds
    samples=1000
)

# Create custom antenna pattern
angles, pattern = radar_sim.create_antenna_pattern(
    num_points=361,      # 1 degree resolution
    max_gain_db=20.0     # 20 dB max gain
)
```

## Running the Example

```bash
# Run the complete example
python example_radarsim.py
```

### Expected Output

```
RadarSim Python Example
=======================
RadarSim Library Version: 14.0.0

1. Creating FMCW waveform...
2. Creating transmitter...
3. Creating receiver...
4. Creating radar system...
5. Creating targets...
6. Running radar simulation...
   Buffer size: 128000 samples
   Max amplitude: 1.234567e-03
   RMS amplitude: 2.345678e-05
   Signal power: 4.567890e-01
8. Cleaning up...

✓ Example completed successfully!
Results plotted and saved as 'radar_simulation_results.png'
```

## Library Path Configuration

The wrapper automatically searches for the RadarSim library in common locations:

```python
# Windows
radarsimlib_win_x86_64_cpu/radarsimc.dll
./radarsimc.dll
./Release/radarsimc.dll
./build/Release/radarsimc.dll

# Linux
./libradarsimc.so
./lib/libradarsimc.so
./build/lib/libradarsimc.so

# macOS
./libradarsimc.dylib
./lib/libradarsimc.dylib
./build/lib/libradarsimc.dylib
```

### Custom Library Path

```python
# Specify custom library path
radar_sim = RadarSimWrapper(lib_path="/path/to/radarsimc.dll")
```

## API Overview

### Core Classes

```python
class RadarSimWrapper:
    """Main wrapper class for RadarSim library"""
    
    def __init__(self, lib_path=None)
    def get_version(self) -> Tuple[int, int, int]
    def create_fmcw_waveform(self, center_freq, bandwidth, duration, samples)
    def create_antenna_pattern(self, num_points, max_gain_db)
    def run_example(self) -> Tuple[np.ndarray, np.ndarray]
    def cleanup(self)

class RadarSimError(Exception):
    """Custom exception for RadarSim errors"""
```

### Helper Functions

```python
# Create FMCW waveform
freq, time = radar_sim.create_fmcw_waveform(77e9, 4e9, 100e-6, 1000)

# Create antenna pattern
angles, pattern = radar_sim.create_antenna_pattern(361, 15.0)
```

## Data Processing

The example returns complex baseband signals as NumPy arrays:

```python
bb_real, bb_imag = radar_sim.run_example()

# Calculate amplitude
amplitude = np.sqrt(bb_real**2 + bb_imag**2)

# Calculate power
power = amplitude**2

# Find peaks (potential targets)
from scipy.signal import find_peaks
peaks, _ = find_peaks(amplitude, height=np.max(amplitude)*0.1)
```

## Visualization

If matplotlib is available, the example automatically generates plots:

```python
import matplotlib.pyplot as plt

# Plot real part
plt.subplot(1, 2, 1)
plt.plot(bb_real[:1000])
plt.title('Baseband Signal (Real)')

# Plot amplitude
plt.subplot(1, 2, 2)
plt.plot(amplitude[:1000])
plt.title('Signal Amplitude')

plt.show()
```

## Advanced Usage

### Custom Radar Configuration

```python
# Access low-level library functions directly
lib = radar_sim.lib

# Create custom transmitter
tx = lib.Create_Transmitter(...)

# Add multiple channels
for i in range(num_channels):
    lib.Add_Txchannel(...)
```

### Batch Processing

```python
def process_multiple_scenarios(scenarios):
    radar_sim = RadarSimWrapper()
    results = []
    
    for scenario in scenarios:
        # Configure scenario
        # Run simulation
        bb_real, bb_imag = radar_sim.run_example()
        results.append((bb_real, bb_imag))
    
    return results
```

## Error Handling

```python
try:
    radar_sim = RadarSimWrapper()
    bb_real, bb_imag = radar_sim.run_example()
except RadarSimError as e:
    print(f"RadarSim Error: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
```

## Performance Tips

1. **Memory Management**: The wrapper handles automatic cleanup, but you can call `cleanup()` explicitly
2. **Array Efficiency**: Use NumPy arrays for all data to avoid copying overhead
3. **Library Loading**: Create one wrapper instance and reuse it for multiple simulations
4. **Quality Settings**: Adjust ray density and quality level for speed vs accuracy tradeoff

## Troubleshooting

### Library Loading Issues

```python
# Check if library is found
try:
    radar_sim = RadarSimWrapper()
    print("Library loaded successfully")
except RadarSimError as e:
    print(f"Library loading failed: {e}")
    # Check paths and dependencies
```

### Common Issues

1. **Import Error**: Ensure NumPy is installed
2. **Library Not Found**: Check that RadarSim DLL/shared library is built and accessible
3. **Architecture Mismatch**: Ensure Python and library have same architecture (x64 vs x86)
4. **Permission Error**: Check file permissions on library file

### Debugging

```python
# Enable debug output
import logging
logging.basicConfig(level=logging.DEBUG)

# Check library version
version = radar_sim.get_version()
print(f"Using RadarSim version: {version}")

# Verify function availability
print("Available functions:", dir(radar_sim.lib))
```

## Integration with Other Libraries

### SciPy Integration

```python
from scipy import signal
from scipy.fft import fft, fftfreq

# FFT processing
fft_result = fft(bb_real + 1j * bb_imag)
frequencies = fftfreq(len(bb_real), 1/sampling_rate)

# Filtering
filtered_signal = signal.butter(4, 0.1, 'low', output='sos')
```

### Pandas Integration

```python
import pandas as pd

# Create DataFrame for analysis
df = pd.DataFrame({
    'real': bb_real,
    'imag': bb_imag,
    'amplitude': amplitude,
    'power': power
})

# Statistical analysis
print(df.describe())
```

## Support

For questions about this Python example:
- Website: https://radarsimx.com
- Email: info@radarsimx.com

For Python-specific issues:
- Check Python version compatibility
- Verify NumPy installation
- Ensure library architecture matches Python architecture
