#!/usr/bin/env python3
"""
RadarSim C Wrapper Library - Python Example using ctypes

This example demonstrates how to use the RadarSim C library from Python
using the ctypes foreign function interface.

Features demonstrated:
- Loading the RadarSim DLL
- Creating transmitter and receiver configurations
- Setting up a radar system with targets
- Running radar simulation
- Processing simulation results
"""

import ctypes
import numpy as np
import os
import sys
from typing import Optional, Tuple

# Platform-specific library loading
if sys.platform == "win32":
    lib_name = "radarsimc.dll"
elif sys.platform == "darwin":
    lib_name = "libradarsimc.dylib"
else:
    lib_name = "libradarsimc.so"

class RadarSimError(Exception):
    """Custom exception for RadarSim library errors"""
    pass

class RadarSimWrapper:
    """Python wrapper for RadarSim C library using ctypes"""
    
    def __init__(self, lib_path: str = None):
        """Initialize the RadarSim wrapper
        
        Args:
            lib_path: Path to the RadarSim library. If None, searches common locations.
        """
        self.lib = self._load_library(lib_path)
        self._setup_function_signatures()
        self._setup_cleanup()
        
    def _load_library(self, lib_path: str) -> ctypes.CDLL:
        """Load the RadarSim dynamic library"""
        search_paths = []
        
        if lib_path:
            search_paths.append(lib_path)
        
        # Add common search locations
        search_paths.extend([
            f"./radarsimlib_win_x86_64_cpu/{lib_name}",
            f"./{lib_name}",
            f"./Release/{lib_name}",
            f"./build/Release/{lib_name}"
        ])
        
        for path in search_paths:
            if os.path.exists(path):
                try:
                    return ctypes.CDLL(path)
                except OSError as e:
                    print(f"Failed to load {path}: {e}")
                    continue
        
        raise RadarSimError(f"Could not find RadarSim library ({lib_name})")
    
    def _setup_function_signatures(self):
        """Set up function signatures for type safety"""
        
        # Version function
        self.lib.Get_Version.argtypes = [ctypes.POINTER(ctypes.c_int)]
        self.lib.Get_Version.restype = None
        
        # Transmitter functions
        self.lib.Create_Transmitter.argtypes = [
            ctypes.POINTER(ctypes.c_double),  # freq
            ctypes.POINTER(ctypes.c_double),  # freq_time
            ctypes.c_int,                     # waveform_size
            ctypes.POINTER(ctypes.c_double),  # freq_offset
            ctypes.POINTER(ctypes.c_double),  # pulse_start_time
            ctypes.c_int,                     # num_pulses
            ctypes.c_float                    # tx_power
        ]
        self.lib.Create_Transmitter.restype = ctypes.c_void_p
        
        self.lib.Add_Txchannel.argtypes = [
            ctypes.POINTER(ctypes.c_float),   # location
            ctypes.POINTER(ctypes.c_float),   # polar_real
            ctypes.POINTER(ctypes.c_float),   # polar_imag
            ctypes.POINTER(ctypes.c_float),   # phi
            ctypes.POINTER(ctypes.c_float),   # phi_ptn
            ctypes.c_int,                     # phi_length
            ctypes.POINTER(ctypes.c_float),   # theta
            ctypes.POINTER(ctypes.c_float),   # theta_ptn
            ctypes.c_int,                     # theta_length
            ctypes.c_float,                   # antenna_gain
            ctypes.POINTER(ctypes.c_float),   # mod_t
            ctypes.POINTER(ctypes.c_float),   # mod_var_real
            ctypes.POINTER(ctypes.c_float),   # mod_var_imag
            ctypes.c_int,                     # mod_length
            ctypes.POINTER(ctypes.c_float),   # pulse_mod_real
            ctypes.POINTER(ctypes.c_float),   # pulse_mod_imag
            ctypes.c_float,                   # delay
            ctypes.c_float,                   # grid
            ctypes.c_void_p                   # ptr_tx_c
        ]
        self.lib.Add_Txchannel.restype = ctypes.c_int
        
        self.lib.Free_Transmitter.argtypes = [ctypes.c_void_p]
        self.lib.Free_Transmitter.restype = None
        
        # Receiver functions
        self.lib.Create_Receiver.argtypes = [
            ctypes.c_float,  # fs
            ctypes.c_float,  # rf_gain
            ctypes.c_float,  # resistor
            ctypes.c_float,  # baseband_gain
            ctypes.c_float   # baseband_bw
        ]
        self.lib.Create_Receiver.restype = ctypes.c_void_p
        
        self.lib.Add_Rxchannel.argtypes = [
            ctypes.POINTER(ctypes.c_float),   # location
            ctypes.POINTER(ctypes.c_float),   # polar_real
            ctypes.POINTER(ctypes.c_float),   # polar_imag
            ctypes.POINTER(ctypes.c_float),   # phi
            ctypes.POINTER(ctypes.c_float),   # phi_ptn
            ctypes.c_int,                     # phi_length
            ctypes.POINTER(ctypes.c_float),   # theta
            ctypes.POINTER(ctypes.c_float),   # theta_ptn
            ctypes.c_int,                     # theta_length
            ctypes.c_float,                   # antenna_gain
            ctypes.c_void_p                   # ptr_rx_c
        ]
        self.lib.Add_Rxchannel.restype = ctypes.c_int
        
        self.lib.Free_Receiver.argtypes = [ctypes.c_void_p]
        self.lib.Free_Receiver.restype = None
        
        # Radar functions
        self.lib.Create_Radar.argtypes = [
            ctypes.c_void_p,                  # ptr_tx_c
            ctypes.c_void_p,                  # ptr_rx_c
            ctypes.POINTER(ctypes.c_double),  # frame_start_time
            ctypes.c_int,                     # num_frames
            ctypes.POINTER(ctypes.c_float),   # location
            ctypes.POINTER(ctypes.c_float),   # speed
            ctypes.POINTER(ctypes.c_float),   # rotation
            ctypes.POINTER(ctypes.c_float)    # rotation_rate
        ]
        self.lib.Create_Radar.restype = ctypes.c_void_p
        
        self.lib.Free_Radar.argtypes = [ctypes.c_void_p]
        self.lib.Free_Radar.restype = None
        
        # Target functions
        self.lib.Init_Targets.argtypes = []
        self.lib.Init_Targets.restype = ctypes.c_void_p
        
        self.lib.Add_Point_Target.argtypes = [
            ctypes.POINTER(ctypes.c_float),   # location
            ctypes.POINTER(ctypes.c_float),   # speed
            ctypes.c_float,                   # rcs
            ctypes.c_float,                   # phs
            ctypes.c_void_p                   # ptr_targets_c
        ]
        self.lib.Add_Point_Target.restype = ctypes.c_int
        
        self.lib.Free_Targets.argtypes = [ctypes.c_void_p]
        self.lib.Free_Targets.restype = None
        
        # Simulation function
        self.lib.Run_RadarSimulator.argtypes = [
            ctypes.c_void_p,                  # ptr_radar_c
            ctypes.c_void_p,                  # ptr_targets_c
            ctypes.c_int,                     # level
            ctypes.c_float,                   # density
            ctypes.POINTER(ctypes.c_int),     # ray_filter
            ctypes.POINTER(ctypes.c_double),  # ptr_bb_real
            ctypes.POINTER(ctypes.c_double)   # ptr_bb_imag
        ]
        self.lib.Run_RadarSimulator.restype = None
        
        # Cleanup functions
        self.lib.Force_Cleanup_All.argtypes = []
        self.lib.Force_Cleanup_All.restype = None
    
    def _setup_cleanup(self):
        """Set up automatic cleanup on exit"""
        import atexit
        atexit.register(self.cleanup)
    
    def get_version(self) -> Tuple[int, int, int]:
        """Get the RadarSim library version"""
        version_array = (ctypes.c_int * 3)()
        self.lib.Get_Version(version_array)
        return tuple(version_array)
    
    def create_fmcw_waveform(self, center_freq: float, bandwidth: float, 
                           duration: float, samples: int) -> Tuple[np.ndarray, np.ndarray]:
        """Create FMCW waveform arrays"""
        time = np.linspace(0, duration, samples)
        freq = center_freq - bandwidth/2 + bandwidth * time / duration
        return freq, time
    
    def create_antenna_pattern(self, num_points: int = 361, max_gain_db: float = 15.0) -> Tuple[np.ndarray, np.ndarray]:
        """Create simple antenna pattern"""
        angles = np.linspace(-np.pi, np.pi, num_points)
        pattern = max_gain_db * np.cos(angles/2)**2
        return angles.astype(np.float32), pattern.astype(np.float32)
    
    def run_example(self):
        """Run a complete radar simulation example"""
        print("RadarSim Python Example")
        print("=======================")
        
        # Get version
        version = self.get_version()
        print(f"RadarSim Library Version: {version[0]}.{version[1]}.{version[2]}")
        
        # Create waveform
        print("\\n1. Creating FMCW waveform...")
        center_freq = 77e9  # 77 GHz
        bandwidth = 4e9     # 4 GHz
        duration = 100e-6   # 100 microseconds
        samples = 1000
        num_pulses = 128
        
        freq, freq_time = self.create_fmcw_waveform(center_freq, bandwidth, duration, samples)
        freq_offset = np.zeros(num_pulses, dtype=np.float64)
        pulse_start_time = np.arange(num_pulses, dtype=np.float64) * 200e-6
        
        # Convert to ctypes arrays
        freq_c = freq.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        freq_time_c = freq_time.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        freq_offset_c = freq_offset.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        pulse_start_time_c = pulse_start_time.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        
        # Create transmitter
        print("2. Creating transmitter...")
        tx = self.lib.Create_Transmitter(freq_c, freq_time_c, samples,
                                        freq_offset_c, pulse_start_time_c,
                                        num_pulses, 20.0)
        if not tx:
            raise RadarSimError("Failed to create transmitter")
        
        # Create antenna patterns
        phi_angles, phi_pattern = self.create_antenna_pattern()
        theta_angles, theta_pattern = self.create_antenna_pattern()
        
        # Transmitter configuration
        tx_location = np.array([0.0, 0.0, 1.5], dtype=np.float32)
        tx_polar_real = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        tx_polar_imag = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        
        result = self.lib.Add_Txchannel(
            tx_location.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            tx_polar_real.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            tx_polar_imag.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            phi_angles.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            phi_pattern.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            len(phi_angles),
            theta_angles.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            theta_pattern.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            len(theta_angles),
            15.0,  # antenna gain
            None, None, None, 0,  # No modulation
            None, None,           # No pulse modulation
            0.0, 0.1,            # delay, grid
            tx
        )
        if result != 0:
            raise RadarSimError("Failed to add transmitter channel")
        
        # Create receiver
        print("3. Creating receiver...")
        rx = self.lib.Create_Receiver(10e6, 20.0, 50.0, 30.0, 5e6)
        if not rx:
            raise RadarSimError("Failed to create receiver")
        
        rx_location = np.array([0.1, 0.0, 1.5], dtype=np.float32)
        result = self.lib.Add_Rxchannel(
            rx_location.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            tx_polar_real.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            tx_polar_imag.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            phi_angles.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            phi_pattern.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            len(phi_angles),
            theta_angles.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            theta_pattern.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            len(theta_angles),
            15.0,  # antenna gain
            rx
        )
        if result != 0:
            raise RadarSimError("Failed to add receiver channel")
        
        # Create radar system
        print("4. Creating radar system...")
        frame_start_time = np.array([0.0], dtype=np.float64)
        radar_location = np.array([0.0, 0.0, 1.5], dtype=np.float32)
        radar_speed = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        radar_rotation = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        radar_rotation_rate = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        
        radar = self.lib.Create_Radar(
            tx, rx,
            frame_start_time.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            1,  # num_frames
            radar_location.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            radar_speed.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            radar_rotation.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            radar_rotation_rate.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        )
        if not radar:
            raise RadarSimError("Failed to create radar system")
        
        # Create targets
        print("5. Creating targets...")
        targets = self.lib.Init_Targets()
        if not targets:
            raise RadarSimError("Failed to initialize targets")
        
        # Add point target
        target_location = np.array([100.0, 0.0, 1.0], dtype=np.float32)
        target_speed = np.array([-20.0, 0.0, 0.0], dtype=np.float32)
        result = self.lib.Add_Point_Target(
            target_location.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            target_speed.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            10.0,  # RCS in dBsm
            0.0,   # phase
            targets
        )
        if result != 0:
            raise RadarSimError("Failed to add point target")
        
        # Run simulation
        print("6. Running radar simulation...")
        sampling_rate = 10e6
        samples_per_pulse = int(duration * sampling_rate)
        buffer_size = num_pulses * 1 * samples_per_pulse  # 1 rx channel
        
        bb_real = np.zeros(buffer_size, dtype=np.float64)
        bb_imag = np.zeros(buffer_size, dtype=np.float64)
        ray_filter = np.array([0, 1000], dtype=np.int32)
        
        self.lib.Run_RadarSimulator(
            radar, targets, 3, 1.0,
            ray_filter.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
            bb_real.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            bb_imag.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        )
        
        # Analyze results
        print("7. Analyzing results...")
        amplitude = np.sqrt(bb_real**2 + bb_imag**2)
        max_amplitude = np.max(amplitude)
        rms_amplitude = np.sqrt(np.mean(amplitude**2))
        
        print(f"   Buffer size: {buffer_size} samples")
        print(f"   Max amplitude: {max_amplitude:.6e}")
        print(f"   RMS amplitude: {rms_amplitude:.6e}")
        print(f"   Signal power: {np.sum(amplitude**2):.6e}")
        
        # Clean up
        print("8. Cleaning up...")
        self.lib.Free_Targets(targets)
        self.lib.Free_Radar(radar)
        self.lib.Free_Receiver(rx)
        self.lib.Free_Transmitter(tx)
        
        print("\\n✓ Example completed successfully!")
        return bb_real, bb_imag
    
    def cleanup(self):
        """Clean up library resources"""
        if hasattr(self, 'lib'):
            self.lib.Force_Cleanup_All()

def main():
    """Main function to run the example"""
    try:
        # Create RadarSim wrapper
        radar_sim = RadarSimWrapper()
        
        # Run example
        bb_real, bb_imag = radar_sim.run_example()
        
        # Optional: Plot results if matplotlib is available
        try:
            import matplotlib.pyplot as plt
            
            amplitude = np.sqrt(bb_real**2 + bb_imag**2)
            
            plt.figure(figsize=(12, 4))
            plt.subplot(1, 2, 1)
            plt.plot(bb_real[:1000])
            plt.title('Baseband Signal (Real Part)')
            plt.xlabel('Sample')
            plt.ylabel('Amplitude')
            
            plt.subplot(1, 2, 2)
            plt.plot(amplitude[:1000])
            plt.title('Signal Amplitude')
            plt.xlabel('Sample')
            plt.ylabel('Amplitude')
            
            plt.tight_layout()
            plt.savefig('radar_simulation_results.png', dpi=150)
            plt.show()
            print("Results plotted and saved as 'radar_simulation_results.png'")
            
        except ImportError:
            print("Matplotlib not available for plotting results")
            
    except RadarSimError as e:
        print(f"RadarSim Error: {e}")
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
