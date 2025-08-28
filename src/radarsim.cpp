/*
 * @file radarsim.cpp
 * @brief C wrapper implementation for RadarSimCpp library
 *
 * @details
 * C-compatible interface for RadarSimCpp library with opaque pointer pattern.
 * Provides radar simulation, target management, and automatic memory cleanup.
 *
 * Features:
 * - Transmitter/receiver configuration with antenna patterns
 * - Point and mesh target simulation
 * - RCS and LiDAR simulation
 * - Automatic memory management with cleanup registration
 * - Exception-safe resource handling
 *
 * @note Automatic cleanup provides safety net; manual Free_* calls recommended.
 * @warning Free tier has channel, target, and mesh complexity limits.
 *
 *    ----------
 *
 *    Copyright (C) 2023 - PRESENT  radarsimx.com
 *    E-mail: info@radarsimx.com
 *    Website: https://radarsimx.com
 *
 *    ██████╗  █████╗ ██████╗  █████╗ ██████╗ ███████╗██╗███╗   ███╗██╗  ██╗
 *    ██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔════╝██║████╗ ████║╚██╗██╔╝
 *    ██████╔╝███████║██║  ██║███████║██████╔╝███████╗██║██╔████╔██║ ╚███╔╝
 *    ██╔══██╗██╔══██║██║  ██║██╔══██║██╔══██╗╚════██║██║██║╚██╔╝██║ ██╔██╗
 *    ██║  ██║██║  ██║██████╔╝██║  ██║██║  ██║███████║██║██║ ╚═╝ ██║██╔╝ ██╗
 *    ╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝╚═╝╚═╝     ╚═╝╚═╝  ╚═╝
 *
 */

#include "radarsim.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#ifdef RADARSIM_THREAD_SAFE
#include <mutex>
#endif
#include <unordered_set>
#include <vector>

#include "libs/free_tier.hpp"
#include "point.hpp"
#include "points_manager.hpp"
#include "radar.hpp"
#include "receiver.hpp"
#include "simulator_interference.hpp"
#include "simulator_lidar.hpp"
#include "simulator_mesh.hpp"
#include "simulator_point.hpp"
#include "simulator_rcs.hpp"
#include "snapshot.hpp"
#include "targets_manager.hpp"
#include "transmitter.hpp"

/*********************************************
 *
 *  Internal Structures (Opaque Pointer Pattern)
 *
 *********************************************/
/**
 * @brief Internal structure for Transmitter C wrapper
 * @details Contains shared_ptr to Transmitter object with automatic cleanup.
 */
struct s_Transmitter {
  std::shared_ptr<Transmitter<double, float>> _ptr_transmitter;

  /**
   * @brief Destructor ensures proper cleanup of transmitter resources
   */
  ~s_Transmitter() {
    if (_ptr_transmitter) {
      _ptr_transmitter.reset();
    }
  }
};

/**
 * @brief Internal structure for Receiver C wrapper
 * @details Contains shared_ptr to Receiver object with automatic cleanup.
 */
struct s_Receiver {
  std::shared_ptr<Receiver<float>> _ptr_receiver;

  /**
   * @brief Destructor ensures proper cleanup of receiver resources
   */
  ~s_Receiver() {
    if (_ptr_receiver) {
      _ptr_receiver.reset();
    }
  }
};

/**
 * @brief Internal structure for Radar system C wrapper
 * @details Contains pointers to transmitter, receiver, and radar objects.
 */
struct s_Radar {
  t_Transmitter *_ptr_tx;
  t_Receiver *_ptr_rx;
  std::shared_ptr<Radar<double, float>> _ptr_radar;

  /**
   * @brief Destructor ensures proper cleanup of radar system resources
   */
  ~s_Radar() {
    if (_ptr_radar) {
      _ptr_radar.reset();
    }
  }
};

/**
 * @brief Internal structure for target management C wrapper
 * @details Contains shared_ptr to PointsManager and TargetsManager.
 */
struct s_Targets {
  std::shared_ptr<PointsManager<float>> _ptr_points;
  std::shared_ptr<TargetsManager<float>> _ptr_targets;

  /**
   * @brief Destructor ensures proper cleanup of target management resources
   */
  ~s_Targets() {
    if (_ptr_points) {
      _ptr_points.reset();
    }
    if (_ptr_targets) {
      _ptr_targets.reset();
    }
  }
};

/*********************************************
 *
 *  Automatic Cleanup Registry
 *
 *********************************************/

// Configuration: Choose cleanup implementation
// Option 1: RADARSIM_SIMPLE_CLEANUP - Simple array-based, no STL containers
// (maximum compatibility) Option 2: RADARSIM_THREAD_SAFE - Thread-safe with
// mutex (for multi-threaded apps) Option 3: Default - Lock-free with STL
// containers (good balance)

#ifdef RADARSIM_SIMPLE_CLEANUP

/**
 * @brief Simple automatic cleanup registry for maximum compatibility
 * @details Uses basic C-style arrays to avoid STL dependencies and locking.
 * Suitable for embedded systems and applications with strict requirements.
 */
class AutoCleanupRegistry {
 private:
  struct RegisteredObject {
    void *ptr;
    void (*cleanup_func)(void *);
  };

  static const int MAX_OBJECTS = 1000;  // Adjust as needed
  static RegisteredObject registered_objects[MAX_OBJECTS];
  static int object_count;
  static bool cleanup_in_progress;

 public:
  /**
   * @brief Register an object for automatic cleanup
   */
  static void register_object(void *ptr, void (*cleanup_func)(void *)) {
    if (!ptr || cleanup_in_progress || object_count >= MAX_OBJECTS) return;

    registered_objects[object_count].ptr = ptr;
    registered_objects[object_count].cleanup_func = cleanup_func;
    object_count++;
  }

  /**
   * @brief Unregister an object from automatic cleanup
   */
  static void unregister_object(void *ptr) {
    if (!ptr || cleanup_in_progress) return;

    for (int i = 0; i < object_count; i++) {
      if (registered_objects[i].ptr == ptr) {
        // Move last element to this position
        registered_objects[i] = registered_objects[object_count - 1];
        object_count--;
        break;
      }
    }
  }

  /**
   * @brief Clean up all registered objects
   */
  static void cleanup_all() {
    cleanup_in_progress = true;

    // Clean up in reverse order of registration
    for (int i = object_count - 1; i >= 0; i--) {
      try {
        registered_objects[i].cleanup_func(registered_objects[i].ptr);
      } catch (...) {
        // Continue cleanup even if one fails
      }
    }

    object_count = 0;
    cleanup_in_progress = false;
  }

  /**
   * @brief Check if cleanup is currently in progress
   */
  static bool is_cleanup_in_progress() { return cleanup_in_progress; }
};

// Static member definitions
AutoCleanupRegistry::RegisteredObject
    AutoCleanupRegistry::registered_objects[AutoCleanupRegistry::MAX_OBJECTS];
int AutoCleanupRegistry::object_count = 0;
bool AutoCleanupRegistry::cleanup_in_progress = false;

// Helper functions for simple cleanup
extern "C" {
static void cleanup_transmitter(void *ptr) {
  delete static_cast<t_Transmitter *>(ptr);
}
static void cleanup_receiver(void *ptr) {
  delete static_cast<t_Receiver *>(ptr);
}
static void cleanup_radar(void *ptr) { delete static_cast<t_Radar *>(ptr); }
static void cleanup_targets(void *ptr) { delete static_cast<t_Targets *>(ptr); }
}

#else

/**
 * @brief Automatic cleanup registry for memory management
 * @details Provides automatic cleanup of allocated resources when the DLL is
 * unloaded or the process exits. This serves as a safety net for clients who
 * forget to call the appropriate Free_* functions.
 *
 * Thread safety can be enabled by defining RADARSIM_THREAD_SAFE at compile
 * time. By default, this implementation is lock-free for maximum compatibility.
 */
class AutoCleanupRegistry {
 private:
  struct RegisteredObject {
    void *ptr;
    std::function<void()> cleanup_func;

    RegisteredObject(void *p, std::function<void()> func)
        : ptr(p), cleanup_func(func) {}
  };

  static std::vector<RegisteredObject> registered_objects;
#ifdef RADARSIM_THREAD_SAFE
  static std::mutex cleanup_mutex;
#endif
  static bool cleanup_in_progress;

 public:
  /**
   * @brief Register an object for automatic cleanup
   * @param ptr Pointer to the object to be cleaned up
   * @param cleanup_func Function to call for cleanup
   */
  static void register_object(void *ptr, std::function<void()> cleanup_func) {
    if (!ptr || cleanup_in_progress) return;

#ifdef RADARSIM_THREAD_SAFE
    std::lock_guard<std::mutex> lock(cleanup_mutex);
#endif
    registered_objects.emplace_back(ptr, cleanup_func);
  }

  /**
   * @brief Unregister an object from automatic cleanup
   * @param ptr Pointer to the object to unregister
   */
  static void unregister_object(void *ptr) {
    if (!ptr || cleanup_in_progress) return;

#ifdef RADARSIM_THREAD_SAFE
    std::lock_guard<std::mutex> lock(cleanup_mutex);
#endif
    registered_objects.erase(
        std::remove_if(
            registered_objects.begin(), registered_objects.end(),
            [ptr](const RegisteredObject &obj) { return obj.ptr == ptr; }),
        registered_objects.end());
  }

  /**
   * @brief Clean up all registered objects
   * @details Called automatically when DLL is unloaded or process exits
   */
  static void cleanup_all() {
#ifdef RADARSIM_THREAD_SAFE
    std::lock_guard<std::mutex> lock(cleanup_mutex);
#endif
    cleanup_in_progress = true;

    // Clean up in reverse order of registration
    for (auto it = registered_objects.rbegin(); it != registered_objects.rend();
         ++it) {
      try {
        it->cleanup_func();
      } catch (...) {
        // Log error but continue cleanup - don't let one failure stop others
        std::cerr << "Warning: Exception during automatic cleanup of object at "
                  << it->ptr << std::endl;
      }
    }

    registered_objects.clear();
    cleanup_in_progress = false;
  }

  /**
   * @brief Check if cleanup is currently in progress
   */
  static bool is_cleanup_in_progress() { return cleanup_in_progress; }
};

// Static member definitions
std::vector<AutoCleanupRegistry::RegisteredObject>
    AutoCleanupRegistry::registered_objects;
#ifdef RADARSIM_THREAD_SAFE
std::mutex AutoCleanupRegistry::cleanup_mutex;
#endif
bool AutoCleanupRegistry::cleanup_in_progress = false;

#endif  // RADARSIM_SIMPLE_CLEANUP

#ifdef _WIN32
/**
 * @brief DLL entry point for Windows
 */
BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call,
                      LPVOID lpReserved) {
  switch (ul_reason_for_call) {
    case DLL_PROCESS_ATTACH:
      // DLL is being loaded
      break;
    case DLL_PROCESS_DETACH:
      // DLL is being unloaded - perform automatic cleanup
      AutoCleanupRegistry::cleanup_all();
      break;
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
      // Thread events - no action needed
      break;
  }
  return TRUE;
}
#else
// Non-Windows platforms: Use constructor/destructor attributes if supported
#if defined(__GNUC__) || defined(__clang__)
/**
 * @brief Constructor function for non-Windows platforms
 */
__attribute__((constructor)) static void library_init() {
  // Library initialization if needed
}

/**
 * @brief Destructor function for non-Windows platforms
 */
__attribute__((destructor)) static void library_cleanup() {
  AutoCleanupRegistry::cleanup_all();
}
#else
// For compilers without constructor/destructor support,
// manual cleanup management is required
#warning \
    "Automatic cleanup not supported on this compiler. Manual cleanup required."
#endif
#endif

/*********************************************
 *
 *  Version
 *
 *********************************************/
/**
 * @brief Get the RadarSimLib version information
 *
 * @details Retrieves the current version numbers for the RadarSimLib library.
 * Version follows semantic versioning (major.minor.patch).
 *
 * @param[out] version Pre-allocated array to store version numbers {major,
 * minor, patch} Array must have at least 3 elements
 *
 * @note This function is thread-safe and has no failure conditions
 */
void Get_Version(int version[3]) {
  version[0] = VERSION_MAJOR;
  version[1] = VERSION_MINOR;
  version[2] = VERSION_PATCH;
}

/*********************************************
 *
 *  Memory Management
 *
 *********************************************/
/**
 * @brief Force cleanup of all automatically registered objects
 *
 * @details Manually triggers cleanup of all registered objects. Called
 * automatically on DLL unload. Use for testing or explicit cleanup timing.
 *
 * @note Thread-safe when RADARSIM_THREAD_SAFE is defined.
 * @warning All automatically managed objects become invalid after calling.
 */
void Force_Cleanup_All() { AutoCleanupRegistry::cleanup_all(); }

/**
 * @brief Check if automatic cleanup is currently in progress
 *
 * @return int 1 if cleanup is in progress, 0 otherwise
 */
int Is_Cleanup_In_Progress() {
  return AutoCleanupRegistry::is_cleanup_in_progress() ? 1 : 0;
}

/*********************************************
 *
 *  Transmitter
 *
 *********************************************/
/**
 * @brief Create a Transmitter object with waveform parameters
 *
 * @param[in] freq Frequency vector (Hz) - must not be NULL
 * @param[in] freq_time Timestamp vector for frequency samples (s) - must not be
 * NULL
 * @param[in] waveform_size Length of freq and freq_time arrays - must be > 0
 * @param[in] freq_offset Frequency offset per pulse (Hz) - must not be NULL
 * @param[in] pulse_start_time Pulse start time vector (s) - must not be NULL
 * @param[in] num_pulses Number of pulses - must be > 0
 * @param[in] tx_power Transmitter power (dBm)
 *
 * @return t_Transmitter* Pointer to Transmitter object, NULL on failure
 *
 * @note Automatically registered for cleanup. Use Free_Transmitter() for manual
 * cleanup.
 */
t_Transmitter *Create_Transmitter(double *freq, double *freq_time,
                                  int waveform_size, double *freq_offset,
                                  double *pulse_start_time, int num_pulses,
                                  float tx_power) {
  // Input validation
  if (freq == nullptr || freq_time == nullptr || waveform_size <= 0) {
    return nullptr;
  }
  if (freq_offset == nullptr || pulse_start_time == nullptr ||
      num_pulses <= 0) {
    return nullptr;
  }

  // Allocate memory for the wrapper struct
  t_Transmitter *ptr_tx_c = nullptr;

  try {
    // Use new instead of malloc for proper C++ object construction
    ptr_tx_c = new t_Transmitter();

    if (ptr_tx_c == nullptr) {
      return nullptr;
    }
    // Convert C arrays to C++ vectors
    std::vector<double> freq_vt;
    std::vector<double> freq_time_vt;
    freq_vt.reserve(waveform_size);
    freq_time_vt.reserve(waveform_size);
    for (int idx = 0; idx < waveform_size; idx++) {
      freq_vt.push_back(freq[idx]);
      freq_time_vt.push_back(freq_time[idx]);
    }

    std::vector<double> freq_offset_vt;
    std::vector<double> pulse_start_time_vt;
    freq_offset_vt.reserve(num_pulses);
    pulse_start_time_vt.reserve(num_pulses);
    for (int idx = 0; idx < num_pulses; idx++) {
      freq_offset_vt.push_back(freq_offset[idx]);
      pulse_start_time_vt.push_back(pulse_start_time[idx]);
    }

    // Create the Transmitter object with correct parameter order
    ptr_tx_c->_ptr_transmitter = std::make_shared<Transmitter<double, float>>(
        tx_power, freq_vt, freq_time_vt, freq_offset_vt, pulse_start_time_vt);

    // Register for automatic cleanup
#ifdef RADARSIM_SIMPLE_CLEANUP
    AutoCleanupRegistry::register_object(ptr_tx_c, cleanup_transmitter);
#else
    AutoCleanupRegistry::register_object(ptr_tx_c,
                                         [ptr_tx_c]() { delete ptr_tx_c; });
#endif

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Create_Transmitter: Memory allocation failed: " << e.what()
              << std::endl;
    delete ptr_tx_c;
    return nullptr;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to constructor
    std::cerr << "Create_Transmitter: Invalid argument: " << e.what()
              << std::endl;
    delete ptr_tx_c;
    return nullptr;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Create_Transmitter: Unexpected error: " << e.what()
              << std::endl;
    delete ptr_tx_c;
    return nullptr;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Create_Transmitter: Unknown error occurred" << std::endl;
    delete ptr_tx_c;
    return nullptr;
  }

  return ptr_tx_c;
}

/**
 * @brief Add a transmitter channel with antenna pattern and modulation
 *
 * @param[in] location Channel location {x, y, z} (m)
 * @param[in] polar_real Real part of polarization vector {x, y, z}
 * @param[in] polar_imag Imaginary part of polarization vector {x, y, z}
 * @param[in] phi Azimuth angles for radiation pattern (rad)
 * @param[in] phi_ptn Normalized phi pattern values (dB)
 * @param[in] phi_length Length of phi and phi_ptn arrays
 * @param[in] theta Elevation angles for radiation pattern (rad)
 * @param[in] theta_ptn Normalized theta pattern values (dB)
 * @param[in] theta_length Length of theta and theta_ptn arrays
 * @param[in] antenna_gain Maximum antenna gain (dB)
 * @param[in] mod_t Modulation timestamps (s)
 * @param[in] mod_var_real Real part of modulation amplitude
 * @param[in] mod_var_imag Imaginary part of modulation amplitude
 * @param[in] mod_length Length of modulation arrays (0 for no modulation)
 * @param[in] pulse_mod_real Real part of pulse modulation
 * @param[in] pulse_mod_imag Imaginary part of pulse modulation
 * @param[in] delay Transmitting delay (s)
 * @param[in] grid Ray occupancy grid resolution (rad)
 * @param[in] ptr_tx_c Pointer to the Transmitter object
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Free tier limited to 1 transmitter channel.
 */
int Add_Txchannel(float *location, float *polar_real, float *polar_imag,
                  float *phi, float *phi_ptn, int phi_length, float *theta,
                  float *theta_ptn, int theta_length, float antenna_gain,
                  float *mod_t, float *mod_var_real, float *mod_var_imag,
                  int mod_length, float *pulse_mod_real, float *pulse_mod_imag,
                  float delay, float grid, t_Transmitter *ptr_tx_c) {
  // Input validation - check for null transmitter pointer
  if (ptr_tx_c == nullptr) {
    return 1;
  }

  if (IsFreeTier() && ptr_tx_c->_ptr_transmitter->channel_size_ > 0) {
    return 1;
  }
  std::vector<float> phi_vt, phi_ptn_vt;
  phi_vt.reserve(phi_length);
  phi_ptn_vt.reserve(phi_length);
  for (int idx = 0; idx < phi_length; idx++) {
    phi_vt.push_back(phi[idx]);
    phi_ptn_vt.push_back(phi_ptn[idx]);
  }

  std::vector<float> theta_vt, theta_ptn_vt;
  theta_vt.reserve(theta_length);
  theta_ptn_vt.reserve(theta_length);
  for (int idx = 0; idx < theta_length; idx++) {
    theta_vt.push_back(theta[idx]);
    theta_ptn_vt.push_back(theta_ptn[idx]);
  }

  std::vector<float> mod_t_vt;
  std::vector<std::complex<float>> mod_var_vt;
  mod_t_vt.reserve(mod_length);
  mod_var_vt.reserve(mod_length);
  for (int idx = 0; idx < mod_length; idx++) {
    mod_t_vt.push_back(mod_t[idx]);
    mod_var_vt.push_back(
        std::complex<float>(mod_var_real[idx], mod_var_imag[idx]));
  }

  std::vector<std::complex<float>> pulse_mod_vt;
  pulse_mod_vt.reserve(ptr_tx_c->_ptr_transmitter->pulse_size_);
  for (int idx = 0; idx < ptr_tx_c->_ptr_transmitter->pulse_size_; idx++) {
    pulse_mod_vt.push_back(
        std::complex<float>(pulse_mod_real[idx], pulse_mod_imag[idx]));
  }

  rsv::Vec3<std::complex<float>> polar_complex = rsv::Vec3<std::complex<float>>(
      std::complex<float>(polar_real[0], polar_imag[0]),
      std::complex<float>(polar_real[1], polar_imag[1]),
      std::complex<float>(polar_real[2], polar_imag[2]));

  ptr_tx_c->_ptr_transmitter->AddChannel(
      rsv::Vec3<float>(location[0], location[1], location[2]), polar_complex,
      phi_vt, phi_ptn_vt, theta_vt, theta_ptn_vt, antenna_gain, mod_t_vt,
      mod_var_vt, pulse_mod_vt, delay, grid);
  return 0;
}

/**
 * @brief Get the number of configured transmitter channels
 *
 * @param[in] ptr_tx_c Pointer to the Transmitter object
 *
 * @return int Number of configured transmitter channels
 */
int Get_Num_Txchannel(t_Transmitter *ptr_tx_c) {
  return ptr_tx_c->_ptr_transmitter->channel_size_;
}

/**
 * @brief Safely release transmitter resources
 *
 * @param[in] ptr_tx_c Pointer to the Transmitter object to free (may be NULL)
 *
 * @note Automatically unregisters from cleanup system. Safe with NULL pointer.
 */
void Free_Transmitter(t_Transmitter *ptr_tx_c) {
  if (ptr_tx_c == nullptr) {
    return;
  }

  // Unregister from automatic cleanup
  AutoCleanupRegistry::unregister_object(ptr_tx_c);

  // shared_ptr automatically handles cleanup in destructor
  delete ptr_tx_c;
}

/*********************************************
 *
 *  Receiver
 *
 *********************************************/
/**
 * @brief Create a Receiver object with RF and baseband parameters
 *
 * @param[in] fs Sampling rate (Hz) - must be > 0
 * @param[in] rf_gain RF amplifier gain (dB)
 * @param[in] resistor Load resistor (Ohm) - must be > 0
 * @param[in] baseband_gain Baseband amplifier gain (dB)
 * @param[in] baseband_bw Baseband bandwidth (Hz) - must be > 0
 *
 * @return t_Receiver* Pointer to Receiver object, NULL on failure
 *
 * @note Automatically registered for cleanup. Use Free_Receiver() for manual
 * cleanup.
 */
t_Receiver *Create_Receiver(float fs, float rf_gain, float resistor,
                            float baseband_gain, float baseband_bw) {
  // Input validation
  if (fs <= 0 || resistor <= 0 || baseband_bw <= 0) {
    return nullptr;
  }

  t_Receiver *ptr_rx_c = nullptr;

  try {
    // Use new instead of malloc for proper C++ object construction
    ptr_rx_c = new t_Receiver();

    // Create the Receiver object using shared_ptr
    ptr_rx_c->_ptr_receiver = std::make_shared<Receiver<float>>(
        fs, rf_gain, resistor, baseband_gain, baseband_bw);

    // Register for automatic cleanup
#ifdef RADARSIM_SIMPLE_CLEANUP
    AutoCleanupRegistry::register_object(ptr_rx_c, cleanup_receiver);
#else
    AutoCleanupRegistry::register_object(ptr_rx_c,
                                         [ptr_rx_c]() { delete ptr_rx_c; });
#endif

    return ptr_rx_c;

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Create_Receiver: Memory allocation failed: " << e.what()
              << std::endl;
    delete ptr_rx_c;
    return nullptr;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to constructor
    std::cerr << "Create_Receiver: Invalid argument: " << e.what() << std::endl;
    delete ptr_rx_c;
    return nullptr;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Create_Receiver: Unexpected error: " << e.what() << std::endl;
    delete ptr_rx_c;
    return nullptr;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Create_Receiver: Unknown error occurred" << std::endl;
    delete ptr_rx_c;
    return nullptr;
  }
}

/**
 * @brief Add a receiver channel with antenna pattern configuration
 *
 * @param[in] location Channel location {x, y, z} (m)
 * @param[in] polar_real Real part of polarization vector {x, y, z}
 * @param[in] polar_imag Imaginary part of polarization vector {x, y, z}
 * @param[in] phi Azimuth angles for radiation pattern (rad)
 * @param[in] phi_ptn Normalized phi pattern values (dB)
 * @param[in] phi_length Length of phi and phi_ptn arrays
 * @param[in] theta Elevation angles for radiation pattern (rad)
 * @param[in] theta_ptn Normalized theta pattern values (dB)
 * @param[in] theta_length Length of theta and theta_ptn arrays
 * @param[in] antenna_gain Maximum antenna gain (dB)
 * @param[in] ptr_rx_c Pointer to the Receiver object
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Free tier limited to 1 receiver channel.
 */
int Add_Rxchannel(float *location, float *polar_real, float *polar_imag,
                  float *phi, float *phi_ptn, int phi_length, float *theta,
                  float *theta_ptn, int theta_length, float antenna_gain,
                  t_Receiver *ptr_rx_c) {
  // Input validation - check for null receiver pointer
  if (ptr_rx_c == nullptr) {
    return 1;
  }

  if (IsFreeTier() && ptr_rx_c->_ptr_receiver->channel_size_ > 0) {
    return 1;
  }
  std::vector<float> phi_vt, phi_ptn_vt;
  phi_vt.reserve(phi_length);
  phi_ptn_vt.reserve(phi_length);
  for (int idx = 0; idx < phi_length; idx++) {
    phi_vt.push_back(phi[idx]);
    phi_ptn_vt.push_back(phi_ptn[idx]);
  }

  std::vector<float> theta_vt, theta_ptn_vt;
  theta_vt.reserve(theta_length);
  theta_ptn_vt.reserve(theta_length);
  for (int idx = 0; idx < theta_length; idx++) {
    theta_vt.push_back(theta[idx]);
    theta_ptn_vt.push_back(theta_ptn[idx]);
  }

  rsv::Vec3<std::complex<float>> polar_complex = rsv::Vec3<std::complex<float>>(
      std::complex<float>(polar_real[0], polar_imag[0]),
      std::complex<float>(polar_real[1], polar_imag[1]),
      std::complex<float>(polar_real[2], polar_imag[2]));

  ptr_rx_c->_ptr_receiver->AddChannel(
      rsv::Vec3<float>(location[0], location[1], location[2]), polar_complex,
      phi_vt, phi_ptn_vt, theta_vt, theta_ptn_vt, antenna_gain);
  return 0;
}

/**
 * @brief Get the number of configured receiver channels
 *
 * @param[in] ptr_rx_c Pointer to the Receiver object
 *
 * @return int Number of configured receiver channels
 */
int Get_Num_Rxchannel(t_Receiver *ptr_rx_c) {
  return ptr_rx_c->_ptr_receiver->channel_size_;
}

/**
 * @brief Safely release receiver resources
 *
 * @param[in] ptr_rx_c Pointer to the Receiver object to free (may be NULL)
 *
 * @note Automatically unregisters from cleanup system. Safe with NULL pointer.
 */
void Free_Receiver(t_Receiver *ptr_rx_c) {
  if (ptr_rx_c == nullptr) {
    return;
  }

  // Unregister from automatic cleanup
  AutoCleanupRegistry::unregister_object(ptr_rx_c);

  // shared_ptr automatically handles cleanup in destructor
  delete ptr_rx_c;
}

/*********************************************
 *
 *  Radar
 *
 *********************************************/
/**
 * @brief Create a complete Radar system from transmitter and receiver
 *
 * @param[in] ptr_tx_c Pointer to configured Transmitter object
 * @param[in] ptr_rx_c Pointer to configured Receiver object
 * @param[in] frame_start_time Frame start time vector (s)
 * @param[in] num_frames Number of radar frames - must be > 0
 * @param[in] location Initial radar platform location {x, y, z} (m)
 * @param[in] speed Radar platform velocity {x, y, z} (m/s)
 * @param[in] rotation Initial radar platform orientation {x, y, z} (rad)
 * @param[in] rotation_rate Radar platform angular velocity {x, y, z} (rad/s)
 *
 * @return t_Radar* Pointer to Radar system object, NULL on failure
 *
 * @note Automatically registered for cleanup. Use Free_Radar() for manual
 * cleanup.
 * @warning TX/RX objects must remain valid for radar system's lifetime.
 */
t_Radar *Create_Radar(t_Transmitter *ptr_tx_c, t_Receiver *ptr_rx_c,
                      double *frame_start_time, int num_frames, float *location,
                      float *speed, float *rotation, float *rotation_rate) {
  // Input validation
  if (ptr_tx_c == nullptr || ptr_rx_c == nullptr ||
      frame_start_time == nullptr || num_frames <= 0 || location == nullptr ||
      speed == nullptr || rotation == nullptr || rotation_rate == nullptr) {
    return nullptr;
  }

  // Allocate memory for the wrapper struct
  t_Radar *ptr_radar_c = nullptr;

  try {
    // Use new instead of malloc for proper C++ object construction
    ptr_radar_c = new t_Radar();
    if (ptr_radar_c == nullptr) {
      return nullptr;
    }

    ptr_radar_c->_ptr_tx = ptr_tx_c;
    ptr_radar_c->_ptr_rx = ptr_rx_c;

    // Convert C arrays to C++ vectors
    std::vector<double> frame_start_time_vt;
    frame_start_time_vt.reserve(num_frames);
    for (int idx = 0; idx < num_frames; idx++) {
      frame_start_time_vt.push_back(frame_start_time[idx]);
    }

    std::vector<rsv::Vec3<float>> loc_vt, rot_vt;
    loc_vt.push_back(rsv::Vec3<float>(location[0], location[1], location[2]));
    rot_vt.push_back(rsv::Vec3<float>(rotation[0], rotation[1], rotation[2]));

    // Create the Radar object using shared_ptr and pass shared_ptr objects
    ptr_radar_c->_ptr_radar = std::make_shared<Radar<double, float>>(
        ptr_tx_c->_ptr_transmitter, ptr_rx_c->_ptr_receiver,
        frame_start_time_vt, loc_vt,
        rsv::Vec3<float>(speed[0], speed[1], speed[2]), rot_vt,
        rsv::Vec3<float>(rotation_rate[0], rotation_rate[1], rotation_rate[2]));

    // Register for automatic cleanup
#ifdef RADARSIM_SIMPLE_CLEANUP
    AutoCleanupRegistry::register_object(ptr_radar_c, cleanup_radar);
#else
    AutoCleanupRegistry::register_object(
        ptr_radar_c, [ptr_radar_c]() { delete ptr_radar_c; });
#endif

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Create_Radar: Memory allocation failed: " << e.what()
              << std::endl;
    delete ptr_radar_c;
    return nullptr;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to constructor
    std::cerr << "Create_Radar: Invalid argument: " << e.what() << std::endl;
    delete ptr_radar_c;
    return nullptr;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Create_Radar: Unexpected error: " << e.what() << std::endl;
    delete ptr_radar_c;
    return nullptr;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Create_Radar: Unknown error occurred" << std::endl;
    delete ptr_radar_c;
    return nullptr;
  }

  return ptr_radar_c;
}

/**
 * @brief Safely release radar system resources
 *
 * @param[in] ptr_radar_c Pointer to the Radar system object to free (may be
 * NULL)
 *
 * @note TX/RX objects are NOT freed - manage separately. Auto-unregisters from
 * cleanup.
 */
void Free_Radar(t_Radar *ptr_radar_c) {
  if (ptr_radar_c == nullptr) {
    return;
  }

  // Unregister from automatic cleanup
  AutoCleanupRegistry::unregister_object(ptr_radar_c);

  // shared_ptr automatically handles cleanup in destructor
  delete ptr_radar_c;
}

/*********************************************
 *
 *  Targets
 *
 *********************************************/
/**
 * @brief Initialize the target management system
 *
 * @return t_Targets* Pointer to target management system, NULL on failure
 *
 * @note Automatically registered for cleanup. Use Free_Targets() for manual
 * cleanup.
 * @note Use Add_Point_Target() and Add_Mesh_Target() to populate targets.
 */
t_Targets *Init_Targets() {
  t_Targets *ptr_targets_c = nullptr;

  try {
    // Use new instead of malloc for proper C++ object construction
    ptr_targets_c = new t_Targets();
    if (ptr_targets_c == nullptr) {
      return nullptr;
    }

    // Create the manager objects using shared_ptr
    ptr_targets_c->_ptr_points = std::make_shared<PointsManager<float>>();
    ptr_targets_c->_ptr_targets = std::make_shared<TargetsManager<float>>();

    // Register for automatic cleanup
#ifdef RADARSIM_SIMPLE_CLEANUP
    AutoCleanupRegistry::register_object(ptr_targets_c, cleanup_targets);
#else
    AutoCleanupRegistry::register_object(
        ptr_targets_c, [ptr_targets_c]() { delete ptr_targets_c; });
#endif

    return ptr_targets_c;

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Init_Targets: Memory allocation failed: " << e.what()
              << std::endl;
    delete ptr_targets_c;
    return nullptr;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to constructor
    std::cerr << "Init_Targets: Invalid argument: " << e.what() << std::endl;
    delete ptr_targets_c;
    return nullptr;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Init_Targets: Unexpected error: " << e.what() << std::endl;
    delete ptr_targets_c;
    return nullptr;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Init_Targets: Unknown error occurred" << std::endl;
    delete ptr_targets_c;
    return nullptr;
  }

  return ptr_targets_c;
}

/**
 * @brief Add an ideal point scatterer to the simulation
 *
 * @param[in] location Target's initial location {x, y, z} (m)
 * @param[in] speed Target's velocity vector {x, y, z} (m/s)
 * @param[in] rcs Target's radar cross section (dBsm)
 * @param[in] phs Target's initial phase (rad)
 * @param[in] ptr_targets_c Pointer to the target management system
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Free tier limited to 2 point targets.
 */
int Add_Point_Target(float *location, float *speed, float rcs, float phs,
                     t_Targets *ptr_targets_c) {
  // Input validation - check for null pointers
  if (ptr_targets_c == nullptr || location == nullptr || speed == nullptr) {
    return 1;
  }

  if (IsFreeTier() && ptr_targets_c->_ptr_points->vect_points_.size() > 1) {
    return 1;
  }
  ptr_targets_c->_ptr_points->AddPointSimple(
      rsv::Vec3<float>(location[0], location[1], location[2]),
      rsv::Vec3<float>(speed[0], speed[1], speed[2]), rcs, phs);
  return 0;
}

/**
 * @brief Add a complex 3D mesh target to the simulation
 *
 * @param[in] points Mesh vertex coordinates array [x₁,y₁,z₁, x₂,y₂,z₂, ...]
 * @param[in] cells Triangle connectivity array [v₁,v₂,v₃, ...] (0-indexed)
 * @param[in] cell_size Number of triangular mesh faces
 * @param[in] origin Target's local coordinate origin (m)
 * @param[in] location Target's initial location {x, y, z} (m)
 * @param[in] speed Target's velocity vector {x, y, z} (m/s)
 * @param[in] rotation Target's initial orientation {x, y, z} (rad)
 * @param[in] rotation_rate Target's angular velocity {x, y, z} (rad/s)
 * @param[in] ep_real Real part of relative permittivity εᵣ
 * @param[in] ep_imag Imaginary part of relative permittivity εᵣ
 * @param[in] mu_real Real part of relative permeability μᵣ
 * @param[in] mu_imag Imaginary part of relative permeability μᵣ
 * @param[in] is_ground Flag for ground surface (affects ray tracing)
 * @param[in] ptr_targets_c Pointer to the target management system
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Free tier limits: 2 mesh targets max, 8 triangles per mesh max.
 */
int Add_Mesh_Target(float *points, int *cells, int cell_size, float *origin,
                    float *location, float *speed, float *rotation,
                    float *rotation_rate, float ep_real, float ep_imag,
                    float mu_real, float mu_imag, bool is_ground,
                    t_Targets *ptr_targets_c) {
  // Input validation - check for null pointers and invalid parameters
  if (ptr_targets_c == nullptr || points == nullptr || cells == nullptr ||
      cell_size <= 0 || origin == nullptr || location == nullptr ||
      speed == nullptr || rotation == nullptr || rotation_rate == nullptr) {
    return 1;
  }

  if (IsFreeTier() && ptr_targets_c->_ptr_targets->vect_targets_.size() > 1) {
    return 1;
  }

  if (IsFreeTier() && cell_size > 8) {
    return 1;
  }

  // Create single-element arrays for the time-varying parameters
  std::vector<rsv::Vec3<float>> location_array;
  location_array.push_back(
      rsv::Vec3<float>(location[0], location[1], location[2]));

  std::vector<rsv::Vec3<float>> speed_array;
  speed_array.push_back(rsv::Vec3<float>(speed[0], speed[1], speed[2]));

  std::vector<rsv::Vec3<float>> rotation_array;
  rotation_array.push_back(
      rsv::Vec3<float>(rotation[0], rotation[1], rotation[2]));

  std::vector<rsv::Vec3<float>> rotrate_array;
  rotrate_array.push_back(
      rsv::Vec3<float>(rotation_rate[0], rotation_rate[1], rotation_rate[2]));

  std::complex<float> ep = std::complex(ep_real, ep_imag);
  std::complex<float> mu = std::complex(mu_real, mu_imag);

  ptr_targets_c->_ptr_targets->AddTarget(
      points, cells, cell_size,
      rsv::Vec3<float>(origin[0], origin[1], origin[2]), location_array,
      speed_array, rotation_array, rotrate_array, ep, mu, is_ground);
  return 0;
}

/**
 * @brief Safely release target management system resources
 *
 * @param[in] ptr_targets_c Pointer to the target management system to free (may
 * be NULL)
 *
 * @note Auto-unregisters from cleanup system. Safe with NULL pointer.
 */
void Free_Targets(t_Targets *ptr_targets_c) {
  if (ptr_targets_c == nullptr) {
    return;
  }

  // Unregister from automatic cleanup
  AutoCleanupRegistry::unregister_object(ptr_targets_c);

  // shared_ptr automatically handles cleanup in destructor
  delete ptr_targets_c;
}

/*********************************************
 *
 *  Simulator
 *
 *********************************************/
/**
 * @brief Execute comprehensive radar simulation for all configured targets
 *
 * @param[in] ptr_radar_c Pointer to the radar system
 * @param[in] ptr_targets_c Pointer to the target management system
 * @param[in] level Ray tracing quality level for mesh targets (1-5)
 * @param[in] density Ray density for mesh simulation (rays per wavelength²)
 * @param[in] ray_filter Ray filter range {min_range, max_range} (m)
 * @param[out] ptr_bb_real Real part of baseband signal buffer (pre-allocated)
 * @param[out] ptr_bb_imag Imaginary part of baseband signal buffer
 * (pre-allocated)
 *
 * @note Buffer size: [num_pulses × num_rx_channels × samples_per_pulse]
 * @warning Buffers must be pre-allocated to correct size.
 */
void Run_RadarSimulator(t_Radar *ptr_radar_c, t_Targets *ptr_targets_c,
                        int level, float density, int *ray_filter,
                        double *ptr_bb_real, double *ptr_bb_imag) {
  ptr_radar_c->_ptr_radar->InitBaseband(ptr_bb_real, ptr_bb_imag);
  if (ptr_targets_c->_ptr_points->vect_points_.size() > 0) {
    PointSimulator<double, float> simc = PointSimulator<double, float>();

    simc.Run(ptr_radar_c->_ptr_radar, ptr_targets_c->_ptr_points);
  }

  if (ptr_targets_c->_ptr_targets->vect_targets_.size() > 0) {
    MeshSimulator<double, float> scene_c = MeshSimulator<double, float>();

    rsv::Vec2<int> ray_filter_vec2 =
        rsv::Vec2<int>(ray_filter[0], ray_filter[1]);

    scene_c.Run(ptr_radar_c->_ptr_radar, ptr_targets_c->_ptr_targets, level,
                density, ray_filter_vec2, false, "", false);
  }
  ptr_radar_c->_ptr_radar->SyncBaseband();
}

/**
 * @brief Execute radar-to-radar interference simulation
 *
 * @param[in] ptr_radar_c Pointer to the victim radar system
 * @param[in] ptr_interf_radar_c Pointer to the interfering radar system
 * @param[out] ptr_interf_real Real part of interference signal buffer
 * (pre-allocated)
 * @param[out] ptr_interf_imag Imaginary part of interference signal buffer
 * (pre-allocated)
 *
 * @note Buffer size: [num_pulses × num_rx_channels × samples_per_pulse]
 * @warning Buffers must match victim radar's baseband dimensions.
 */
void Run_InterferenceSimulator(t_Radar *ptr_radar_c,
                               t_Radar *ptr_interf_radar_c,
                               double *ptr_interf_real,
                               double *ptr_interf_imag) {
  InterferenceSimulator<double, float> simc =
      InterferenceSimulator<double, float>();
  ptr_radar_c->_ptr_radar->InitBaseband(ptr_interf_real, ptr_interf_imag);
  simc.Run(ptr_radar_c->_ptr_radar, ptr_interf_radar_c->_ptr_radar);
  ptr_radar_c->_ptr_radar->SyncBaseband();
}

/**
 * @brief Execute Radar Cross Section (RCS) simulation using Physical Optics
 *
 * @param[in] ptr_targets_c Pointer to target management system with mesh
 * targets
 * @param[in] inc_dir_array Incident direction vectors [x₁,y₁,z₁, x₂,y₂,z₂, ...]
 * @param[in] obs_dir_array Observation direction vectors [x₁,y₁,z₁, x₂,y₂,z₂,
 * ...]
 * @param[in] num_directions Number of direction pairs for RCS calculation
 * @param[in] inc_polar_real Real part of incident polarization vector {x, y, z}
 * @param[in] inc_polar_imag Imaginary part of incident polarization vector {x,
 * y, z}
 * @param[in] obs_polar_real Real part of observation polarization vector {x, y,
 * z}
 * @param[in] obs_polar_imag Imaginary part of observation polarization vector
 * {x, y, z}
 * @param[in] frequency Electromagnetic frequency (Hz)
 * @param[in] density Ray density for Physical Optics (rays per wavelength²)
 * @param[out] rcs_result Output array for RCS values (m²) - pre-allocated
 *
 * @return int 0 for success, 1 for failure
 *
 * @note Higher density = more accurate but slower computation.
 */
int Run_RcsSimulator(t_Targets *ptr_targets_c, double *inc_dir_array,
                     double *obs_dir_array, int num_directions,
                     double *inc_polar_real, double *inc_polar_imag,
                     double *obs_polar_real, double *obs_polar_imag,
                     double frequency, double density, double *rcs_result) {
  // Input validation
  if (ptr_targets_c == nullptr || inc_dir_array == nullptr ||
      obs_dir_array == nullptr || num_directions <= 0 ||
      inc_polar_real == nullptr || inc_polar_imag == nullptr ||
      obs_polar_real == nullptr || obs_polar_imag == nullptr ||
      frequency <= 0 || density <= 0 || rcs_result == nullptr) {
    return 1;
  }

  try {
    // Create RCS simulator
    RcsSimulator<double> rcs_sim;

    // Convert C arrays to C++ vectors for incident directions
    std::vector<rsv::Vec3<double>> inc_dir_vect;
    inc_dir_vect.reserve(num_directions);
    for (int i = 0; i < num_directions; i++) {
      inc_dir_vect.push_back(rsv::Vec3<double>(inc_dir_array[i * 3],
                                               inc_dir_array[i * 3 + 1],
                                               inc_dir_array[i * 3 + 2]));
    }

    // Convert C arrays to C++ vectors for observation directions
    std::vector<rsv::Vec3<double>> obs_dir_vect;
    obs_dir_vect.reserve(num_directions);
    for (int i = 0; i < num_directions; i++) {
      obs_dir_vect.push_back(rsv::Vec3<double>(obs_dir_array[i * 3],
                                               obs_dir_array[i * 3 + 1],
                                               obs_dir_array[i * 3 + 2]));
    }

    // Create complex polarization vectors
    rsv::Vec3<std::complex<double>> inc_polar(
        std::complex<double>(inc_polar_real[0], inc_polar_imag[0]),
        std::complex<double>(inc_polar_real[1], inc_polar_imag[1]),
        std::complex<double>(inc_polar_real[2], inc_polar_imag[2]));

    rsv::Vec3<std::complex<double>> obs_polar(
        std::complex<double>(obs_polar_real[0], obs_polar_imag[0]),
        std::complex<double>(obs_polar_real[1], obs_polar_imag[1]),
        std::complex<double>(obs_polar_real[2], obs_polar_imag[2]));

    // Run RCS simulation
    std::vector<double> rcs_values =
        rcs_sim.Run(ptr_targets_c->_ptr_targets, inc_dir_vect, obs_dir_vect,
                    inc_polar, obs_polar, frequency, density);

    // Copy results back to C array
    for (size_t i = 0;
         i < rcs_values.size() && i < static_cast<size_t>(num_directions);
         i++) {
      rcs_result[i] = rcs_values[i];
    }

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Run_RcsSimulator: Memory allocation failed: " << e.what()
              << std::endl;
    return 1;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to simulator
    std::cerr << "Run_RcsSimulator: Invalid argument: " << e.what()
              << std::endl;
    return 1;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Run_RcsSimulator: Unexpected error: " << e.what()
              << std::endl;
    return 1;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Run_RcsSimulator: Unknown error occurred" << std::endl;
    return 1;
  }

  return 0;
}

/**
 * @brief Execute LiDAR point cloud simulation using ray tracing
 *
 * @param[in] ptr_targets_c Pointer to target management system with mesh
 * targets
 * @param[in] phi_array Azimuth angle array (rad) - range: [0, 2π]
 * @param[in] theta_array Elevation angle array (rad) - range: [0, π]
 * @param[in] num_rays Number of rays to shoot
 * @param[in] sensor_location LiDAR sensor position {x, y, z} (m)
 * @param[out] cloud_points Output point cloud coordinates [x₁,y₁,z₁, x₂,y₂,z₂,
 * ...] (pre-allocated)
 * @param[out] cloud_distances Output distances from sensor (m) (pre-allocated)
 * @param[out] cloud_intensities Output point intensities (pre-allocated)
 * @param[in] max_points Maximum number of points to return
 * @param[out] actual_points Actual number of points found and returned
 *
 * @return int 0 for success, 1 for failure
 *
 * @note φ=0 is +X axis, θ=0 is +Z axis. Only first-surface intersections
 * recorded.
 * @warning Output arrays must be allocated for at least max_points elements.
 */
int Run_LidarSimulator(t_Targets *ptr_targets_c, double *phi_array,
                       double *theta_array, int num_rays,
                       double *sensor_location, double *cloud_points,
                       double *cloud_distances, double *cloud_intensities,
                       int max_points, int *actual_points) {
  // Input validation
  if (ptr_targets_c == nullptr || phi_array == nullptr ||
      theta_array == nullptr || num_rays <= 0 || sensor_location == nullptr ||
      cloud_points == nullptr || cloud_distances == nullptr ||
      cloud_intensities == nullptr || max_points <= 0 ||
      actual_points == nullptr) {
    return 1;
  }

  try {
    // Create LiDAR simulator
    LidarSimulator<float> lidar_sim;

    // Convert C arrays to C++ vectors
    std::vector<float> phi_vect;
    std::vector<float> theta_vect;
    phi_vect.reserve(num_rays);
    theta_vect.reserve(num_rays);

    for (int i = 0; i < num_rays; i++) {
      phi_vect.push_back(static_cast<float>(phi_array[i]));
      theta_vect.push_back(static_cast<float>(theta_array[i]));
    }

    // Create sensor position vector
    rsv::Vec3<float> location(static_cast<float>(sensor_location[0]),
                              static_cast<float>(sensor_location[1]),
                              static_cast<float>(sensor_location[2]));

    // Run LiDAR simulation
    lidar_sim.Run(ptr_targets_c->_ptr_targets, phi_vect, theta_vect, location);

    // Extract results from point cloud
    int point_count = 0;
    for (size_t i = 0; i < lidar_sim.cloud_.size() && point_count < max_points;
         i++) {
      const auto &ray = lidar_sim.cloud_[i];

      // Calculate hit point coordinates using the final location from the ray
      // arrays The final hit point is at location_[reflections_] and uses array
      // indexing [0], [1], [2]
      cloud_points[point_count * 3] =
          static_cast<double>(ray.location_[ray.reflections_][0]);
      cloud_points[point_count * 3 + 1] =
          static_cast<double>(ray.location_[ray.reflections_][1]);
      cloud_points[point_count * 3 + 2] =
          static_cast<double>(ray.location_[ray.reflections_][2]);

      // Store distance using the final range
      cloud_distances[point_count] =
          static_cast<double>(ray.range_[ray.reflections_]);

      // Store intensity (use 1.0 as default since Ray doesn't have amplitude)
      cloud_intensities[point_count] = 1.0;

      point_count++;
    }

    // Return actual number of points found
    *actual_points = point_count;

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Run_LidarSimulator: Memory allocation failed: " << e.what()
              << std::endl;
    return 1;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to simulator
    std::cerr << "Run_LidarSimulator: Invalid argument: " << e.what()
              << std::endl;
    return 1;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Run_LidarSimulator: Unexpected error: " << e.what()
              << std::endl;
    return 1;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Run_LidarSimulator: Unknown error occurred" << std::endl;
    return 1;
  }

  return 0;
}

/*********************************************
 *
 *  main
 *
 *********************************************/
// int main() {
//   double f[] =
//   {2.40750000e+10, 2.40760901e+10, 2.40771786e+10, 2.40782654e+10,
//                 2.40793506e+10, 2.40804341e+10, 2.40815161e+10, 2.40825964e+10,
//                 2.40836750e+10, 2.40847521e+10, 2.40858275e+10, 2.40869012e+10,
//                 2.40879734e+10, 2.40890439e+10, 2.40901127e+10, 2.40911800e+10,
//                 2.40922456e+10, 2.40933096e+10, 2.40943719e+10, 2.40954326e+10,
//                 2.40964917e+10, 2.40975491e+10, 2.40986049e+10, 2.40996591e+10,
//                 2.41007117e+10, 2.41017626e+10, 2.41028119e+10, 2.41038595e+10,
//                 2.41049055e+10, 2.41059499e+10, 2.41069927e+10, 2.41080338e+10,
//                 2.41090733e+10, 2.41101111e+10, 2.41111473e+10, 2.41121819e+10,
//                 2.41132149e+10, 2.41142462e+10, 2.41152759e+10, 2.41163039e+10,
//                 2.41173304e+10, 2.41183552e+10, 2.41193783e+10, 2.41203999e+10,
//                 2.41214198e+10, 2.41224380e+10, 2.41234546e+10, 2.41244696e+10,
//                 2.41254830e+10, 2.41264947e+10, 2.41275048e+10, 2.41285133e+10,
//                 2.41295202e+10, 2.41305254e+10, 2.41315289e+10, 2.41325309e+10,
//                 2.41335312e+10, 2.41345298e+10, 2.41355269e+10, 2.41365223e+10,
//                 2.41375161e+10, 2.41385082e+10, 2.41394987e+10, 2.41404876e+10,
//                 2.41414748e+10, 2.41424605e+10, 2.41434444e+10, 2.41444268e+10,
//                 2.41454075e+10, 2.41463866e+10, 2.41473640e+10, 2.41483399e+10,
//                 2.41493140e+10, 2.41502866e+10, 2.41512575e+10, 2.41522268e+10,
//                 2.41531945e+10, 2.41541605e+10, 2.41551249e+10, 2.41560876e+10,
//                 2.41570488e+10, 2.41580083e+10, 2.41589661e+10, 2.41599224e+10,
//                 2.41608770e+10, 2.41618299e+10, 2.41627812e+10, 2.41637309e+10,
//                 2.41646790e+10, 2.41656254e+10, 2.41665702e+10, 2.41675134e+10,
//                 2.41684550e+10, 2.41693949e+10, 2.41703331e+10, 2.41712698e+10,
//                 2.41722048e+10, 2.41731381e+10, 2.41740699e+10, 2.41750000e+10};
//   double t[] =
//   {0, 8.08080808080808e-07, 1.61616161616162e-06, 2.42424242424242e-06, 3.23232323232323e-06,
//   4.04040404040404e-06, 4.84848484848485e-06, 5.65656565656566e-06, 6.46464646464647e-06,
//   7.27272727272727e-06, 8.08080808080808e-06, 8.88888888888889e-06, 9.69696969696970e-06,
//   1.05050505050505e-05, 1.13131313131313e-05, 1.21212121212121e-05, 1.29292929292929e-05,
//   1.37373737373737e-05, 1.45454545454545e-05, 1.53535353535354e-05, 1.61616161616162e-05,
//   1.69696969696970e-05, 1.77777777777778e-05, 1.85858585858586e-05, 1.93939393939394e-05,
//   2.02020202020202e-05, 2.10101010101010e-05, 2.18181818181818e-05, 2.26262626262626e-05,
//   2.34343434343434e-05, 2.42424242424242e-05, 2.50505050505051e-05, 2.58585858585859e-05,
//   2.66666666666667e-05, 2.74747474747475e-05, 2.82828282828283e-05, 2.90909090909091e-05,
//   2.98989898989899e-05, 3.07070707070707e-05, 3.15151515151515e-05, 3.23232323232323e-05,
//   3.31313131313131e-05, 3.39393939393939e-05, 3.47474747474748e-05, 3.55555555555556e-05,
//   3.63636363636364e-05, 3.71717171717172e-05, 3.79797979797980e-05, 3.87878787878788e-05,
//   3.95959595959596e-05, 4.04040404040404e-05, 4.12121212121212e-05, 4.20202020202020e-05,
//   4.28282828282828e-05, 4.36363636363636e-05, 4.44444444444444e-05, 4.52525252525253e-05,
//   4.60606060606061e-05, 4.68686868686869e-05, 4.76767676767677e-05, 4.84848484848485e-05,
//   4.92929292929293e-05, 5.01010101010101e-05, 5.09090909090909e-05, 5.17171717171717e-05,
//   5.25252525252525e-05, 5.33333333333333e-05, 5.41414141414141e-05, 5.49494949494950e-05,
//   5.57575757575758e-05, 5.65656565656566e-05, 5.73737373737374e-05, 5.81818181818182e-05,
//   5.89898989898990e-05, 5.97979797979798e-05, 6.06060606060606e-05, 6.14141414141414e-05,
//   6.22222222222222e-05, 6.30303030303030e-05, 6.38383838383838e-05, 6.46464646464647e-05,
//   6.54545454545455e-05, 6.62626262626263e-05, 6.70707070707071e-05, 6.78787878787879e-05,
//   6.86868686868687e-05, 6.94949494949495e-05, 7.03030303030303e-05, 7.11111111111111e-05,
//   7.19191919191919e-05, 7.27272727272727e-05, 7.35353535353536e-05, 7.43434343434344e-05,
//   7.51515151515152e-05, 7.59595959595960e-05, 7.67676767676768e-05, 7.75757575757576e-05,
//   7.83838383838384e-05, 7.91919191919192e-05, 8.00000000000000e-05};

//   double freq_offset[256];
//   for (int idx = 0; idx < 256; idx++) {
//     freq_offset[idx] = 0.0;
//   }

//   double pulse_start_time[256];
//   for (int idx = 0; idx < 256; idx++) {
//     pulse_start_time[idx] = 100e-6 * idx;
//   }

//   double frame_start_time[] = {0};

//   t_Transmitter *tx_ptr = Create_Transmitter(
//       f, t, 100, freq_offset, pulse_start_time, 256, frame_start_time,
//       1, 60.0);

//   float location[] = {0, 0, 0};
//   float polar[] = {0, 0, 1};
//   float phi[] = {-90.0/180.0*kPI, 90.0/180.0*kPI};
//   float phi_ptn[] = {0, 0};
//   float theta[] = {0, 180.0/180.0*kPI};
//   float theta_ptn[] = {0, 0};
//   float antenna_gain = 0;
//   float mod_t[1];
//   float mod_var_real[1];
//   float mod_var_imag[1];

//   float pulse_mod_real[256];
//   float pulse_mod_imag[256];
//   for (int idx = 0; idx < 256; idx++) {
//     pulse_mod_real[idx] = 1.0;
//     pulse_mod_imag[idx] = 0.0;
//   }

//   Add_Txchannel(location, polar, phi, phi_ptn, 2, theta, theta_ptn, 2,
//                 antenna_gain, mod_t, mod_var_real, mod_var_imag, 0,
//                 pulse_mod_real, pulse_mod_imag, 0, 1.0/180.0*kPI, tx_ptr);

//   float fs = 2e6;
//   float rf_gain = 20;
//   float resistor = 500;
//   float baseband_gain = 30;
//   t_Receiver *rx_ptr = Create_Receiver(fs, rf_gain, resistor, baseband_gain);

//   Add_Rxchannel(location, polar, phi, phi_ptn, 2, theta, theta_ptn, 2,
//                 antenna_gain, rx_ptr);

//   t_Radar *radar_ptr = Create_Radar(tx_ptr, rx_ptr);

//   float radar_loc_ptr[] = {0, 0, 0};
//   float radar_spd_ptr[] = {0, 0, 0};
//   float radar_rot_ptr[] = {0, 0, 0};
//   float radar_rrt_ptr[] = {0, 0, 0};
//   Set_Radar_Motion(radar_loc_ptr, radar_spd_ptr, radar_rot_ptr,
//   radar_rrt_ptr,
//                radar_ptr);

//   t_Targets *targets_ptr = Init_Targets();

//   float points[] =
//   {0,0,0,-0.0577,0,0.0816,-0.0577,0.0707,-0.0408,-0.0577,-0.0707,-0.0408};
//   int cells[]={0,1,2,0,2,3,0,3,1};
//   float tg_org[] = {0, 0, 0};
//   float tg_loc[] = {50, 0, 0};
//   float tg_speed[] = {-5, 0, 0};
//   float tg_rot[] = {0, 0, 0};
//   float tg_rrt[] = {0, 0, 0};
//   Add_Mesh_Target(points, cells, 3, tg_org, tg_loc, tg_speed, tg_rot, tg_rrt,
//   -1.0, 0.0, 1.0, 0.0, false, targets_ptr);

//   double bb_real[256][160];
//   double bb_imag[256][160];
//   for (int idx_a = 0; idx_a < 160; idx_a++) {
//     for (int idx_b = 0; idx_b < 256; idx_b++) {
//       bb_real[idx_b][idx_a] = 0.0;
//       bb_imag[idx_b][idx_a] = 0.0;
//     }
//   }

//   Run_RadarSimulator(radar_ptr, targets_ptr, &bb_real[0][0], &bb_imag[0][0]);

//   return 0;
// }
