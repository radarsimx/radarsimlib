/*
 * @file radarsim.cpp
 * @brief C wrapper implementation for RadarSimCpp library
 *
 * @details
 * This file provides a C-compatible interface for the RadarSimCpp library,
 * enabling radar simulation functionality for C applications. Features include:
 * - Transmitter and receiver configuration
 * - Radar system modeling
 * - Point and mesh target management
 * - Simulation execution
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

#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
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
 *  Version
 *
 *********************************************/
/**
 * @brief Get the version of RadarSimLib
 *
 * @param version Version numbers {major, minor}
 */
void Get_Version(int version[2]) {
  version[0] = VERSION_MAJOR;
  version[1] = VERSION_MINOR;
}

/*********************************************
 *
 *  Transmitter
 *
 *********************************************/
/**
 * @brief Internal structure for Transmitter C wrapper
 * @details Contains shared_ptr to Transmitter object for memory management
 */
struct s_Transmitter {
  std::shared_ptr<Transmitter<double, float>> _ptr_transmitter;
};

/**
 * @brief Create a Transmitter object
 *
 * @details Creates a new transmitter with specified waveform and timing
 * parameters. Performs comprehensive input validation and uses modern C++
 * memory management.
 *
 * @param freq Frequency vector (Hz) - must not be NULL
 * @param freq_time Timestamp vector for the frequency vector (s) - must not be
 * NULL
 * @param waveform_size Length of the frequency and timestamp vector - must be >
 * 0
 * @param freq_offset Frequency offset per pulse (Hz) - must not be NULL
 * @param pulse_start_time Pulse start time vector (s) - must not be NULL
 * @param num_pulses Number of pulses - must be > 0
 * @param tx_power Transmitter power (dBm)
 * @return t_Transmitter* Pointer to the Transmitter object, NULL on failure
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
  t_Transmitter *ptr_tx_c = (t_Transmitter *)malloc(sizeof(t_Transmitter));
  if (ptr_tx_c == nullptr) {
    return nullptr;
  }

  try {
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

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Create_Transmitter: Memory allocation failed: " << e.what()
              << std::endl;
    free(ptr_tx_c);
    return nullptr;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to constructor
    std::cerr << "Create_Transmitter: Invalid argument: " << e.what()
              << std::endl;
    free(ptr_tx_c);
    return nullptr;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Create_Transmitter: Unexpected error: " << e.what()
              << std::endl;
    free(ptr_tx_c);
    return nullptr;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Create_Transmitter: Unknown error occurred" << std::endl;
    free(ptr_tx_c);
    return nullptr;
  }

  return ptr_tx_c;
}

/**
 * @brief Add a transmitter channel with full configuration
 *
 * @details Configures a transmitter channel with antenna pattern, polarization,
 * and modulation parameters. Supports both amplitude and phase modulation.
 *
 * @param location Channel location {x, y, z} (m) - must not be NULL
 * @param polar_real Real part of polarization vector {x, y, z} - must not be
 * NULL
 * @param polar_imag Imaginary part of polarization vector {x, y, z} - must not
 * be NULL
 * @param phi Phi angles for radiation pattern (rad) - must be equal-spaced -
 * must not be NULL
 * @param phi_ptn Normalized phi pattern values (dB) - must not be NULL
 * @param phi_length Length of phi and phi_ptn arrays - must be > 0
 * @param theta Theta angles for radiation pattern (rad) - must be equal-spaced
 * - must not be NULL
 * @param theta_ptn Normalized theta pattern values (dB) - must not be NULL
 * @param theta_length Length of theta and theta_ptn arrays - must be > 0
 * @param antenna_gain Antenna gain (dB)
 * @param mod_t Modulation timestamps (s) - must be equal-spaced - must not be
 * NULL
 * @param mod_var_real Real part of modulation values - must not be NULL
 * @param mod_var_imag Imaginary part of modulation values - must not be NULL
 * @param mod_length Length of modulation arrays - must be > 0
 * @param pulse_mod_real Real part of pulse modulation - must not be NULL
 * @param pulse_mod_imag Imaginary part of pulse modulation - must not be NULL
 * @param delay Transmitting delay (s)
 * @param grid Ray occupancy checking grid resolution (rad)
 * @param ptr_tx_c Pointer to the Transmitter - must not be NULL
 * @return int Status code (0 for success, 1 for failure/free tier limit)
 */
int Add_Txchannel(float *location, float *polar_real, float *polar_imag,
                  float *phi, float *phi_ptn, int phi_length, float *theta,
                  float *theta_ptn, int theta_length, float antenna_gain,
                  float *mod_t, float *mod_var_real, float *mod_var_imag,
                  int mod_length, float *pulse_mod_real, float *pulse_mod_imag,
                  float delay, float grid, t_Transmitter *ptr_tx_c) {
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
 * @brief Get the number of Tx channels
 *
 * @param ptr_tx_c Pointer to the Transmitter
 * @return int Number of Tx channels
 */
int Get_Num_Txchannel(t_Transmitter *ptr_tx_c) {
  return ptr_tx_c->_ptr_transmitter->channel_size_;
}

/**
 * @brief Free transmitter memory safely
 *
 * @details Safely releases transmitter resources using modern C++
 * memory management. The shared_ptr automatically handles cleanup.
 *
 * @param ptr_tx_c Pointer to the Transmitter
 */
void Free_Transmitter(t_Transmitter *ptr_tx_c) {
  if (ptr_tx_c == NULL) {
    return;
  }
  // shared_ptr automatically handles cleanup, just reset it
  ptr_tx_c->_ptr_transmitter.reset();
  free(ptr_tx_c);
}

/*********************************************
 *
 *  Receiver
 *
 *********************************************/
/**
 * @brief Internal structure for Receiver C wrapper
 * @details Contains shared_ptr to Receiver object for memory management
 */
struct s_Receiver {
  std::shared_ptr<Receiver<float>> _ptr_receiver;
};

/**
 * @brief Create a Receiver object
 *
 * @details Creates a new receiver with specified RF and baseband parameters.
 * Includes input validation and modern C++ memory management.
 *
 * @param fs Sampling rate (Hz) - must be > 0
 * @param rf_gain RF gain (dB)
 * @param resistor Load resistor (Ohm) - must be > 0
 * @param baseband_gain Baseband gain (dB)
 * @param baseband_bw Baseband bandwidth (Hz)
 * @return t_Receiver* Pointer to the Receiver object, NULL on failure
 */
t_Receiver *Create_Receiver(float fs, float rf_gain, float resistor,
                            float baseband_gain, float baseband_bw) {
  // Input validation
  if (fs <= 0 || resistor <= 0) {
    return nullptr;
  }

  // Allocate memory for the wrapper struct
  t_Receiver *ptr_rx_c = (t_Receiver *)malloc(sizeof(t_Receiver));
  if (ptr_rx_c == nullptr) {
    return nullptr;
  }

  try {
    // Create the Receiver object using shared_ptr
    ptr_rx_c->_ptr_receiver = std::make_shared<Receiver<float>>(
        fs, rf_gain, resistor, baseband_gain, baseband_bw);

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Create_Receiver: Memory allocation failed: " << e.what()
              << std::endl;
    free(ptr_rx_c);
    return nullptr;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to constructor
    std::cerr << "Create_Receiver: Invalid argument: " << e.what() << std::endl;
    free(ptr_rx_c);
    return nullptr;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Create_Receiver: Unexpected error: " << e.what() << std::endl;
    free(ptr_rx_c);
    return nullptr;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Create_Receiver: Unknown error occurred" << std::endl;
    free(ptr_rx_c);
    return nullptr;
  }

  return ptr_rx_c;
}

/**
 * @brief Add a receiver channel with antenna configuration
 *
 * @details Configures a receiver channel with antenna pattern and polarization.
 * The receiver channel defines how signals are received and processed.
 *
 * @param location Channel location {x, y, z} (m) - must not be NULL
 * @param polar_real Real part of polarization vector {x, y, z} - must not be
 * NULL
 * @param polar_imag Imaginary part of polarization vector {x, y, z} - must not
 * be NULL
 * @param phi Phi angles for radiation pattern (rad) - must be equal-spaced -
 * must not be NULL
 * @param phi_ptn Normalized phi pattern values (dB) - must not be NULL
 * @param phi_length Length of phi and phi_ptn arrays - must be > 0
 * @param theta Theta angles for radiation pattern (rad) - must be equal-spaced
 * - must not be NULL
 * @param theta_ptn Normalized theta pattern values (dB) - must not be NULL
 * @param theta_length Length of theta and theta_ptn arrays - must be > 0
 * @param antenna_gain Antenna gain (dB)
 * @param ptr_rx_c Pointer to the Receiver - must not be NULL
 * @return int Status code (0 for success, 1 for failure/free tier limit)
 */
int Add_Rxchannel(float *location, float *polar_real, float *polar_imag,
                  float *phi, float *phi_ptn, int phi_length, float *theta,
                  float *theta_ptn, int theta_length, float antenna_gain,
                  t_Receiver *ptr_rx_c) {
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
 * @brief Get the number of Rx channels
 *
 * @param ptr_rx_c Pointer to the Receiver
 * @return int Number of Rx channels
 */
int Get_Num_Rxchannel(t_Receiver *ptr_rx_c) {
  return ptr_rx_c->_ptr_receiver->channel_size_;
}

/**
 * @brief Free receiver memory safely
 *
 * @details Safely releases receiver resources using modern C++
 * memory management. The shared_ptr automatically handles cleanup.
 *
 * @param ptr_rx_c Pointer to the Receiver
 */
void Free_Receiver(t_Receiver *ptr_rx_c) {
  if (ptr_rx_c == NULL) {
    return;
  }
  // shared_ptr automatically handles cleanup, just reset it
  ptr_rx_c->_ptr_receiver.reset();
  free(ptr_rx_c);
}

/*********************************************
 *
 *  Radar
 *
 *********************************************/
/**
 * @brief Internal structure for Radar C wrapper
 * @details Contains pointers to transmitter, receiver, and radar objects for
 * complete system
 */
struct s_Radar {
  t_Transmitter *_ptr_tx;
  t_Receiver *_ptr_rx;
  std::shared_ptr<Radar<double, float>> _ptr_radar;
};

/**
 * @brief Create a Radar system object
 *
 * @details Creates a complete radar system by combining transmitter and
 * receiver with platform motion parameters. Performs comprehensive input
 * validation.
 *
 * @param ptr_tx_c Pointer to the Transmitter - must not be NULL
 * @param ptr_rx_c Pointer to the Receiver - must not be NULL
 * @param frame_start_time Frame start time vector (s) - must not be NULL
 * @param num_frames Number of frames - must be > 0
 * @param location Radar's location {x, y, z} (m) - must not be NULL
 * @param speed Radar's speed {x, y, z} (m/s) - must not be NULL
 * @param rotation Radar's rotation {x, y, z} (rad) - must not be NULL
 * @param rotation_rate Radar's rotation rate {x, y, z} (rad/s) - must not be
 * NULL
 * @return t_Radar* Pointer to the Radar object, NULL on failure
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
  t_Radar *ptr_radar_c = (t_Radar *)malloc(sizeof(t_Radar));
  if (ptr_radar_c == nullptr) {
    return nullptr;
  }

  ptr_radar_c->_ptr_tx = ptr_tx_c;
  ptr_radar_c->_ptr_rx = ptr_rx_c;

  try {
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

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Create_Radar: Memory allocation failed: " << e.what()
              << std::endl;
    free(ptr_radar_c);
    return nullptr;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to constructor
    std::cerr << "Create_Radar: Invalid argument: " << e.what() << std::endl;
    free(ptr_radar_c);
    return nullptr;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Create_Radar: Unexpected error: " << e.what() << std::endl;
    free(ptr_radar_c);
    return nullptr;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Create_Radar: Unknown error occurred" << std::endl;
    free(ptr_radar_c);
    return nullptr;
  }

  return ptr_radar_c;
}

/**
 * @brief Free radar system memory safely
 *
 * @details Safely releases radar system resources using modern C++
 * memory management. The shared_ptr automatically handles cleanup.
 *
 * @param ptr_radar_c Pointer to the Radar system
 */
void Free_Radar(t_Radar *ptr_radar_c) {
  if (ptr_radar_c == NULL) {
    return;
  }
  // shared_ptr automatically handles cleanup, just reset it
  ptr_radar_c->_ptr_radar.reset();
  free(ptr_radar_c);
}

/*********************************************
 *
 *  Targets
 *
 *********************************************/
/**
 * @brief Internal structure for Targets C wrapper
 * @details Contains shared_ptr to PointsManager and TargetsManager for
 * comprehensive target management including both point targets and mesh targets
 */
struct s_Targets {
  std::shared_ptr<PointsManager<float>> _ptr_points;
  std::shared_ptr<TargetsManager<float>> _ptr_targets;
};

/**
 * @brief Initialize the target management system
 *
 * @details Creates and initializes both point and mesh target managers.
 * Must be called before adding any targets to the simulation.
 *
 * @return t_Targets* Pointer to the target management system, NULL on failure
 */
t_Targets *Init_Targets() {
  // Allocate memory for the wrapper struct
  t_Targets *ptr_targets_c = (t_Targets *)malloc(sizeof(t_Targets));
  if (ptr_targets_c == nullptr) {
    return nullptr;
  }

  try {
    // Create the manager objects using shared_ptr
    ptr_targets_c->_ptr_points = std::make_shared<PointsManager<float>>();
    ptr_targets_c->_ptr_targets = std::make_shared<TargetsManager<float>>();

  } catch (const std::bad_alloc &e) {
    // Memory allocation failed
    std::cerr << "Init_Targets: Memory allocation failed: " << e.what()
              << std::endl;
    free(ptr_targets_c);
    return nullptr;
  } catch (const std::invalid_argument &e) {
    // Invalid parameters passed to constructor
    std::cerr << "Init_Targets: Invalid argument: " << e.what() << std::endl;
    free(ptr_targets_c);
    return nullptr;
  } catch (const std::exception &e) {
    // Any other standard exception
    std::cerr << "Init_Targets: Unexpected error: " << e.what() << std::endl;
    free(ptr_targets_c);
    return nullptr;
  } catch (...) {
    // Any non-standard exception
    std::cerr << "Init_Targets: Unknown error occurred" << std::endl;
    free(ptr_targets_c);
    return nullptr;
  }

  return ptr_targets_c;
}

/**
 * @brief Add an ideal point target to the simulation
 *
 * @details Adds a point target with specified radar cross section and motion.
 * Point targets are ideal scatterers used for basic simulations.
 *
 * @param location Target's location {x, y, z} (m) - must not be NULL
 * @param speed Target's speed {x, y, z} (m/s) - must not be NULL
 * @param rcs Target's RCS (dBsm)
 * @param phs Target's phase (rad)
 * @param ptr_targets_c Pointer to the target management system - must not be
 * NULL
 * @return int Status code (0 for success, 1 for failure/free tier limit)
 */
int Add_Point_Target(float *location, float *speed, float rcs, float phs,
                     t_Targets *ptr_targets_c) {
  if (IsFreeTier() && ptr_targets_c->_ptr_points->vect_points_.size() > 1) {
    return 1;
  }
  ptr_targets_c->_ptr_points->AddPointSimple(
      rsv::Vec3<float>(location[0], location[1], location[2]),
      rsv::Vec3<float>(speed[0], speed[1], speed[2]), rcs, phs);
  return 0;
}

/**
 * @brief Add a 3D mesh target to the simulation
 *
 * @details Adds a complex 3D mesh target with electromagnetic properties.
 * Mesh targets provide realistic scattering behavior for detailed simulations.
 *
 * @param points Mesh vertex coordinates - must not be NULL
 * @param cells Mesh triangle connectivity - must not be NULL
 * @param cell_size Number of mesh triangles - must be > 0
 * @param origin Target origin point (m) - must not be NULL
 * @param location Target location {x, y, z} (m) - must not be NULL
 * @param speed Target speed {x, y, z} (m/s) - must not be NULL
 * @param rotation Target rotation {x, y, z} (rad) - must not be NULL
 * @param rotation_rate Target rotation rate {x, y, z} (rad/s) - must not be
 * NULL
 * @param ep_real Real part of relative permittivity
 * @param ep_imag Imaginary part of relative permittivity
 * @param mu_real Real part of relative permeability
 * @param mu_imag Imaginary part of relative permeability
 * @param is_ground Flag to identify if the target represents ground surface
 * @param ptr_targets_c Pointer to the target management system - must not be
 * NULL
 * @return int Status code (0 for success, 1 for failure/free tier limit)
 */
int Add_Mesh_Target(float *points, int *cells, int cell_size, float *origin,
                    float *location, float *speed, float *rotation,
                    float *rotation_rate, float ep_real, float ep_imag,
                    float mu_real, float mu_imag, bool is_ground,
                    t_Targets *ptr_targets_c) {
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
 * @brief Free target management system memory
 *
 * @details Safely releases all target-related resources using modern C++
 * memory management. Automatically handles cleanup of shared_ptr objects.
 *
 * @param ptr_targets_c Pointer to the target management system
 */
void Free_Targets(t_Targets *ptr_targets_c) {
  if (ptr_targets_c == NULL) {
    return;
  }
  // shared_ptr automatically handles cleanup, just reset them
  ptr_targets_c->_ptr_targets.reset();
  ptr_targets_c->_ptr_points.reset();
  free(ptr_targets_c);
}

/**
 * @brief Complete the initialization of all targets
 *
 * @details Finalizes the setup of both point and mesh targets. This function
 * must be called after adding all targets and before running simulation.
 * It prepares the targets for efficient GPU processing if available.
 *
 * @param ptr_targets_c Pointer to the target management system - must not be
 * NULL
 */
void Complete_Targets_Initialization(t_Targets *ptr_targets_c) {
  if (ptr_targets_c == NULL) {
    return;
  }
  ptr_targets_c->_ptr_points->CompletePointInitialization();
  ptr_targets_c->_ptr_targets->CompleteTargetInitialization();
}

/*********************************************
 *
 *  Simulator
 *
 *********************************************/
/**
 * @brief Execute radar simulation for all configured targets
 *
 * @details Runs comprehensive radar simulation including both point and mesh
 * targets. Initializes baseband buffers, processes targets based on type, and
 * synchronizes results. Supports GPU acceleration when available.
 *
 * @param ptr_radar_c Pointer to the radar system - must not be NULL
 * @param ptr_targets_c Pointer to the target management system - must not be
 * NULL
 * @param level Ray tracing fidelity level (higher = more accurate, slower)
 * @param density Ray density for mesh target simulation
 * @param ray_filter Ray filter parameters {min, max} - must not be NULL
 * @param ptr_bb_real Real part of baseband signal buffer - must not be NULL
 * @param ptr_bb_imag Imaginary part of baseband signal buffer - must not be
 * NULL
 */
void Run_Simulator(t_Radar *ptr_radar_c, t_Targets *ptr_targets_c, int level,
                   float density, int *ray_filter, double *ptr_bb_real,
                   double *ptr_bb_imag) {
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
 * @brief Execute interference simulation between radar systems
 *
 * @details Simulates electromagnetic interference between two radar systems.
 * The victim radar receives interference from the interfering radar, allowing
 * analysis of interference effects on radar performance.
 *
 * @param ptr_radar_c Pointer to the victim radar system - must not be NULL
 * @param ptr_interf_radar_c Pointer to the interfering radar system - must not
 * be NULL
 * @param ptr_interf_real Real part of the interference baseband buffer - must
 * not be NULL
 * @param ptr_interf_imag Imaginary part of the interference baseband buffer -
 * must not be NULL
 */
void Run_Interference(t_Radar *ptr_radar_c, t_Radar *ptr_interf_radar_c,
                      double *ptr_interf_real, double *ptr_interf_imag) {
  InterferenceSimulator<double, float> simc =
      InterferenceSimulator<double, float>();
  ptr_radar_c->_ptr_radar->InitBaseband(ptr_interf_real, ptr_interf_imag);
  simc.Run(ptr_radar_c->_ptr_radar, ptr_interf_radar_c->_ptr_radar);
  ptr_radar_c->_ptr_radar->SyncBaseband();
}

/**
 * @brief Execute Radar Cross Section (RCS) simulation
 *
 * @details Calculates the Radar Cross Section of targets using Physical Optics
 * ray tracing method. Supports multiple incident and observation directions
 * for comprehensive RCS analysis. Uses BVH acceleration for efficient
 * computation.
 *
 * @param ptr_targets_c Pointer to the target management system - must not be
 * NULL
 * @param inc_dir_array Array of incident direction vectors {x, y, z} - must not
 * be NULL
 * @param obs_dir_array Array of observation direction vectors {x, y, z} - must
 * not be NULL
 * @param num_directions Number of direction pairs - must be > 0
 * @param inc_polar_real Real part of incident polarization vector {x, y, z} -
 * must not be NULL
 * @param inc_polar_imag Imaginary part of incident polarization vector {x, y,
 * z} - must not be NULL
 * @param obs_polar_real Real part of observation polarization vector {x, y, z}
 * - must not be NULL
 * @param obs_polar_imag Imaginary part of observation polarization vector {x,
 * y, z} - must not be NULL
 * @param frequency Electromagnetic frequency (Hz) - must be > 0
 * @param density Ray density for Physical Optics (rays per wavelength) - must
 * be > 0
 * @param rcs_result Output array for RCS values (m²) - must not be NULL, size
 * >= num_directions
 * @return int Status code (0 for success, 1 for failure)
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
 * @brief Execute LiDAR point cloud simulation
 *
 * @details Performs LiDAR point cloud generation using ray tracing simulation.
 * Shoots rays from the sensor position in specified directions and records
 * hit points on target surfaces. Uses BVH acceleration for efficient
 * computation.
 *
 * @param ptr_targets_c Pointer to the target management system - must not be
 * NULL
 * @param phi_array Array of azimuth angles (rad) - must not be NULL
 * @param theta_array Array of elevation angles (rad) - must not be NULL
 * @param num_rays Number of rays to shoot - must be > 0
 * @param sensor_location LiDAR sensor position {x, y, z} (m) - must not be NULL
 * @param cloud_points Output array for point cloud coordinates {x, y, z} - must
 * not be NULL
 * @param cloud_distances Output array for point distances (m) - must not be
 * NULL
 * @param cloud_intensities Output array for point intensities - must not be
 * NULL
 * @param max_points Maximum number of points to return - must be > 0
 * @param actual_points Output: actual number of points found - must not be NULL
 * @return int Status code (0 for success, 1 for failure)
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

//   Run_Simulator(radar_ptr, targets_ptr, &bb_real[0][0], &bb_imag[0][0]);

//   return 0;
// }
