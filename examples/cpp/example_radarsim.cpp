/*
 * @file example_radarsim.cpp
 * @brief Complete example demonstrating RadarSim C wrapper library usage from
 * C++
 *
 * This example shows how to:
 * 1. Create and configure transmitter with waveform and antenna pattern
 * 2. Create and configure receiver with RF parameters and antenna pattern
 * 3. Create a radar system combining transmitter and receiver
 * 4. Add point targets and mesh targets to the simulation
 * 5. Run radar simulation to get baseband signals
 * 6. Run RCS simulation for radar cross section analysis
 * 7. Run LiDAR simulation for point cloud generation
 * 8. Properly clean up resources
 */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "radarsim.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper function to create simple antenna pattern (isotropic with slight
// directivity)
void create_simple_antenna_pattern(float* angles, float* pattern,
                                   int num_points, float max_gain_db) {
  for (int i = 0; i < num_points; i++) {
    angles[i] = static_cast<float>(i) / (num_points - 1) * 2.0f * M_PI - M_PI;
    // Simple cosine-squared pattern
    pattern[i] =
        max_gain_db * std::cos(angles[i] / 2.0f) * std::cos(angles[i] / 2.0f);
  }
}

// RAII wrapper for automatic cleanup
class RadarSimManager {
 private:
  t_Transmitter* tx_;
  t_Receiver* rx_;
  t_Radar* radar_;
  t_Targets* targets_;

 public:
  RadarSimManager()
      : tx_(nullptr), rx_(nullptr), radar_(nullptr), targets_(nullptr) {}

  ~RadarSimManager() { cleanup(); }

  void cleanup() {
    if (targets_) {
      Free_Targets(targets_);
      targets_ = nullptr;
    }
    if (radar_) {
      Free_Radar(radar_);
      radar_ = nullptr;
    }
    if (rx_) {
      Free_Receiver(rx_);
      rx_ = nullptr;
    }
    if (tx_) {
      Free_Transmitter(tx_);
      tx_ = nullptr;
    }
  }

  // Getters and setters
  void set_transmitter(t_Transmitter* tx) { tx_ = tx; }
  void set_receiver(t_Receiver* rx) { rx_ = rx; }
  void set_radar(t_Radar* radar) { radar_ = radar; }
  void set_targets(t_Targets* targets) { targets_ = targets; }

  t_Transmitter* get_transmitter() const { return tx_; }
  t_Receiver* get_receiver() const { return rx_; }
  t_Radar* get_radar() const { return radar_; }
  t_Targets* get_targets() const { return targets_; }
};

int main() {
  std::cout << "RadarSim C++ Wrapper Library Example\n";
  std::cout << "====================================\n\n";

  try {
    RadarSimManager manager;

    // Get library version
    int version[3];
    Get_Version(version);
    std::cout << "RadarSim Library Version: " << version[0] << "." << version[1]
              << "." << version[2] << "\n\n";

    /*********************************************
     * 1. Create Transmitter Configuration
     *********************************************/
    std::cout << "1. Creating Transmitter...\n";

    // FMCW radar parameters
    const int num_pulses = 128;
    const float tx_power = 20.0f;  // 20 dBm

    // Use vectors for automatic memory management
    std::vector<double> freq = {76e9, 77e9};
    std::vector<double> freq_time = {0, 80e-6};
    std::vector<double> freq_offset(num_pulses, 0.0);
    std::vector<double> pulse_start_time(num_pulses);
    std::vector<float> pulse_mod_real(num_pulses,
                                      1.0f);  // No pulse modulation (all 1.0)
    std::vector<float> pulse_mod_imag(num_pulses,
                                      0.0f);  // No imaginary component

    // Set pulse timing
    for (int i = 0; i < num_pulses; i++) {
      pulse_start_time[i] = i * 100e-6;  // 100 microsecond PRI
    }

    // Create transmitter
    t_Transmitter* tx =
        Create_Transmitter(freq.data(), freq_time.data(), 2, freq_offset.data(),
                           pulse_start_time.data(), num_pulses, tx_power);
    if (!tx) {
      throw std::runtime_error("Failed to create transmitter");
    }
    manager.set_transmitter(tx);

    std::cout << "set_transmitter\n";

    // Configure transmitter antenna
    const int ant_points = 361;  // 1 degree resolution
    std::vector<float> phi_angles(ant_points);
    std::vector<float> phi_pattern(ant_points);
    std::vector<float> theta_angles(ant_points);
    std::vector<float> theta_pattern(ant_points);

    create_simple_antenna_pattern(phi_angles.data(), phi_pattern.data(),
                                  ant_points, 15.0f);
    create_simple_antenna_pattern(theta_angles.data(), theta_pattern.data(),
                                  ant_points, 15.0f);

    std::cout << "create_simple_antenna_pattern\n";

    // Transmitter location and polarization
    std::vector<float> tx_location = {0.0f, 0.0f, 1.5f};  // 1.5m height
    std::vector<float> tx_polar_real = {1.0f, 0.0f,
                                        0.0f};  // Horizontal polarization
    std::vector<float> tx_polar_imag = {0.0f, 0.0f, 0.0f};

    // Add transmitter channel
    int result = Add_Txchannel(
        tx_location.data(), tx_polar_real.data(), tx_polar_imag.data(),
        phi_angles.data(), phi_pattern.data(), ant_points, theta_angles.data(),
        theta_pattern.data(), ant_points,
        15.0f,                         // antenna gain
        nullptr, nullptr, nullptr, 0,  // No amplitude modulation
        pulse_mod_real.data(),
        pulse_mod_imag.data(),  // Pulse modulation arrays
        0.0f,                   // No delay
        0.1f,                   // Grid resolution
        tx);
    if (result != 0) {
      throw std::runtime_error("Failed to add transmitter channel");
    }

    std::cout << "   ✓ Transmitter created with " << Get_Num_Txchannel(tx)
              << " channels\n";

    /*********************************************
     * 2. Create Receiver Configuration
     *********************************************/
    std::cout << "2. Creating Receiver...\n";

    const float sampling_rate = 10e6;   // 10 MHz sampling rate
    const float rf_gain = 20.0f;        // 20 dB RF gain
    const float load_resistor = 50.0f;  // 50 ohm load
    const float baseband_gain = 30.0f;  // 30 dB baseband gain
    const float baseband_bw = 5e6;      // 5 MHz baseband bandwidth

    // Create receiver
    t_Receiver* rx = Create_Receiver(sampling_rate, rf_gain, load_resistor,
                                     baseband_gain, baseband_bw);
    if (!rx) {
      throw std::runtime_error("Failed to create receiver");
    }
    manager.set_receiver(rx);

    // Configure receiver antenna (same as transmitter for simplicity)
    std::vector<float> rx_location = {0.1f, 0.0f,
                                      1.5f};  // Slightly offset from TX

    result = Add_Rxchannel(rx_location.data(), tx_polar_real.data(),
                           tx_polar_imag.data(), phi_angles.data(),
                           phi_pattern.data(), ant_points, theta_angles.data(),
                           theta_pattern.data(), ant_points,
                           15.0f,  // antenna gain
                           rx);
    if (result != 0) {
      throw std::runtime_error("Failed to add receiver channel");
    }

    std::cout << "   ✓ Receiver created with " << Get_Num_Rxchannel(rx)
              << " channels\n";

    /*********************************************
     * 3. Create Radar System
     *********************************************/
    std::cout << "3. Creating Radar System...\n";

    const int num_frames = 1;
    std::vector<double> frame_start_time = {0.0};
    std::vector<float> radar_location = {0.0f, 0.0f, 1.5f};
    std::vector<float> radar_speed = {0.0f, 0.0f, 0.0f};  // Stationary radar
    std::vector<float> radar_rotation = {0.0f, 0.0f,
                                         0.0f};  // No initial rotation
    std::vector<float> radar_rotation_rate = {0.0f, 0.0f, 0.0f};  // No rotation

    t_Radar* radar = Create_Radar(
        tx, rx, frame_start_time.data(), num_frames, radar_location.data(),
        radar_speed.data(), radar_rotation.data(), radar_rotation_rate.data());
    if (!radar) {
      throw std::runtime_error("Failed to create radar system");
    }
    manager.set_radar(radar);

    std::cout << "   ✓ Radar system created successfully\n";

    /*********************************************
     * 4. Create Target Scenario
     *********************************************/
    std::cout << "4. Creating Target Scenario...\n";

    // Initialize target management system
    t_Targets* targets = Init_Targets();
    if (!targets) {
      throw std::runtime_error("Failed to initialize targets");
    }
    manager.set_targets(targets);

    // Add point targets
    std::vector<float> target1_location = {100.0f, 0.0f,
                                           1.0f};  // 100m range, 1m height
    std::vector<float> target1_speed = {-20.0f, 0.0f,
                                        0.0f};  // Approaching at 20 m/s
    float target1_rcs = 10.0f;                  // 10 dBsm RCS
    float target1_phase = 0.0f;

    result = Add_Point_Target(target1_location.data(), target1_speed.data(),
                              target1_rcs, target1_phase, targets);
    if (result != 0) {
      throw std::runtime_error("Failed to add point target 1");
    }

    std::vector<float> target2_location = {
        200.0f, 50.0f, 1.5f};  // 200m range, 50m cross-range
    std::vector<float> target2_speed = {0.0f, -10.0f,
                                        0.0f};  // Moving laterally
    float target2_rcs = 5.0f;                   // 5 dBsm RCS
    float target2_phase = M_PI / 4;

    result = Add_Point_Target(target2_location.data(), target2_speed.data(),
                              target2_rcs, target2_phase, targets);
    if (result != 0) {
      throw std::runtime_error("Failed to add point target 2");
    }

    // Add a simple mesh target (triangular plate)
    std::vector<float> mesh_points = {
        -1.0f, -1.0f, 0.0f,  // Vertex 0
        1.0f,  -1.0f, 0.0f,  // Vertex 1
        0.0f,  1.0f,  0.0f,   // Vertex 2
        0.0f,  1.0f,  2.0f   // Vertex 3
    };
    std::vector<int> mesh_cells = {0, 1, 2, 0, 2, 3};  // Single triangle
    std::vector<float> mesh_origin = {0.0f, 0.0f, 0.0f};
    std::vector<float> mesh_location = {150.0f, -30.0f, 2.0f};
    std::vector<float> mesh_speed = {5.0f, 0.0f, 0.0f};
    std::vector<float> mesh_rotation = {0.0f, 0.0f, 0.0f};
    std::vector<float> mesh_rotation_rate = {0.0f, 0.0f,
                                             0.1f};  // Slow rotation

    result = Add_Mesh_Target(mesh_points.data(), mesh_cells.data(), 2,
                             mesh_origin.data(), mesh_location.data(),
                             mesh_speed.data(), mesh_rotation.data(),
                             mesh_rotation_rate.data(), 1.0f,
                             0.0f,        // Relative permittivity (no loss)
                             1.0f, 0.0f,  // Relative permeability (no loss)
                             false,       // Not ground
                             targets);
    if (result != 0) {
      throw std::runtime_error("Failed to add mesh target");
    }

    std::cout
        << "   ✓ Target scenario created (2 point targets + 1 mesh target)\n";

    /*********************************************
     * 5. Run Radar Simulation
     *********************************************/
    std::cout << "5. Running Radar Simulation...\n";

    // Calculate buffer size for baseband signal
    // int samples_per_pulse = static_cast<int>(pulse_duration * sampling_rate);
    int buffer_size = Get_BB_Size(radar);

    std::cout << "   Buffer size: " << buffer_size;

    // Allocate baseband signal buffers
    std::vector<double> bb_real(buffer_size);
    std::vector<double> bb_imag(buffer_size);

    // Run radar simulation
    std::vector<int> ray_filter = {0, 10};  // No range filtering (0-10m)
    Run_RadarSimulator(radar, targets, 0, 1.0f, ray_filter.data(),
                       bb_real.data(), bb_imag.data());

    // Analyze simulation results
    double max_amplitude = 0.0;
    double total_power = 0.0;
    for (size_t i = 0; i < bb_real.size(); i++) {
      double amplitude =
          std::sqrt(bb_real[i] * bb_real[i] + bb_imag[i] * bb_imag[i]);
      max_amplitude = std::max(max_amplitude, amplitude);
      total_power += amplitude * amplitude;
    }
    double rms_amplitude = std::sqrt(total_power / bb_real.size());

    std::cout << "   ✓ Radar simulation completed\n";
    std::cout << "   Max amplitude: " << std::scientific << max_amplitude
              << ", RMS amplitude: " << rms_amplitude << std::endl;

    /*********************************************
     * 6. Run RCS Simulation (optional)
     *********************************************/
    std::cout << "6. Running RCS Simulation...\n";

    const int num_rcs_directions = 36;  // 10 degree steps
    std::vector<double> inc_directions(num_rcs_directions * 3);
    std::vector<double> obs_directions(num_rcs_directions * 3);
    std::vector<double> rcs_results(num_rcs_directions);

    // Create bistatic RCS measurement directions
    for (int i = 0; i < num_rcs_directions; i++) {
      double angle = i * 2.0 * M_PI / num_rcs_directions;
      // Incident direction (always from front)
      inc_directions[i * 3 + 0] = -1.0;  // -X direction
      inc_directions[i * 3 + 1] = 0.0;
      inc_directions[i * 3 + 2] = 0.0;
      // Observation direction (varying azimuth)
      obs_directions[i * 3 + 0] = std::cos(angle);
      obs_directions[i * 3 + 1] = std::sin(angle);
      obs_directions[i * 3 + 2] = 0.0;
    }

    std::vector<double> inc_polar_real = {0.0, 0.0,
                                          1.0};  // Vertical polarization
    std::vector<double> inc_polar_imag = {0.0, 0.0, 0.0};
    std::vector<double> obs_polar_real = {0.0, 0.0, 1.0};  // Same polarization
    std::vector<double> obs_polar_imag = {0.0, 0.0, 0.0};

    result = Run_RcsSimulator(targets, inc_directions.data(),
                              obs_directions.data(), num_rcs_directions,
                              inc_polar_real.data(), inc_polar_imag.data(),
                              obs_polar_real.data(), obs_polar_imag.data(),
                              76.5e9, 2.0, rcs_results.data());

    if (result == 0) {
      std::cout << "   ✓ RCS simulation completed\n";
      auto max_rcs_iter =
          std::max_element(rcs_results.begin(), rcs_results.end());
      std::cout << "   Maximum RCS: " << std::scientific << *max_rcs_iter
                << " m²\n";
    } else {
      std::cout << "   RCS simulation failed or not available\n";
    }

    /*********************************************
     * 7. Run LiDAR Simulation (optional)
     *********************************************/
    std::cout << "7. Running LiDAR Simulation...\n";

    const int num_lidar_rays = 1000;
    const int max_lidar_points = 500;

    std::vector<double> phi_array(num_lidar_rays);
    std::vector<double> theta_array(num_lidar_rays);
    std::vector<double> cloud_points(max_lidar_points * 3);
    std::vector<double> cloud_distances(max_lidar_points);
    std::vector<double> cloud_intensities(max_lidar_points);

    // Create random ray directions
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> phi_dist(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> theta_dist(0.0, M_PI);

    for (int i = 0; i < num_lidar_rays; i++) {
      phi_array[i] = phi_dist(gen);
      theta_array[i] = theta_dist(gen);
    }

    std::vector<double> lidar_location = {0.0, 0.0, 1.5};  // Same as radar
    int actual_points = 0;

    result = Run_LidarSimulator(
        targets, phi_array.data(), theta_array.data(), num_lidar_rays,
        lidar_location.data(), cloud_points.data(), cloud_distances.data(),
        cloud_intensities.data(), max_lidar_points, &actual_points);

    if (result == 0) {
      std::cout << "   ✓ LiDAR simulation completed\n";
      std::cout << "   Point cloud contains " << actual_points << " points\n";
      if (actual_points > 0) {
        std::cout << "   First point: (" << std::fixed << std::setprecision(2)
                  << cloud_points[0] << ", " << cloud_points[1] << ", "
                  << cloud_points[2] << ") at distance " << cloud_distances[0]
                  << " m\n";
      }
    } else {
      std::cout << "   LiDAR simulation failed or not available\n";
    }

    std::cout << "\n✓ All simulations completed successfully!\n";

  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "\n8. Cleaning up...\n";
  std::cout << "   ✓ Cleanup completed (automatic with RAII)\n";
  std::cout << "\nExample completed successfully!\n";

  return 0;
}
