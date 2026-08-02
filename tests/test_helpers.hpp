/*
 * @file test_helpers.hpp
 * @brief Shared fixtures and utilities for the RadarSim C wrapper test suite
 *
 * @details
 * Provides the building blocks every test file needs so that scenario setup
 * lives in exactly one place:
 * - Shared numeric constants and documented free-tier limits
 * - RAII handles so no test leaks a C API object, even when it fails
 * - RadarScenario: a configurable tx/rx/radar builder
 * - Canonical mesh geometries for target, RCS and LiDAR tests
 * - RS_SKIP_IF_LICENSED(): keeps free-tier tests meaningful in licensed builds
 *
 *    ----------
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

#ifndef RADARSIM_TESTS_TEST_HELPERS_HPP
#define RADARSIM_TESTS_TEST_HELPERS_HPP

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "radarsim.h"

namespace rstest {

/*********************************************
 *
 *  Constants
 *
 *********************************************/
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr float kPiF = static_cast<float>(kPi);

/** @brief Position tolerance (m) for geometry assertions */
inline constexpr float kTolerance = 1e-3f;

/**
 * @brief Free-tier limits documented in radarsim.h
 * @details Kept here so a limit change is a one-line test update.
 */
inline constexpr int kFreeTierMaxTxChannels = 1;
inline constexpr int kFreeTierMaxRxChannels = 1;
inline constexpr int kFreeTierMaxPointTargets = 2;
inline constexpr int kFreeTierMaxMeshTargets = 2;
inline constexpr int kFreeTierMaxMeshTriangles = 8;

/**
 * @brief Whether the running library enforces free-tier limits
 *
 * @details Builds without RADARSIMCPP_ENABLE_LICENSE report themselves as
 * licensed, so channel/target limits are not enforced. Tests that assert on
 * those limits must gate on this.
 */
inline bool IsFreeTier() { return Is_Licensed() == 0; }

/**
 * @brief Skip the current test when free-tier limits are not enforced
 */
#define RS_SKIP_IF_LICENSED()                                              \
  do {                                                                     \
    if (!::rstest::IsFreeTier()) {                                         \
      GTEST_SKIP() << "Licensed build - free-tier limits not enforced";    \
    }                                                                      \
  } while (0)

/*********************************************
 *
 *  RAII handles
 *
 *********************************************/
/** @brief unique_ptr deleter calling Free_Transmitter */
struct TransmitterDeleter {
  void operator()(t_Transmitter* ptr) const noexcept { Free_Transmitter(ptr); }
};
/** @brief unique_ptr deleter calling Free_Receiver */
struct ReceiverDeleter {
  void operator()(t_Receiver* ptr) const noexcept { Free_Receiver(ptr); }
};
/** @brief unique_ptr deleter calling Free_Radar */
struct RadarDeleter {
  void operator()(t_Radar* ptr) const noexcept { Free_Radar(ptr); }
};
/** @brief unique_ptr deleter calling Free_Targets */
struct TargetsDeleter {
  void operator()(t_Targets* ptr) const noexcept { Free_Targets(ptr); }
};

using TransmitterPtr = std::unique_ptr<t_Transmitter, TransmitterDeleter>;
using ReceiverPtr = std::unique_ptr<t_Receiver, ReceiverDeleter>;
using RadarPtr = std::unique_ptr<t_Radar, RadarDeleter>;
using TargetsPtr = std::unique_ptr<t_Targets, TargetsDeleter>;

/** @brief Create an owning target-manager handle */
inline TargetsPtr MakeTargets() { return TargetsPtr(Init_Targets()); }

/*********************************************
 *
 *  Antenna / polarization defaults
 *
 *********************************************/
/**
 * @brief Flat (isotropic) radiation pattern over the forward hemisphere
 */
struct IsotropicAntenna {
  static constexpr int kLength = 2;
  static constexpr float kGain = 20.0f;

  float phi[kLength] = {-kPiF / 2.0f, kPiF / 2.0f};
  float phi_pattern[kLength] = {0.0f, 0.0f};
  float theta[kLength] = {0.0f, kPiF};
  float theta_pattern[kLength] = {0.0f, 0.0f};
};

/** @brief Linear polarization along +X */
struct LinearPolarization {
  float real[3] = {1.0f, 0.0f, 0.0f};
  float imag[3] = {0.0f, 0.0f, 0.0f};
};

/*********************************************
 *
 *  Radar scenario builder
 *
 *********************************************/
/**
 * @brief Builds a minimal but complete transmitter/receiver/radar chain
 *
 * @details Public fields are configuration knobs; set them before calling a
 * Build* method. Every handle is owned by the scenario, so destroying it
 * releases the radar first and then the tx/rx pair.
 *
 * Typical use:
 * @code
 *   rstest::RadarScenario scenario;
 *   ASSERT_TRUE(scenario.Build());
 *   int bb_size = scenario.BasebandSize();
 * @endcode
 */
class RadarScenario {
 public:
  // ---- configuration (set before the first Build*/Attach* call) ----
  int num_samples = 64;   ///< Frequency LUT length
  int num_pulses = 1;     ///< Pulses per frame
  double fs = 1e6;        ///< Receiver sampling rate (Hz)
  float tx_power = 10.0f; ///< Transmitter power (dBm)
  double gate_delay = 0.0; ///< Range-gate / deramp reference delay (s)
  float tx_channel_location[3] = {0.0f, 0.0f, 0.0f};
  float rx_channel_location[3] = {0.0f, 0.0f, 0.0f};

  /**
   * @brief Create the transmitter and receiver, each with one channel
   * @return true on success
   */
  [[nodiscard]] bool BuildTxRx() {
    freq_.resize(num_samples);
    freq_time_.resize(num_samples);
    for (int i = 0; i < num_samples; i++) {
      freq_[i] = 24.0e9 + i * 3e6;
      freq_time_[i] = i * 1e-6;
    }

    freq_offset_.assign(num_pulses, 0.0);
    pulse_start_time_.resize(num_pulses);
    for (int i = 0; i < num_pulses; i++) {
      pulse_start_time_[i] = i * 100e-6;
    }

    pulse_mod_real_.assign(num_pulses, 1.0f);
    pulse_mod_imag_.assign(num_pulses, 0.0f);

    tx_.reset(Create_Transmitter(freq_.data(), freq_time_.data(), num_samples,
                                 freq_offset_.data(), pulse_start_time_.data(),
                                 num_pulses, tx_power));
    if (!tx_) return false;

    if (Add_Txchannel(tx_channel_location, polarization_.real,
                      polarization_.imag, antenna_.phi, antenna_.phi_pattern,
                      IsotropicAntenna::kLength, antenna_.theta,
                      antenna_.theta_pattern, IsotropicAntenna::kLength,
                      IsotropicAntenna::kGain, nullptr, nullptr, nullptr, 0,
                      pulse_mod_real_.data(), pulse_mod_imag_.data(), 0.0f,
                      0.1f, tx_.get()) != RADARSIM_SUCCESS) {
      return false;
    }

    rx_.reset(Create_Receiver(static_cast<float>(fs), 30.0f, 50.0f, 20.0f,
                              500e3f, gate_delay));
    if (!rx_) return false;

    return Add_Rxchannel(rx_channel_location, polarization_.real,
                         polarization_.imag, antenna_.phi,
                         antenna_.phi_pattern, IsotropicAntenna::kLength,
                         antenna_.theta, antenna_.theta_pattern,
                         IsotropicAntenna::kLength, IsotropicAntenna::kGain,
                         rx_.get()) == RADARSIM_SUCCESS;
  }

  /**
   * @brief Attach a constant-motion radar to an already-built tx/rx pair
   */
  [[nodiscard]] bool AttachRadar(float location[3], float speed[3],
                                 float rotation[3], float rotation_rate[3]) {
    frame_start_time_.assign(1, 0.0);
    radar_.reset(Create_Radar(tx_.get(), rx_.get(), frame_start_time_.data(),
                              1, location, speed, rotation, rotation_rate));
    return radar_ != nullptr;
  }

  /**
   * @brief Attach a time-varying radar to an already-built tx/rx pair
   */
  [[nodiscard]] bool AttachRadarArray(double* frame_start_time, int num_frames,
                                      float* location_array, int num_locations,
                                      float* speed, float* rotation_array,
                                      int num_rotations,
                                      float* rotation_rate) {
    radar_.reset(Create_Radar_Array(tx_.get(), rx_.get(), frame_start_time,
                                    num_frames, location_array, num_locations,
                                    speed, rotation_array, num_rotations,
                                    rotation_rate));
    return radar_ != nullptr;
  }

  /**
   * @brief Build tx/rx plus a stationary single-frame radar at the origin
   */
  [[nodiscard]] bool Build() {
    if (!BuildTxRx()) return false;
    float zero[3] = {0.0f, 0.0f, 0.0f};
    return AttachRadar(zero, zero, zero, zero);
  }

  // ---- derived sizes ----
  /**
   * @brief Samples per pulse, mirroring Radar::SetupRadar
   * @details sample_size = (freq_time.back() - freq_time.front()) * fs
   */
  [[nodiscard]] int SampleSize() const {
    if (freq_time_.size() < 2) return 0;
    return static_cast<int>((freq_time_.back() - freq_time_.front()) * fs);
  }

  /** @brief Virtual channel count (num_tx * num_rx), 0 if not built yet */
  [[nodiscard]] int VirtualChannelSize() const {
    if (!tx_ || !rx_) return 0;
    return Get_Num_Txchannel(tx_.get()) * Get_Num_Rxchannel(rx_.get());
  }

  /** @brief Baseband buffer size reported by the library */
  [[nodiscard]] int BasebandSize() const { return Get_BB_Size(radar_.get()); }

  /**
   * @brief Timestamp cube for the noise simulator
   * @details Layout [v_channel_size][pulse_size][sample_size], with
   * ts[c][p][s] = pulse_start_time[p] + s / fs
   */
  [[nodiscard]] std::vector<double> MakeTimestamps() const {
    const int sample_size = SampleSize();
    const int channels = VirtualChannelSize();
    std::vector<double> timestamps(
        static_cast<size_t>(channels) * num_pulses * sample_size);

    for (int c = 0; c < channels; c++) {
      for (int p = 0; p < num_pulses; p++) {
        for (int s = 0; s < sample_size; s++) {
          const size_t idx = (static_cast<size_t>(c) * num_pulses + p) *
                                 sample_size + s;
          timestamps[idx] = pulse_start_time_[p] + s / fs;
        }
      }
    }
    return timestamps;
  }

  // ---- accessors ----
  [[nodiscard]] t_Transmitter* tx() const { return tx_.get(); }
  [[nodiscard]] t_Receiver* rx() const { return rx_.get(); }
  [[nodiscard]] t_Radar* radar() const { return radar_.get(); }

 private:
  IsotropicAntenna antenna_;
  LinearPolarization polarization_;

  std::vector<double> freq_;
  std::vector<double> freq_time_;
  std::vector<double> freq_offset_;
  std::vector<double> pulse_start_time_;
  std::vector<double> frame_start_time_;
  std::vector<float> pulse_mod_real_;
  std::vector<float> pulse_mod_imag_;

  // Declaration order matters: radar_ is destroyed before tx_/rx_.
  TransmitterPtr tx_;
  ReceiverPtr rx_;
  RadarPtr radar_;
};

/*********************************************
 *
 *  Mesh geometries
 *
 *********************************************/
/** @brief Single unit triangle lying in the z = 0 plane */
struct TriangleMesh {
  std::vector<float> points = {0.0f, 0.0f, 0.0f,   // vertex 0
                               1.0f, 0.0f, 0.0f,   // vertex 1
                               0.5f, 1.0f, 0.0f};  // vertex 2
  std::vector<int> cells = {0, 1, 2};
  int cell_size = 1;
};

/**
 * @brief Square plate in the y-z plane whose surface normal faces -X
 *
 * @details Two triangles, so it fits inside the free-tier 8-triangle limit.
 * Place it with a +X location offset to face a sensor sitting at the origin.
 *
 * @param half_size Half the plate edge length (m)
 */
struct PlateMesh {
  explicit PlateMesh(float half_size = 1.0f) {
    const float h = half_size;
    points = {0.0f, -h, -h,   // vertex 0
              0.0f, h,  -h,   // vertex 1
              0.0f, h,  h,    // vertex 2
              0.0f, -h, h};   // vertex 3
    // Wound so the normal points along -X, back toward the sensor.
    cells = {0, 2, 1, 0, 3, 2};
    cell_size = 2;
    edge_length = 2.0f * h;
  }

  std::vector<float> points;
  std::vector<int> cells;
  int cell_size = 0;
  float edge_length = 0.0f;
};

/** @brief Zero vector, handy for the many {0,0,0} arguments in this API */
struct Motion {
  float location[3] = {0.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};
  float origin[3] = {0.0f, 0.0f, 0.0f};
};

/**
 * @brief Add a mesh target with default material and motion overrides
 *
 * @param mesh Geometry to add
 * @param motion Placement and velocity of the target
 * @param targets Target manager receiving the mesh
 * @return int RADARSIM_SUCCESS or a RADARSIM_ERROR_* code
 */
template <typename MeshT>
inline int AddMesh(MeshT& mesh, Motion& motion, t_Targets* targets) {
  return Add_Mesh_Target(mesh.points.data(), mesh.cells.data(), mesh.cell_size,
                         motion.origin, motion.location, motion.speed,
                         motion.rotation, motion.rotation_rate, -1.0f, 0.0f,
                         1.0f, 0.0f, false, 0.0f, false, targets);
}

}  // namespace rstest

#endif  // RADARSIM_TESTS_TEST_HELPERS_HPP
