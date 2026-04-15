/*
 * @file test_simulator.cpp
 * @brief Unit tests for Simulator C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Noise simulator creation and execution
 * - Interference simulator return type
 * - Parameter validation
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

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "radarsim.h"

#define kPI 3.141592653589793

/**
 * @brief Helper to create a minimal configured radar system for simulator tests
 */
struct RadarSetup {
  t_Transmitter* tx = nullptr;
  t_Receiver* rx = nullptr;
  t_Radar* radar = nullptr;

  std::vector<double> freq;
  std::vector<double> freq_time;
  std::vector<double> freq_offset;
  std::vector<double> pulse_start_time;
  std::vector<float> pulse_mod_real;
  std::vector<float> pulse_mod_imag;

  // Radar parameters exposed for timestamp generation
  int num_samples_ = 0;
  int num_pulses_ = 0;
  double fs_ = 0.0;

  bool Create(int num_samples = 64, int num_pulses = 1) {
    num_samples_ = num_samples;
    num_pulses_ = num_pulses;
    fs_ = 1e6;  // Must match receiver fs

    // Setup frequency sweep
    freq.resize(num_samples);
    freq_time.resize(num_samples);
    for (int i = 0; i < num_samples; i++) {
      freq[i] = 24.0e9 + i * 3e6;
      freq_time[i] = i * 1e-6;
    }

    freq_offset.resize(num_pulses, 0.0);
    pulse_start_time.resize(num_pulses);
    for (int i = 0; i < num_pulses; i++) {
      pulse_start_time[i] = i * 100e-6;
    }

    // Create transmitter
    tx = Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                            freq_offset.data(), pulse_start_time.data(),
                            num_pulses, 10.0f);
    if (!tx) return false;

    // Add tx channel
    float location[3] = {0.0f, 0.0f, 0.0f};
    float polar_real[3] = {1.0f, 0.0f, 0.0f};
    float polar_imag[3] = {0.0f, 0.0f, 0.0f};
    float phi[2] = {-static_cast<float>(kPI) / 2.0f,
                    static_cast<float>(kPI) / 2.0f};
    float phi_ptn[2] = {0.0f, 0.0f};
    float theta[2] = {0.0f, static_cast<float>(kPI)};
    float theta_ptn[2] = {0.0f, 0.0f};

    pulse_mod_real.resize(num_pulses, 1.0f);
    pulse_mod_imag.resize(num_pulses, 0.0f);

    int result =
        Add_Txchannel(location, polar_real, polar_imag, phi, phi_ptn, 2, theta,
                      theta_ptn, 2, 20.0f, nullptr, nullptr, nullptr, 0,
                      pulse_mod_real.data(), pulse_mod_imag.data(), 0.0f, 0.1f, tx);
    if (result != 0) return false;

    // Create receiver (fs = 1 MHz)
    rx = Create_Receiver(static_cast<float>(fs_), 30.0f, 50.0f, 20.0f, 500e3f);
    if (!rx) return false;

    // Add rx channel
    result = Add_Rxchannel(location, polar_real, polar_imag, phi, phi_ptn, 2,
                           theta, theta_ptn, 2, 20.0f, rx);
    if (result != 0) return false;

    // Create radar
    double frame_start_time[1] = {0.0};
    float loc[3] = {0.0f, 0.0f, 0.0f};
    float speed[3] = {0.0f, 0.0f, 0.0f};
    float rotation[3] = {0.0f, 0.0f, 0.0f};
    float rotation_rate[3] = {0.0f, 0.0f, 0.0f};

    radar = Create_Radar(tx, rx, frame_start_time, 1, loc, speed, rotation,
                         rotation_rate);
    return radar != nullptr;
  }

  /**
   * @brief Compute the sample_size as Radar constructor does:
   *        sample_size = (freq_time[last] - freq_time[first]) * fs
   */
  int GetSampleSize() const {
    if (num_samples_ < 2) return 0;
    return static_cast<int>((freq_time.back() - freq_time.front()) * fs_);
  }

  /**
   * @brief Build timestamps array for noise simulator.
   *
   * Shape: [v_channel_size, pulse_size, sample_size]
   * ts[ch][p][s] = pulse_start_time[p] + s / fs
   *
   * With 1 tx channel and 1 rx channel, v_channel_size = 1.
   */
  std::vector<double> BuildTimestamps() const {
    int sample_size = GetSampleSize();
    int v_channel_size = 1;  // 1 tx * 1 rx
    int total = v_channel_size * num_pulses_ * sample_size;
    std::vector<double> ts(total);

    for (int ch = 0; ch < v_channel_size; ch++) {
      for (int p = 0; p < num_pulses_; p++) {
        for (int s = 0; s < sample_size; s++) {
          int idx = ch * num_pulses_ * sample_size + p * sample_size + s;
          ts[idx] = pulse_start_time[p] + s / fs_;
        }
      }
    }
    return ts;
  }

  void Destroy() {
    if (radar) {
      Free_Radar(radar);
      radar = nullptr;
    }
    if (rx) {
      Free_Receiver(rx);
      rx = nullptr;
    }
    if (tx) {
      Free_Transmitter(tx);
      tx = nullptr;
    }
  }
};

/**
 * @brief Test fixture for Simulator tests
 */
class SimulatorTest : public ::testing::Test {
 protected:
  void TearDown() override { setup_.Destroy(); }

  RadarSetup setup_;
};

/*********************************************
 *
 *  Noise Simulator Tests
 *
 *********************************************/

/**
 * @brief Test noise simulator with valid parameters
 */
TEST_F(SimulatorTest, NoiseSimulatorValid) {
  ASSERT_TRUE(setup_.Create(64, 1));

  int sample_size = setup_.GetSampleSize();
  ASSERT_GT(sample_size, 0);
  int v_channel_size = 1;
  int pulse_size = 1;

  auto timestamps = setup_.BuildTimestamps();
  int out_size = 1 * v_channel_size * pulse_size * sample_size;  // 1 frame
  std::vector<double> noise_real(out_size, 0.0);
  std::vector<double> noise_imag(out_size, 0.0);

  int result = Run_NoiseSimulator(setup_.radar, 1.0, false, timestamps.data(),
                                  v_channel_size, pulse_size, sample_size,
                                  noise_real.data(), noise_imag.data(), 42);
  EXPECT_EQ(result, RADARSIM_SUCCESS);
}

/**
 * @brief Test noise simulator with complex noise
 */
TEST_F(SimulatorTest, NoiseSimulatorComplex) {
  ASSERT_TRUE(setup_.Create(64, 1));

  int sample_size = setup_.GetSampleSize();
  ASSERT_GT(sample_size, 0);
  int v_channel_size = 1;
  int pulse_size = 1;

  auto timestamps = setup_.BuildTimestamps();
  int out_size = 1 * v_channel_size * pulse_size * sample_size;
  std::vector<double> noise_real(out_size, 0.0);
  std::vector<double> noise_imag(out_size, 0.0);

  int result = Run_NoiseSimulator(setup_.radar, 1.0, true, timestamps.data(),
                                  v_channel_size, pulse_size, sample_size,
                                  noise_real.data(), noise_imag.data(), 42);
  EXPECT_EQ(result, RADARSIM_SUCCESS);

  // For complex noise, both real and imag should have non-zero values
  bool has_nonzero_imag = false;
  for (int i = 0; i < out_size; i++) {
    if (noise_imag[i] != 0.0) {
      has_nonzero_imag = true;
      break;
    }
  }
  EXPECT_TRUE(has_nonzero_imag);
}

/**
 * @brief Test noise simulator with null radar pointer
 */
TEST_F(SimulatorTest, NoiseSimulatorNullRadar) {
  double noise_real[64] = {};
  double noise_imag[64] = {};
  double timestamps[64] = {};

  int result = Run_NoiseSimulator(nullptr, 1.0, false, timestamps, 1, 1, 64,
                                  noise_real, noise_imag, 0);
  EXPECT_EQ(result, RADARSIM_ERROR_NULL_POINTER);
}

/**
 * @brief Test noise simulator with null output buffers
 */
TEST_F(SimulatorTest, NoiseSimulatorNullBuffers) {
  ASSERT_TRUE(setup_.Create(64, 1));

  double noise_real[64] = {};
  double timestamps[64] = {};

  // Null noise_real
  int result = Run_NoiseSimulator(setup_.radar, 1.0, false, timestamps, 1, 1,
                                  64, nullptr, noise_real, 0);
  EXPECT_EQ(result, RADARSIM_ERROR_NULL_POINTER);

  // Null noise_imag
  result = Run_NoiseSimulator(setup_.radar, 1.0, false, timestamps, 1, 1, 64,
                              noise_real, nullptr, 0);
  EXPECT_EQ(result, RADARSIM_ERROR_NULL_POINTER);
}

/**
 * @brief Test noise simulator with invalid noise level
 */
TEST_F(SimulatorTest, NoiseSimulatorInvalidLevel) {
  ASSERT_TRUE(setup_.Create(64, 1));

  double noise_real[64] = {};
  double noise_imag[64] = {};
  double timestamps[64] = {};

  int result = Run_NoiseSimulator(setup_.radar, -1.0, false, timestamps, 1, 1,
                                  64, noise_real, noise_imag, 0);
  EXPECT_EQ(result, RADARSIM_ERROR_INVALID_PARAMETER);
}

/**
 * @brief Test noise simulator with zero noise level
 */
TEST_F(SimulatorTest, NoiseSimulatorZeroLevel) {
  ASSERT_TRUE(setup_.Create(64, 1));

  int sample_size = setup_.GetSampleSize();
  ASSERT_GT(sample_size, 0);
  int v_channel_size = 1;
  int pulse_size = 1;

  auto timestamps = setup_.BuildTimestamps();
  int out_size = 1 * v_channel_size * pulse_size * sample_size;
  std::vector<double> noise_real(out_size, 0.0);
  std::vector<double> noise_imag(out_size, 0.0);

  int result = Run_NoiseSimulator(setup_.radar, 0.0, false, timestamps.data(),
                                  v_channel_size, pulse_size, sample_size,
                                  noise_real.data(), noise_imag.data(), 42);
  EXPECT_EQ(result, RADARSIM_SUCCESS);

  // Zero noise level should produce all zeros
  for (int i = 0; i < out_size; i++) {
    EXPECT_DOUBLE_EQ(noise_real[i], 0.0);
  }
}

/**
 * @brief Test noise simulator with deterministic seed
 */
TEST_F(SimulatorTest, NoiseSimulatorDeterministicSeed) {
  ASSERT_TRUE(setup_.Create(64, 1));

  int sample_size = setup_.GetSampleSize();
  ASSERT_GT(sample_size, 0);
  int v_channel_size = 1;
  int pulse_size = 1;

  auto timestamps = setup_.BuildTimestamps();
  int out_size = 1 * v_channel_size * pulse_size * sample_size;
  std::vector<double> noise_real_1(out_size, 0.0);
  std::vector<double> noise_imag_1(out_size, 0.0);
  std::vector<double> noise_real_2(out_size, 0.0);
  std::vector<double> noise_imag_2(out_size, 0.0);

  // Run twice with same seed
  int result1 = Run_NoiseSimulator(
      setup_.radar, 1.0, false, timestamps.data(), v_channel_size, pulse_size,
      sample_size, noise_real_1.data(), noise_imag_1.data(), 12345);
  int result2 = Run_NoiseSimulator(
      setup_.radar, 1.0, false, timestamps.data(), v_channel_size, pulse_size,
      sample_size, noise_real_2.data(), noise_imag_2.data(), 12345);

  EXPECT_EQ(result1, RADARSIM_SUCCESS);
  EXPECT_EQ(result2, RADARSIM_SUCCESS);

  // Same seed should produce same results
  for (int i = 0; i < out_size; i++) {
    EXPECT_DOUBLE_EQ(noise_real_1[i], noise_real_2[i]);
    EXPECT_DOUBLE_EQ(noise_imag_1[i], noise_imag_2[i]);
  }
}

/*********************************************
 *
 *  Interference Simulator Tests
 *
 *********************************************/

/**
 * @brief Test interference simulator returns int error code
 */
TEST_F(SimulatorTest, InterferenceSimulatorReturnType) {
  ASSERT_TRUE(setup_.Create(64, 1));

  // Create a second radar as interferer
  RadarSetup interf_setup;
  ASSERT_TRUE(interf_setup.Create(64, 1));

  int bb_size = Get_BB_Size(setup_.radar);
  ASSERT_GT(bb_size, 0);

  std::vector<double> interf_real(bb_size, 0.0);
  std::vector<double> interf_imag(bb_size, 0.0);

  // Run interference simulation - verify it returns int
  int result = Run_InterferenceSimulator(setup_.radar, interf_setup.radar,
                                         interf_real.data(),
                                         interf_imag.data());
  EXPECT_EQ(result, RADARSIM_SUCCESS);

  interf_setup.Destroy();
}

/*********************************************
 *
 *  Radar Simulator Tests
 *
 *********************************************/

/**
 * @brief Test radar simulator with point targets
 */
TEST_F(SimulatorTest, RadarSimulatorPointTarget) {
  ASSERT_TRUE(setup_.Create(64, 1));

  // Create targets
  t_Targets* targets = Init_Targets();
  ASSERT_NE(targets, nullptr);

  float target_loc[3] = {100.0f, 0.0f, 0.0f};
  float target_speed[3] = {0.0f, 0.0f, 0.0f};
  int result =
      Add_Point_Target(target_loc, target_speed, 10.0f, 0.0f, targets);
  EXPECT_EQ(result, RADARSIM_SUCCESS);

  int bb_size = Get_BB_Size(setup_.radar);
  ASSERT_GT(bb_size, 0);

  std::vector<double> bb_real(bb_size, 0.0);
  std::vector<double> bb_imag(bb_size, 0.0);

  int ray_filter[2] = {0, 0};
  result = Run_RadarSimulator(setup_.radar, targets, 0, 1.0f, ray_filter,
                              bb_real.data(), bb_imag.data());
  EXPECT_EQ(result, RADARSIM_SUCCESS);

  Free_Targets(targets);
}
