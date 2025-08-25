/*
 * @file test_transmitter.cpp
 * @brief Unit tests for Transmitter C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Transmitter creation and destruction
 * - Channel addition
 * - Parameter validation
 * - Automatic memory management
 * - Manual vs automatic cleanup
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

// Define constants since type_def.hpp is not directly accessible
#define kPI 3.141592653589793

/**
 * @brief Test fixture for Transmitter tests
 *
 * @details
 * This test suite demonstrates both automatic and manual memory management:
 * - Automatic: Objects are automatically cleaned up at program exit
 * - Manual: Objects can still be explicitly freed with Free_Transmitter()
 * - Mixed: Some objects freed manually, others automatically
 */
class TransmitterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup test data
    setupTestData();
  }

  void TearDown() override {
    // With automatic memory management, manual cleanup is optional
    // Objects will be automatically cleaned up at program exit
    // But we can still manually free for testing purposes
    if (valid_transmitter) {
      Free_Transmitter(valid_transmitter);
      valid_transmitter = nullptr;
    }
  }

  void setupTestData() {
    // Setup frequency sweep (24 GHz +/- 150 MHz)
    num_samples = 100;
    freq.resize(num_samples);
    freq_time.resize(num_samples);

    for (int i = 0; i < num_samples; i++) {
      freq[i] = 24.0e9 + (i - num_samples / 2) * 3e6;  // 24 GHz +/- 150 MHz
      freq_time[i] = i * 1e-6;                         // 1 microsecond samples
    }

    // Setup pulse parameters
    num_pulses = 256;
    freq_offset.resize(num_pulses, 0.0);
    pulse_start_time.resize(num_pulses);

    for (int i = 0; i < num_pulses; i++) {
      pulse_start_time[i] = i * 100e-6;  // 100 microsecond PRI
    }

    tx_power = 10.0f;  // 10 dBm

    // Setup channel parameters
    location[0] = 0.0f;
    location[1] = 0.0f;
    location[2] = 0.0f;
    polar_real[0] = 1.0f;
    polar_real[1] = 0.0f;
    polar_real[2] = 0.0f;
    polar_imag[0] = 0.0f;
    polar_imag[1] = 0.0f;
    polar_imag[2] = 0.0f;

    // Setup radiation pattern
    phi_length = 2;
    phi[0] = -static_cast<float>(kPI) / 2.0f;
    phi[1] = static_cast<float>(kPI) / 2.0f;
    phi_ptn[0] = 0.0f;
    phi_ptn[1] = 0.0f;

    theta_length = 2;
    theta[0] = 0.0f;
    theta[1] = static_cast<float>(kPI);
    theta_ptn[0] = 0.0f;
    theta_ptn[1] = 0.0f;

    antenna_gain = 20.0f;  // 20 dB

    // Setup modulation
    mod_length = 10;
    mod_t.resize(mod_length);
    mod_var_real.resize(mod_length);
    mod_var_imag.resize(mod_length);

    for (int i = 0; i < mod_length; i++) {
      mod_t[i] = static_cast<float>(i * 1e-8);  // 10 ns samples
      mod_var_real[i] = 1.0f;
      mod_var_imag[i] = 0.0f;
    }

    // Setup pulse modulation
    pulse_mod_real.resize(num_pulses, 1.0f);
    pulse_mod_imag.resize(num_pulses, 0.0f);

    delay = 0.0f;
    grid = 0.1f;  // 0.1 rad grid
  }

  // Test data
  std::vector<double> freq, freq_time, freq_offset, pulse_start_time;
  std::vector<float> mod_t, mod_var_real, mod_var_imag, pulse_mod_real,
      pulse_mod_imag;
  int num_samples, num_pulses, phi_length, theta_length, mod_length;
  float tx_power;
  float location[3], polar_real[3], polar_imag[3];
  float phi[2], phi_ptn[2], theta[2], theta_ptn[2];
  float antenna_gain, delay, grid;

  t_Transmitter* valid_transmitter = nullptr;
};

/**
 * @brief Test transmitter creation with valid parameters
 */
TEST_F(TransmitterTest, CreateTransmitterValid) {
  valid_transmitter = Create_Transmitter(
      freq.data(), freq_time.data(), num_samples, freq_offset.data(),
      pulse_start_time.data(), num_pulses, tx_power);

  EXPECT_NE(valid_transmitter, nullptr);
  // Note: No Is_Valid_Pointer function available in the C API
}

/**
 * @brief Test transmitter creation with null parameters
 */
TEST_F(TransmitterTest, CreateTransmitterNullParams) {
  // Test with null frequency array
  t_Transmitter* tx = Create_Transmitter(
      nullptr, freq_time.data(), num_samples, freq_offset.data(),
      pulse_start_time.data(), num_pulses, tx_power);
  EXPECT_EQ(tx, nullptr);

  // Test with null time array
  tx = Create_Transmitter(freq.data(), nullptr, num_samples, freq_offset.data(),
                          pulse_start_time.data(), num_pulses, tx_power);
  EXPECT_EQ(tx, nullptr);

  // Test with null frequency offset
  tx = Create_Transmitter(freq.data(), freq_time.data(), num_samples, nullptr,
                          pulse_start_time.data(), num_pulses, tx_power);
  EXPECT_EQ(tx, nullptr);

  // Test with null pulse start time
  tx = Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                          freq_offset.data(), nullptr, num_pulses, tx_power);
  EXPECT_EQ(tx, nullptr);
}

/**
 * @brief Test transmitter creation with invalid parameters
 */
TEST_F(TransmitterTest, CreateTransmitterInvalidParams) {
  // Test with zero waveform size
  t_Transmitter* tx =
      Create_Transmitter(freq.data(), freq_time.data(), 0, freq_offset.data(),
                         pulse_start_time.data(), num_pulses, tx_power);
  EXPECT_EQ(tx, nullptr);

  // Test with zero pulses
  tx = Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                          freq_offset.data(), pulse_start_time.data(), 0,
                          tx_power);
  EXPECT_EQ(tx, nullptr);
}

/**
 * @brief Test adding transmitter channels
 */
TEST_F(TransmitterTest, AddTxChannel) {
  valid_transmitter = Create_Transmitter(
      freq.data(), freq_time.data(), num_samples, freq_offset.data(),
      pulse_start_time.data(), num_pulses, tx_power);
  ASSERT_NE(valid_transmitter, nullptr);

  // Test adding valid channel
  int result = Add_Txchannel(
      location, polar_real, polar_imag, phi, phi_ptn, phi_length, theta,
      theta_ptn, theta_length, antenna_gain, mod_t.data(), mod_var_real.data(),
      mod_var_imag.data(), mod_length, pulse_mod_real.data(),
      pulse_mod_imag.data(), delay, grid, valid_transmitter);

  EXPECT_EQ(result, 0);  // 0 for success according to API
  EXPECT_EQ(Get_Num_Txchannel(valid_transmitter), 1);
}

/**
 * @brief Test adding transmitter channel with null transmitter
 */
TEST_F(TransmitterTest, AddTxChannelNullTransmitter) {
  int result = Add_Txchannel(
      location, polar_real, polar_imag, phi, phi_ptn, phi_length, theta,
      theta_ptn, theta_length, antenna_gain, mod_t.data(), mod_var_real.data(),
      mod_var_imag.data(), mod_length, pulse_mod_real.data(),
      pulse_mod_imag.data(), delay, grid, nullptr);

  EXPECT_NE(result, 0);  // Non-zero for failure
}

/**
 * @brief Test getting number of channels
 */
TEST_F(TransmitterTest, GetNumTxChannels) {
  valid_transmitter = Create_Transmitter(
      freq.data(), freq_time.data(), num_samples, freq_offset.data(),
      pulse_start_time.data(), num_pulses, tx_power);
  ASSERT_NE(valid_transmitter, nullptr);

  // Initially should have 0 channels
  EXPECT_EQ(Get_Num_Txchannel(valid_transmitter), 0);

  // Add a channel
  Add_Txchannel(location, polar_real, polar_imag, phi, phi_ptn, phi_length,
                theta, theta_ptn, theta_length, antenna_gain, mod_t.data(),
                mod_var_real.data(), mod_var_imag.data(), mod_length,
                pulse_mod_real.data(), pulse_mod_imag.data(), delay, grid,
                valid_transmitter);

  EXPECT_EQ(Get_Num_Txchannel(valid_transmitter), 1);
}

/**
 * @brief Test automatic memory management
 */
TEST_F(TransmitterTest, AutomaticMemoryManagement) {
  // Test that we can create transmitters without manual cleanup
  t_Transmitter* tx1 = Create_Transmitter(
      freq.data(), freq_time.data(), num_samples, freq_offset.data(),
      pulse_start_time.data(), num_pulses, tx_power);
  ASSERT_NE(tx1, nullptr);

  t_Transmitter* tx2 = Create_Transmitter(freq.data(), freq_time.data(),
                                          num_samples, freq_offset.data(),
                                          pulse_start_time.data(), num_pulses,
                                          tx_power + 5.0f  // Different power
  );
  ASSERT_NE(tx2, nullptr);

  // Add channels to both transmitters
  int result1 = Add_Txchannel(
      location, polar_real, polar_imag, phi, phi_ptn, phi_length, theta,
      theta_ptn, theta_length, antenna_gain, mod_t.data(), mod_var_real.data(),
      mod_var_imag.data(), mod_length, pulse_mod_real.data(),
      pulse_mod_imag.data(), delay, grid, tx1);
  EXPECT_EQ(result1, 0);

  int result2 = Add_Txchannel(
      location, polar_real, polar_imag, phi, phi_ptn, phi_length, theta,
      theta_ptn, theta_length, antenna_gain, mod_t.data(), mod_var_real.data(),
      mod_var_imag.data(), mod_length, pulse_mod_real.data(),
      pulse_mod_imag.data(), delay, grid, tx2);
  EXPECT_EQ(result2, 0);

  // Don't call Free_Transmitter - test automatic cleanup
  // These transmitters will be automatically cleaned up at program exit
}

/**
 * @brief Test manual vs automatic cleanup
 */
TEST_F(TransmitterTest, ManualVsAutomaticCleanup) {
  // Create two transmitters
  t_Transmitter* manual_tx = Create_Transmitter(
      freq.data(), freq_time.data(), num_samples, freq_offset.data(),
      pulse_start_time.data(), num_pulses, tx_power);
  t_Transmitter* auto_tx = Create_Transmitter(
      freq.data(), freq_time.data(), num_samples, freq_offset.data(),
      pulse_start_time.data(), num_pulses, tx_power + 5.0f);

  ASSERT_NE(manual_tx, nullptr);
  ASSERT_NE(auto_tx, nullptr);

  // Manually free one transmitter
  Free_Transmitter(manual_tx);

  // Leave auto_tx for automatic cleanup at program exit
  // This demonstrates both cleanup methods work
}

/**
 * @brief Test transmitter destruction (original test preserved)
 */
TEST_F(TransmitterTest, FreeTransmitter) {
  // Test freeing a valid transmitter
  t_Transmitter* test_transmitter = Create_Transmitter(
      freq.data(), freq_time.data(), num_samples, freq_offset.data(),
      pulse_start_time.data(), num_pulses, tx_power);
  ASSERT_NE(test_transmitter, nullptr);

  // Should not crash when freeing a valid transmitter
  Free_Transmitter(test_transmitter);

  // Test freeing a null pointer - should handle gracefully
  Free_Transmitter(nullptr);
}

/**
 * @brief Test automatic cleanup control
 */
TEST_F(TransmitterTest, AutomaticCleanupControl) {
  // Test enabling automatic cleanup (should be enabled by default)
  Enable_Automatic_Cleanup(true);

  // Create a transmitter that will be automatically cleaned up
  t_Transmitter* tx = Create_Transmitter(
      freq.data(), freq_time.data(), num_samples, freq_offset.data(),
      pulse_start_time.data(), num_pulses, tx_power);
  ASSERT_NE(tx, nullptr);

  // Verify it works by adding a channel
  int result = Add_Txchannel(
      location, polar_real, polar_imag, phi, phi_ptn, phi_length, theta,
      theta_ptn, theta_length, antenna_gain, mod_t.data(), mod_var_real.data(),
      mod_var_imag.data(), mod_length, pulse_mod_real.data(),
      pulse_mod_imag.data(), delay, grid, tx);
  EXPECT_EQ(result, 0);
  EXPECT_EQ(Get_Num_Txchannel(tx), 1);

  // Don't free - let automatic cleanup handle it
}
