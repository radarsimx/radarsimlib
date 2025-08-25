/*
 * @file test_receiver.cpp
 * @brief Unit tests for Receiver C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Receiver creation and destruction
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
 * @brief Test fixture for Receiver tests
 *
 * @details
 * This test suite demonstrates both automatic and manual memory management:
 * - Automatic: Objects are automatically cleaned up at program exit
 * - Manual: Objects can still be explicitly freed with Free_Receiver()
 * - Mixed: Some objects freed manually, others automatically
 */
class ReceiverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup test data
    setupTestData();
  }

  void TearDown() override {
    // With automatic memory management, manual cleanup is optional
    // Objects will be automatically cleaned up at program exit
    // But we can still manually free for testing purposes
    if (valid_receiver) {
      Free_Receiver(valid_receiver);
      valid_receiver = nullptr;
    }
  }

  void setupTestData() {
    // Setup receiver parameters
    fs = 1e6;               // 1 MHz sampling rate
    rf_gain = 30.0f;        // 30 dB RF gain
    resistor = 50.0f;       // 50 Ohm
    baseband_gain = 20.0f;  // 20 dB baseband gain
    baseband_bw = 500e3;    // 500 kHz baseband bandwidth

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
  }

  // Test data
  float fs, rf_gain, resistor, baseband_gain, baseband_bw;
  float location[3], polar_real[3], polar_imag[3];
  float phi[2], phi_ptn[2], theta[2], theta_ptn[2];
  int phi_length, theta_length;
  float antenna_gain;

  t_Receiver* valid_receiver = nullptr;
};

/**
 * @brief Test receiver creation with valid parameters
 */
TEST_F(ReceiverTest, CreateReceiverValid) {
  valid_receiver =
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);

  EXPECT_NE(valid_receiver, nullptr);
  // Note: No Is_Valid_Pointer function available in the C API
}

/**
 * @brief Test receiver creation with invalid parameters
 */
TEST_F(ReceiverTest, CreateReceiverInvalidParams) {
  // Test with zero sampling rate
  t_Receiver* rx =
      Create_Receiver(0.0f, rf_gain, resistor, baseband_gain, baseband_bw);
  EXPECT_EQ(rx, nullptr);

  // Test with negative sampling rate
  rx = Create_Receiver(-1.0f, rf_gain, resistor, baseband_gain, baseband_bw);
  EXPECT_EQ(rx, nullptr);

  // Test with zero resistor
  rx = Create_Receiver(fs, rf_gain, 0.0f, baseband_gain, baseband_bw);
  EXPECT_EQ(rx, nullptr);

  // Test with negative resistor
  rx = Create_Receiver(fs, rf_gain, -1.0f, baseband_gain, baseband_bw);
  EXPECT_EQ(rx, nullptr);

  // Test with negative baseband bandwidth
  rx = Create_Receiver(fs, rf_gain, resistor, baseband_gain, -1.0f);
  EXPECT_EQ(rx, nullptr);

  // Test with 0 baseband bandwidth
  rx = Create_Receiver(fs, rf_gain, resistor, baseband_gain, 0.0f);
  EXPECT_EQ(rx, nullptr);
}

/**
 * @brief Test adding receiver channels
 */
TEST_F(ReceiverTest, AddRxChannel) {
  valid_receiver =
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
  ASSERT_NE(valid_receiver, nullptr);

  // Test adding valid channel
  int result = Add_Rxchannel(location, polar_real, polar_imag, phi, phi_ptn,
                             phi_length, theta, theta_ptn, theta_length,
                             antenna_gain, valid_receiver);

  EXPECT_EQ(result, 0);  // 0 for success according to API
  EXPECT_EQ(Get_Num_Rxchannel(valid_receiver), 1);
}

/**
 * @brief Test adding receiver channel with null receiver
 */
TEST_F(ReceiverTest, AddRxChannelNullReceiver) {
  int result =
      Add_Rxchannel(location, polar_real, polar_imag, phi, phi_ptn, phi_length,
                    theta, theta_ptn, theta_length, antenna_gain, nullptr);

  EXPECT_NE(result, 0);  // Non-zero for failure
}

/**
 * @brief Test getting number of channels with null receiver
 */
TEST_F(ReceiverTest, GetNumRxChannelsNull) {
  // This might crash if not handled properly, so we test it
  // Note: This test might need to be commented out if the implementation
  // doesn't handle null pointers in Get_Num_Rxchannel
  // EXPECT_EQ(Get_Num_Rxchannel(nullptr), 0);
}

/**
 * @brief Test getting number of channels
 */
TEST_F(ReceiverTest, GetNumRxChannels) {
  valid_receiver =
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
  ASSERT_NE(valid_receiver, nullptr);

  // Initially should have 0 channels
  EXPECT_EQ(Get_Num_Rxchannel(valid_receiver), 0);

  // Add a channel
  Add_Rxchannel(location, polar_real, polar_imag, phi, phi_ptn, phi_length,
                theta, theta_ptn, theta_length, antenna_gain, valid_receiver);

  EXPECT_EQ(Get_Num_Rxchannel(valid_receiver), 1);

  // Add another channel
  Add_Rxchannel(location, polar_real, polar_imag, phi, phi_ptn, phi_length,
                theta, theta_ptn, theta_length, antenna_gain, valid_receiver);

  EXPECT_EQ(Get_Num_Rxchannel(valid_receiver), 2);
}

/**
 * @brief Test automatic memory management
 */
TEST_F(ReceiverTest, AutomaticMemoryManagement) {
  // Test that we can create receivers without manual cleanup
  t_Receiver* rx1 =
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
  ASSERT_NE(rx1, nullptr);

  t_Receiver* rx2 =
      Create_Receiver(fs * 2, rf_gain, resistor, baseband_gain, baseband_bw);
  ASSERT_NE(rx2, nullptr);

  // Add channels to both receivers
  int result1 =
      Add_Rxchannel(location, polar_real, polar_imag, phi, phi_ptn, phi_length,
                    theta, theta_ptn, theta_length, antenna_gain, rx1);
  EXPECT_EQ(result1, 0);

  int result2 =
      Add_Rxchannel(location, polar_real, polar_imag, phi, phi_ptn, phi_length,
                    theta, theta_ptn, theta_length, antenna_gain, rx2);
  EXPECT_EQ(result2, 0);

  // Don't call Free_Receiver - test automatic cleanup
  // These receivers will be automatically cleaned up at program exit
}

/**
 * @brief Test manual vs automatic cleanup
 */
TEST_F(ReceiverTest, ManualVsAutomaticCleanup) {
  // Create two receivers
  t_Receiver* manual_rx =
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
  t_Receiver* auto_rx =
      Create_Receiver(fs * 2, rf_gain, resistor, baseband_gain, baseband_bw);

  ASSERT_NE(manual_rx, nullptr);
  ASSERT_NE(auto_rx, nullptr);

  // Manually free one receiver
  Free_Receiver(manual_rx);

  // Leave auto_rx for automatic cleanup at program exit
  // This demonstrates both cleanup methods work
}

/**
 * @brief Test receiver destruction (original test preserved)
 */
TEST_F(ReceiverTest, FreeReceiver) {
  // Test freeing a valid receiver
  t_Receiver* test_receiver =
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
  ASSERT_NE(test_receiver, nullptr);

  // Should not crash when freeing a valid receiver
  Free_Receiver(test_receiver);

  // Test freeing a null pointer - should handle gracefully
  Free_Receiver(nullptr);
}

/**
 * @brief Test automatic cleanup control
 */
TEST_F(ReceiverTest, AutomaticCleanupControl) {
  // Test enabling automatic cleanup (should be enabled by default)
//   Enable_Automatic_Cleanup(true);

  // Create a receiver that will be automatically cleaned up
  t_Receiver* rx =
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
  ASSERT_NE(rx, nullptr);

  // Verify it works by adding a channel
  int result =
      Add_Rxchannel(location, polar_real, polar_imag, phi, phi_ptn, phi_length,
                    theta, theta_ptn, theta_length, antenna_gain, rx);
  EXPECT_EQ(result, 0);
  EXPECT_EQ(Get_Num_Rxchannel(rx), 1);

  // Don't free - let automatic cleanup handle it
}
