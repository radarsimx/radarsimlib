/*
 * @file test_receiver.cpp
 * @brief Unit tests for Receiver C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Receiver creation and destruction
 * - Channel addition
 * - Parameter validation
 * - Memory management
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
#include "radarsim.h"
#include <vector>
#include <cmath>

// Define constants since type_def.hpp is not directly accessible
#define kPI 3.141592653589793

/**
 * @brief Test fixture for Receiver tests
 */
class ReceiverTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup test data
        setupTestData();
    }

    void TearDown() override {
        // Cleanup receivers
        if (valid_receiver) {
            Free_Receiver(valid_receiver);
            valid_receiver = nullptr;
        }
    }

    void setupTestData() {
        // Setup receiver parameters
        fs = 1e6; // 1 MHz sampling rate
        rf_gain = 30.0f; // 30 dB RF gain
        resistor = 50.0f; // 50 Ohm
        baseband_gain = 20.0f; // 20 dB baseband gain
        baseband_bw = 500e3; // 500 kHz baseband bandwidth

        // Setup channel parameters
        location[0] = 0.0f; location[1] = 0.0f; location[2] = 0.0f;
        polar_real[0] = 1.0f; polar_real[1] = 0.0f; polar_real[2] = 0.0f;
        polar_imag[0] = 0.0f; polar_imag[1] = 0.0f; polar_imag[2] = 0.0f;

        // Setup radiation pattern
        phi_length = 2;
        phi[0] = -kPI/2; phi[1] = kPI/2;
        phi_ptn[0] = 0.0f; phi_ptn[1] = 0.0f;

        theta_length = 2;
        theta[0] = 0.0f; theta[1] = kPI;
        theta_ptn[0] = 0.0f; theta_ptn[1] = 0.0f;

        antenna_gain = 20.0f; // 20 dB
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
    valid_receiver = Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
    
    EXPECT_NE(valid_receiver, nullptr);
    // Note: No Is_Valid_Pointer function available in the C API
}

/**
 * @brief Test receiver creation with invalid parameters
 */
TEST_F(ReceiverTest, CreateReceiverInvalidParams) {
    // Test with zero sampling rate
    t_Receiver* rx = Create_Receiver(0.0f, rf_gain, resistor, baseband_gain, baseband_bw);
    EXPECT_EQ(rx, nullptr);

    // Test with negative sampling rate
    rx = Create_Receiver(-1.0f, rf_gain, resistor, baseband_gain, baseband_bw);
    EXPECT_EQ(rx, nullptr);

    // Test with zero resistor
    rx = Create_Receiver(fs, rf_gain, 0.0f, baseband_gain, baseband_bw);
    EXPECT_EQ(rx, nullptr);
}

/**
 * @brief Test adding receiver channels
 */
TEST_F(ReceiverTest, AddRxChannel) {
    valid_receiver = Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
    ASSERT_NE(valid_receiver, nullptr);

    // Test adding valid channel
    int result = Add_Rxchannel(
        location, polar_real, polar_imag,
        phi, phi_ptn, phi_length,
        theta, theta_ptn, theta_length,
        antenna_gain,
        valid_receiver
    );
    
    EXPECT_EQ(result, 0); // 0 for success according to API
    EXPECT_EQ(Get_Num_Rxchannel(valid_receiver), 1);
}

/**
 * @brief Test adding receiver channel with null parameters
 */
TEST_F(ReceiverTest, AddRxChannelNullParams) {
    valid_receiver = Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
    ASSERT_NE(valid_receiver, nullptr);

    // Test with null location
    int result = Add_Rxchannel(
        nullptr, polar_real, polar_imag,
        phi, phi_ptn, phi_length,
        theta, theta_ptn, theta_length,
        antenna_gain,
        valid_receiver
    );
    EXPECT_NE(result, 0); // Non-zero for failure

    // Test with null polarization
    result = Add_Rxchannel(
        location, nullptr, polar_imag,
        phi, phi_ptn, phi_length,
        theta, theta_ptn, theta_length,
        antenna_gain,
        valid_receiver
    );
    EXPECT_NE(result, 0); // Non-zero for failure

    // Test with null phi array
    result = Add_Rxchannel(
        location, polar_real, polar_imag,
        nullptr, phi_ptn, phi_length,
        theta, theta_ptn, theta_length,
        antenna_gain,
        valid_receiver
    );
    EXPECT_NE(result, 0); // Non-zero for failure
}

/**
 * @brief Test getting number of channels
 */
TEST_F(ReceiverTest, GetNumRxChannels) {
    valid_receiver = Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
    ASSERT_NE(valid_receiver, nullptr);

    // Initially should have 0 channels
    EXPECT_EQ(Get_Num_Rxchannel(valid_receiver), 0);

    // Add a channel
    Add_Rxchannel(
        location, polar_real, polar_imag,
        phi, phi_ptn, phi_length,
        theta, theta_ptn, theta_length,
        antenna_gain,
        valid_receiver
    );

    EXPECT_EQ(Get_Num_Rxchannel(valid_receiver), 1);

    // Add another channel
    Add_Rxchannel(
        location, polar_real, polar_imag,
        phi, phi_ptn, phi_length,
        theta, theta_ptn, theta_length,
        antenna_gain,
        valid_receiver
    );

    EXPECT_EQ(Get_Num_Rxchannel(valid_receiver), 2);
}

/**
 * @brief Test receiver destruction
 */
TEST_F(ReceiverTest, FreeReceiver) {
    valid_receiver = Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw);
    ASSERT_NE(valid_receiver, nullptr);

    // Should not crash
    Free_Receiver(valid_receiver);
    valid_receiver = nullptr; // Prevent double free in teardown

    // Should handle null pointer gracefully
    Free_Receiver(nullptr);
}
