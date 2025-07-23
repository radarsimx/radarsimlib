/*
 * @file test_transmitter.cpp
 * @brief Unit tests for Transmitter C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Transmitter creation and destruction
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
#include "type_def.hpp"

/**
 * @brief Test fixture for Transmitter tests
 */
class TransmitterTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup test data
        setupTestData();
    }

    void TearDown() override {
        // Cleanup transmitters
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
            freq[i] = 24.0e9 + (i - num_samples/2) * 3e6; // 24 GHz +/- 150 MHz
            freq_time[i] = i * 1e-6; // 1 microsecond samples
        }

        // Setup pulse parameters
        num_pulses = 256;
        freq_offset.resize(num_pulses, 0.0);
        pulse_start_time.resize(num_pulses);
        
        for (int i = 0; i < num_pulses; i++) {
            pulse_start_time[i] = i * 100e-6; // 100 microsecond PRI
        }

        tx_power = 10.0f; // 10 dBm

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

        // Setup modulation
        mod_length = 10;
        mod_t.resize(mod_length);
        mod_var_real.resize(mod_length);
        mod_var_imag.resize(mod_length);
        
        for (int i = 0; i < mod_length; i++) {
            mod_t[i] = i * 1e-8; // 10 ns samples
            mod_var_real[i] = 1.0f;
            mod_var_imag[i] = 0.0f;
        }

        // Setup pulse modulation
        pulse_mod_real.resize(num_pulses, 1.0f);
        pulse_mod_imag.resize(num_pulses, 0.0f);

        delay = 0.0f;
        grid = 0.1f; // 0.1 rad grid
    }

    // Test data
    std::vector<double> freq, freq_time, freq_offset, pulse_start_time;
    std::vector<float> mod_t, mod_var_real, mod_var_imag, pulse_mod_real, pulse_mod_imag;
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
        freq.data(), freq_time.data(), num_samples,
        freq_offset.data(), pulse_start_time.data(), num_pulses,
        tx_power
    );
    
    EXPECT_NE(valid_transmitter, nullptr);
    EXPECT_EQ(Is_Valid_Pointer(valid_transmitter), 1);
}

/**
 * @brief Test transmitter creation with null parameters
 */
TEST_F(TransmitterTest, CreateTransmitterNullParams) {
    // Test with null frequency array
    t_Transmitter* tx = Create_Transmitter(
        nullptr, freq_time.data(), num_samples,
        freq_offset.data(), pulse_start_time.data(), num_pulses,
        tx_power
    );
    EXPECT_EQ(tx, nullptr);

    // Test with null time array
    tx = Create_Transmitter(
        freq.data(), nullptr, num_samples,
        freq_offset.data(), pulse_start_time.data(), num_pulses,
        tx_power
    );
    EXPECT_EQ(tx, nullptr);

    // Test with null frequency offset
    tx = Create_Transmitter(
        freq.data(), freq_time.data(), num_samples,
        nullptr, pulse_start_time.data(), num_pulses,
        tx_power
    );
    EXPECT_EQ(tx, nullptr);

    // Test with null pulse start time
    tx = Create_Transmitter(
        freq.data(), freq_time.data(), num_samples,
        freq_offset.data(), nullptr, num_pulses,
        tx_power
    );
    EXPECT_EQ(tx, nullptr);
}

/**
 * @brief Test transmitter creation with invalid parameters
 */
TEST_F(TransmitterTest, CreateTransmitterInvalidParams) {
    // Test with zero waveform size
    t_Transmitter* tx = Create_Transmitter(
        freq.data(), freq_time.data(), 0,
        freq_offset.data(), pulse_start_time.data(), num_pulses,
        tx_power
    );
    EXPECT_EQ(tx, nullptr);

    // Test with zero pulses
    tx = Create_Transmitter(
        freq.data(), freq_time.data(), num_samples,
        freq_offset.data(), pulse_start_time.data(), 0,
        tx_power
    );
    EXPECT_EQ(tx, nullptr);
}

/**
 * @brief Test adding transmitter channels
 */
TEST_F(TransmitterTest, AddTxChannel) {
    valid_transmitter = Create_Transmitter(
        freq.data(), freq_time.data(), num_samples,
        freq_offset.data(), pulse_start_time.data(), num_pulses,
        tx_power
    );
    ASSERT_NE(valid_transmitter, nullptr);

    // Test adding valid channel
    int result = Add_Txchannel(
        location, polar_real, polar_imag,
        phi, phi_ptn, phi_length,
        theta, theta_ptn, theta_length,
        antenna_gain,
        mod_t.data(), mod_var_real.data(), mod_var_imag.data(), mod_length,
        pulse_mod_real.data(), pulse_mod_imag.data(),
        delay, grid,
        valid_transmitter
    );
    
    EXPECT_EQ(result, RADARSIM_SUCCESS);
    EXPECT_EQ(Get_Num_Txchannel(valid_transmitter), 1);
}

/**
 * @brief Test adding transmitter channel with null transmitter
 */
TEST_F(TransmitterTest, AddTxChannelNullTransmitter) {
    int result = Add_Txchannel(
        location, polar_real, polar_imag,
        phi, phi_ptn, phi_length,
        theta, theta_ptn, theta_length,
        antenna_gain,
        mod_t.data(), mod_var_real.data(), mod_var_imag.data(), mod_length,
        pulse_mod_real.data(), pulse_mod_imag.data(),
        delay, grid,
        nullptr
    );
    
    EXPECT_NE(result, RADARSIM_SUCCESS);
}

/**
 * @brief Test getting number of channels
 */
TEST_F(TransmitterTest, GetNumTxChannels) {
    valid_transmitter = Create_Transmitter(
        freq.data(), freq_time.data(), num_samples,
        freq_offset.data(), pulse_start_time.data(), num_pulses,
        tx_power
    );
    ASSERT_NE(valid_transmitter, nullptr);

    // Initially should have 0 channels
    EXPECT_EQ(Get_Num_Txchannel(valid_transmitter), 0);

    // Add a channel
    Add_Txchannel(
        location, polar_real, polar_imag,
        phi, phi_ptn, phi_length,
        theta, theta_ptn, theta_length,
        antenna_gain,
        mod_t.data(), mod_var_real.data(), mod_var_imag.data(), mod_length,
        pulse_mod_real.data(), pulse_mod_imag.data(),
        delay, grid,
        valid_transmitter
    );

    EXPECT_EQ(Get_Num_Txchannel(valid_transmitter), 1);
}

/**
 * @brief Test transmitter destruction
 */
TEST_F(TransmitterTest, FreeTransmitter) {
    valid_transmitter = Create_Transmitter(
        freq.data(), freq_time.data(), num_samples,
        freq_offset.data(), pulse_start_time.data(), num_pulses,
        tx_power
    );
    ASSERT_NE(valid_transmitter, nullptr);

    // Should not crash
    Free_Transmitter(valid_transmitter);
    valid_transmitter = nullptr; // Prevent double free in teardown

    // Should handle null pointer gracefully
    Free_Transmitter(nullptr);
}
