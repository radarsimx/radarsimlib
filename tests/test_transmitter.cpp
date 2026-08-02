/*
 * @file test_transmitter.cpp
 * @brief Unit tests for Transmitter C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Transmitter creation with valid, null and out-of-range parameters
 * - SSB phase noise transmitter creation
 * - Channel addition and channel counting
 * - Free-tier channel limit enforcement
 * - Destruction and null-safety
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
#include <limits>
#include <vector>

#include "radarsim.h"
#include "test_helpers.hpp"

using rstest::IsotropicAntenna;
using rstest::LinearPolarization;
using rstest::TransmitterPtr;

namespace {

/**
 * @brief Test fixture owning the waveform and channel parameters
 *
 * @details Every transmitter created through Create() is owned by the fixture,
 * so a failing assertion cannot leak a handle.
 */
class TransmitterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    num_samples = 100;
    freq.resize(num_samples);
    freq_time.resize(num_samples);
    for (int i = 0; i < num_samples; i++) {
      // 24 GHz +/- 150 MHz over 100 us
      freq[i] = 24.0e9 + (i - num_samples / 2) * 3e6;
      freq_time[i] = i * 1e-6;
    }

    num_pulses = 256;
    freq_offset.assign(num_pulses, 0.0);
    pulse_start_time.resize(num_pulses);
    for (int i = 0; i < num_pulses; i++) {
      pulse_start_time[i] = i * 100e-6;  // 100 us PRI
    }

    mod_length = 10;
    mod_t.resize(mod_length);
    mod_var_real.assign(mod_length, 1.0f);
    mod_var_imag.assign(mod_length, 0.0f);
    for (int i = 0; i < mod_length; i++) {
      mod_t[i] = static_cast<float>(i * 1e-8);  // 10 ns steps
    }

    pulse_mod_real.assign(num_pulses, 1.0f);
    pulse_mod_imag.assign(num_pulses, 0.0f);
  }

  /** @brief Create a transmitter owned by the fixture */
  TransmitterPtr Create(float power = 10.0f) {
    return TransmitterPtr(Create_Transmitter(
        freq.data(), freq_time.data(), num_samples, freq_offset.data(),
        pulse_start_time.data(), num_pulses, power));
  }

  /** @brief Add the fixture's default channel to the given transmitter */
  int AddChannel(t_Transmitter* tx) {
    return Add_Txchannel(location, polarization.real, polarization.imag,
                         antenna.phi, antenna.phi_pattern,
                         IsotropicAntenna::kLength, antenna.theta,
                         antenna.theta_pattern, IsotropicAntenna::kLength,
                         IsotropicAntenna::kGain, mod_t.data(),
                         mod_var_real.data(), mod_var_imag.data(), mod_length,
                         pulse_mod_real.data(), pulse_mod_imag.data(), 0.0f,
                         0.1f, tx);
  }

  std::vector<double> freq, freq_time, freq_offset, pulse_start_time;
  std::vector<float> mod_t, mod_var_real, mod_var_imag;
  std::vector<float> pulse_mod_real, pulse_mod_imag;
  int num_samples = 0, num_pulses = 0, mod_length = 0;

  IsotropicAntenna antenna;
  LinearPolarization polarization;
  float location[3] = {0.0f, 0.0f, 0.0f};
};

/*********************************************
 *
 *  Creation
 *
 *********************************************/

TEST_F(TransmitterTest, CreateWithValidParameters) {
  TransmitterPtr tx = Create();

  ASSERT_NE(tx, nullptr);
  EXPECT_EQ(Get_Num_Txchannel(tx.get()), 0)
      << "A freshly created transmitter must have no channels";
}

TEST_F(TransmitterTest, CreateRejectsNullArrays) {
  EXPECT_EQ(Create_Transmitter(nullptr, freq_time.data(), num_samples,
                               freq_offset.data(), pulse_start_time.data(),
                               num_pulses, 10.0f),
            nullptr);
  EXPECT_EQ(Create_Transmitter(freq.data(), nullptr, num_samples,
                               freq_offset.data(), pulse_start_time.data(),
                               num_pulses, 10.0f),
            nullptr);
  EXPECT_EQ(Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                               nullptr, pulse_start_time.data(), num_pulses,
                               10.0f),
            nullptr);
  EXPECT_EQ(Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                               freq_offset.data(), nullptr, num_pulses, 10.0f),
            nullptr);
}

TEST_F(TransmitterTest, CreateRejectsNonPositiveSizes) {
  EXPECT_EQ(Create_Transmitter(freq.data(), freq_time.data(), 0,
                               freq_offset.data(), pulse_start_time.data(),
                               num_pulses, 10.0f),
            nullptr);
  EXPECT_EQ(Create_Transmitter(freq.data(), freq_time.data(), -1,
                               freq_offset.data(), pulse_start_time.data(),
                               num_pulses, 10.0f),
            nullptr);
  EXPECT_EQ(Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                               freq_offset.data(), pulse_start_time.data(), 0,
                               10.0f),
            nullptr);
  EXPECT_EQ(Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                               freq_offset.data(), pulse_start_time.data(), -1,
                               10.0f),
            nullptr);
}

/**
 * @brief Non-finite pulse parameters are rejected rather than propagated
 */
TEST_F(TransmitterTest, CreateRejectsNonFinitePulseParameters) {
  std::vector<double> bad_offset = freq_offset;
  bad_offset[num_pulses / 2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                               bad_offset.data(), pulse_start_time.data(),
                               num_pulses, 10.0f),
            nullptr);

  std::vector<double> bad_start = pulse_start_time;
  bad_start.back() = std::numeric_limits<double>::infinity();
  EXPECT_EQ(Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                               freq_offset.data(), bad_start.data(),
                               num_pulses, 10.0f),
            nullptr);
}

/*********************************************
 *
 *  Channels
 *
 *********************************************/

TEST_F(TransmitterTest, AddChannelIncrementsCount) {
  TransmitterPtr tx = Create();
  ASSERT_NE(tx, nullptr);

  EXPECT_EQ(AddChannel(tx.get()), RADARSIM_SUCCESS);
  EXPECT_EQ(Get_Num_Txchannel(tx.get()), 1);
}

TEST_F(TransmitterTest, AddChannelWithoutModulationSucceeds) {
  TransmitterPtr tx = Create();
  ASSERT_NE(tx, nullptr);

  // mod_length == 0 means "no modulation"; the modulation arrays are then
  // never dereferenced and may legitimately be null.
  EXPECT_EQ(Add_Txchannel(location, polarization.real, polarization.imag,
                          antenna.phi, antenna.phi_pattern,
                          IsotropicAntenna::kLength, antenna.theta,
                          antenna.theta_pattern, IsotropicAntenna::kLength,
                          IsotropicAntenna::kGain, nullptr, nullptr, nullptr,
                          0, pulse_mod_real.data(), pulse_mod_imag.data(),
                          0.0f, 0.1f, tx.get()),
            RADARSIM_SUCCESS);
  EXPECT_EQ(Get_Num_Txchannel(tx.get()), 1);
}

TEST_F(TransmitterTest, AddChannelRejectsNullTransmitter) {
  EXPECT_EQ(AddChannel(nullptr), RADARSIM_ERROR_NULL_POINTER);
}

/**
 * @brief Get_Num_Txchannel should report 0 for a null handle
 *
 * @details Disabled: the accessor dereferences its argument unconditionally,
 * unlike the sibling accessors Get_BB_Size and Get_Num_Targets which both
 * return 0 for null. Enable once the guard is added.
 */
TEST_F(TransmitterTest, DISABLED_GetNumChannelsIsNullSafe) {
  EXPECT_EQ(Get_Num_Txchannel(nullptr), 0);
}

/**
 * @brief Unlicensed builds accept exactly one transmitter channel
 */
TEST_F(TransmitterTest, FreeTierChannelLimit) {
  RS_SKIP_IF_LICENSED();

  TransmitterPtr tx = Create();
  ASSERT_NE(tx, nullptr);

  for (int i = 0; i < rstest::kFreeTierMaxTxChannels; i++) {
    EXPECT_EQ(AddChannel(tx.get()), RADARSIM_SUCCESS) << "channel " << i;
  }

  EXPECT_EQ(AddChannel(tx.get()), RADARSIM_ERROR_FREE_TIER_LIMIT);
  EXPECT_EQ(Get_Num_Txchannel(tx.get()), rstest::kFreeTierMaxTxChannels)
      << "A rejected channel must not be registered";
}

/*********************************************
 *
 *  Destruction
 *
 *********************************************/

TEST_F(TransmitterTest, FreeIsNullSafeAndIndependent) {
  Free_Transmitter(nullptr);  // must not crash

  t_Transmitter* first = Create().release();
  t_Transmitter* second = Create(15.0f).release();
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  Free_Transmitter(first);

  // Freeing one transmitter must leave the other usable.
  EXPECT_EQ(AddChannel(second), RADARSIM_SUCCESS);
  EXPECT_EQ(Get_Num_Txchannel(second), 1);
  Free_Transmitter(second);
}

/*********************************************
 *
 *  SSB phase noise variant
 *
 *********************************************/

/**
 * @brief Test fixture adding phase noise parameters on top of the waveform
 */
class TransmitterPhaseNoiseTest : public TransmitterTest {
 protected:
  TransmitterPtr CreatePn(int pn_size = 4, double pn_fs = 2e6,
                          int pn_num_samples = 1000,
                          unsigned long long seed = 42) {
    return TransmitterPtr(Create_Transmitter_SSBPhaseNoise(
        freq.data(), freq_time.data(), num_samples, freq_offset.data(),
        pulse_start_time.data(), num_pulses, 10.0f, pn_freq.data(),
        pn_power.data(), pn_size, pn_fs, pn_num_samples, seed, false));
  }

  std::vector<double> pn_freq = {1e3, 10e3, 100e3, 1e6};
  std::vector<double> pn_power = {-80.0, -90.0, -100.0, -110.0};
};

TEST_F(TransmitterPhaseNoiseTest, CreateWithValidParameters) {
  TransmitterPtr tx = CreatePn();

  ASSERT_NE(tx, nullptr);
  EXPECT_EQ(Get_Num_Txchannel(tx.get()), 0);
}

TEST_F(TransmitterPhaseNoiseTest, CreateRejectsNullArrays) {
  EXPECT_EQ(Create_Transmitter_SSBPhaseNoise(
                nullptr, freq_time.data(), num_samples, freq_offset.data(),
                pulse_start_time.data(), num_pulses, 10.0f, pn_freq.data(),
                pn_power.data(), 2, 2e6, 1000, 0, false),
            nullptr);
  EXPECT_EQ(Create_Transmitter_SSBPhaseNoise(
                freq.data(), freq_time.data(), num_samples, freq_offset.data(),
                pulse_start_time.data(), num_pulses, 10.0f, nullptr,
                pn_power.data(), 2, 2e6, 1000, 0, false),
            nullptr);
  EXPECT_EQ(Create_Transmitter_SSBPhaseNoise(
                freq.data(), freq_time.data(), num_samples, freq_offset.data(),
                pulse_start_time.data(), num_pulses, 10.0f, pn_freq.data(),
                nullptr, 2, 2e6, 1000, 0, false),
            nullptr);
}

TEST_F(TransmitterPhaseNoiseTest, CreateRejectsInvalidPhaseNoiseParameters) {
  EXPECT_EQ(CreatePn(/*pn_size=*/0), nullptr);
  EXPECT_EQ(CreatePn(/*pn_size=*/-1), nullptr);
  EXPECT_EQ(CreatePn(4, /*pn_fs=*/0.0), nullptr);
  EXPECT_EQ(CreatePn(4, /*pn_fs=*/-1.0), nullptr);
  EXPECT_EQ(CreatePn(4, 2e6, /*pn_num_samples=*/0), nullptr);
  EXPECT_EQ(CreatePn(4, 2e6, /*pn_num_samples=*/-5), nullptr);
}

TEST_F(TransmitterPhaseNoiseTest, AcceptsChannels) {
  TransmitterPtr tx = CreatePn();
  ASSERT_NE(tx, nullptr);

  EXPECT_EQ(AddChannel(tx.get()), RADARSIM_SUCCESS);
  EXPECT_EQ(Get_Num_Txchannel(tx.get()), 1);
}

}  // namespace
