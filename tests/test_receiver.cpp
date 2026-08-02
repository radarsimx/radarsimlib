/*
 * @file test_receiver.cpp
 * @brief Unit tests for Receiver C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Receiver creation with valid and out-of-range RF parameters
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

#include <limits>
#include <vector>

#include "radarsim.h"
#include "test_helpers.hpp"

using rstest::IsotropicAntenna;
using rstest::LinearPolarization;
using rstest::ReceiverPtr;

namespace {

/**
 * @brief Test fixture owning receiver and channel parameters
 */
class ReceiverTest : public ::testing::Test {
 protected:
  /** @brief Create a receiver owned by the caller's smart pointer */
  ReceiverPtr Create(float sampling_rate = 1e6f, double gate = 0.0) {
    return ReceiverPtr(Create_Receiver(sampling_rate, rf_gain, resistor,
                                       baseband_gain, baseband_bw, gate));
  }

  /** @brief Add the fixture's default channel to the given receiver */
  int AddChannel(t_Receiver* rx) {
    return Add_Rxchannel(location, polarization.real, polarization.imag,
                         antenna.phi, antenna.phi_pattern,
                         IsotropicAntenna::kLength, antenna.theta,
                         antenna.theta_pattern, IsotropicAntenna::kLength,
                         IsotropicAntenna::kGain, rx);
  }

  float fs = 1e6f;             ///< 1 MHz sampling rate
  float rf_gain = 30.0f;       ///< 30 dB RF gain
  float resistor = 50.0f;      ///< 50 Ohm load
  float baseband_gain = 20.0f; ///< 20 dB baseband gain
  float baseband_bw = 500e3f;  ///< 500 kHz baseband bandwidth
  double gate_delay = 0.0;     ///< No range gate (zero-delay deramp)

  IsotropicAntenna antenna;
  LinearPolarization polarization;
  float location[3] = {0.0f, 0.0f, 0.0f};
};

/*********************************************
 *
 *  Creation
 *
 *********************************************/

TEST_F(ReceiverTest, CreateWithValidParameters) {
  ReceiverPtr rx = Create();

  ASSERT_NE(rx, nullptr);
  EXPECT_EQ(Get_Num_Rxchannel(rx.get()), 0)
      << "A freshly created receiver must have no channels";
}

TEST_F(ReceiverTest, CreateRejectsNonPositiveSamplingRate) {
  EXPECT_EQ(Create_Receiver(0.0f, rf_gain, resistor, baseband_gain, baseband_bw,
                            gate_delay),
            nullptr);
  EXPECT_EQ(Create_Receiver(-1.0f, rf_gain, resistor, baseband_gain,
                            baseband_bw, gate_delay),
            nullptr);
}

TEST_F(ReceiverTest, CreateRejectsNonPositiveResistor) {
  EXPECT_EQ(Create_Receiver(fs, rf_gain, 0.0f, baseband_gain, baseband_bw,
                            gate_delay),
            nullptr);
  EXPECT_EQ(Create_Receiver(fs, rf_gain, -1.0f, baseband_gain, baseband_bw,
                            gate_delay),
            nullptr);
}

TEST_F(ReceiverTest, CreateRejectsNonPositiveBandwidth) {
  EXPECT_EQ(
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, 0.0f, gate_delay),
      nullptr);
  EXPECT_EQ(
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, -1.0f, gate_delay),
      nullptr);
}

/**
 * @brief Gains are unconstrained - negative gain is a valid attenuator
 */
TEST_F(ReceiverTest, CreateAcceptsNegativeGains) {
  ReceiverPtr rx(
      Create_Receiver(fs, -10.0f, resistor, -6.0f, baseband_bw, gate_delay));
  EXPECT_NE(rx, nullptr);
}

/*********************************************
 *
 *  Range gate
 *
 *********************************************/

/**
 * @brief A positive gate delay is a valid configuration
 *
 * @details The gate opens the receive window this far after the chirp start
 * and delays the deramp reference by the same amount, so a target at
 * c * gate / 2 beats at DC. 741 us corresponds to ~111 km.
 */
TEST_F(ReceiverTest, CreateAcceptsPositiveGateDelay) {
  ReceiverPtr rx = Create(fs, 741e-6);

  ASSERT_NE(rx, nullptr);
  EXPECT_EQ(Get_Num_Rxchannel(rx.get()), 0);
}

/**
 * @brief A zero gate delay is the documented default (zero-delay deramp)
 */
TEST_F(ReceiverTest, CreateAcceptsZeroGateDelay) {
  EXPECT_NE(Create(fs, 0.0), nullptr);
}

/**
 * @brief A negative gate would place the deramp reference before the chirp
 */
TEST_F(ReceiverTest, CreateRejectsNegativeGateDelay) {
  EXPECT_EQ(
      Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw, -1e-6),
      nullptr);
}

/**
 * @brief Non-finite gate delays are rejected rather than propagated
 *
 * @details The gate is added to every sample instant, so a NaN or infinite
 * value would silently poison the whole baseband rather than fail loudly.
 */
TEST_F(ReceiverTest, CreateRejectsNonFiniteGateDelay) {
  EXPECT_EQ(Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw,
                            std::numeric_limits<double>::quiet_NaN()),
            nullptr);
  EXPECT_EQ(Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw,
                            std::numeric_limits<double>::infinity()),
            nullptr);
  EXPECT_EQ(Create_Receiver(fs, rf_gain, resistor, baseband_gain, baseband_bw,
                            -std::numeric_limits<double>::infinity()),
            nullptr);
}

/*********************************************
 *
 *  Channels
 *
 *********************************************/

TEST_F(ReceiverTest, AddChannelIncrementsCount) {
  ReceiverPtr rx = Create();
  ASSERT_NE(rx, nullptr);

  EXPECT_EQ(AddChannel(rx.get()), RADARSIM_SUCCESS);
  EXPECT_EQ(Get_Num_Rxchannel(rx.get()), 1);
}

TEST_F(ReceiverTest, AddChannelRejectsNullReceiver) {
  EXPECT_EQ(AddChannel(nullptr), RADARSIM_ERROR_NULL_POINTER);
}

/**
 * @brief Get_Num_Rxchannel should report 0 for a null handle
 *
 * @details Disabled: the accessor dereferences its argument unconditionally,
 * unlike the sibling accessors Get_BB_Size and Get_Num_Targets which both
 * return 0 for null. Enable once the guard is added.
 */
TEST_F(ReceiverTest, DISABLED_GetNumChannelsIsNullSafe) {
  EXPECT_EQ(Get_Num_Rxchannel(nullptr), 0);
}

/**
 * @brief Unlicensed builds accept exactly one receiver channel
 */
TEST_F(ReceiverTest, FreeTierChannelLimit) {
  RS_SKIP_IF_LICENSED();

  ReceiverPtr rx = Create();
  ASSERT_NE(rx, nullptr);

  for (int i = 0; i < rstest::kFreeTierMaxRxChannels; i++) {
    EXPECT_EQ(AddChannel(rx.get()), RADARSIM_SUCCESS) << "channel " << i;
  }

  EXPECT_EQ(AddChannel(rx.get()), RADARSIM_ERROR_FREE_TIER_LIMIT);
  EXPECT_EQ(Get_Num_Rxchannel(rx.get()), rstest::kFreeTierMaxRxChannels)
      << "A rejected channel must not be registered";
}

/*********************************************
 *
 *  Destruction
 *
 *********************************************/

TEST_F(ReceiverTest, FreeIsNullSafeAndIndependent) {
  Free_Receiver(nullptr);  // must not crash

  t_Receiver* first = Create().release();
  t_Receiver* second = Create(2e6f).release();
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  Free_Receiver(first);

  // Freeing one receiver must leave the other usable.
  EXPECT_EQ(AddChannel(second), RADARSIM_SUCCESS);
  EXPECT_EQ(Get_Num_Rxchannel(second), 1);
  Free_Receiver(second);
}

}  // namespace
