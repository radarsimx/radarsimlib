/*
 * @file test_simulator.cpp
 * @brief Unit tests for the radar, interference and noise simulators
 *
 * @details
 * Test scenarios:
 * - Run_RadarSimulator with point and mesh targets, including output checks
 * - Run_InterferenceSimulator victim/interferer pairing
 * - Run_NoiseSimulator parameter validation, determinism and scaling
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
#include "test_helpers.hpp"

using rstest::RadarScenario;
using rstest::TargetsPtr;

namespace {

/** @brief True when any element of the buffer is non-zero */
bool HasNonZero(const std::vector<double>& values) {
  for (double v : values) {
    if (v != 0.0) return true;
  }
  return false;
}

/** @brief True when every element is finite */
bool AllFinite(const std::vector<double>& values) {
  for (double v : values) {
    if (!std::isfinite(v)) return false;
  }
  return true;
}

/**
 * @brief Fixture providing a single-frame, single-channel radar
 */
class SimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override { ASSERT_TRUE(scenario_.Build()); }

  RadarScenario scenario_;
};

/*********************************************
 *
 *  Radar simulator
 *
 *********************************************/

/**
 * @brief The reported baseband size matches the radar's own dimensions
 */
TEST_F(SimulatorTest, BasebandSizeMatchesRadarDimensions) {
  const int expected = scenario_.VirtualChannelSize() * scenario_.num_pulses *
                       scenario_.SampleSize();

  ASSERT_GT(expected, 0);
  EXPECT_EQ(scenario_.BasebandSize(), expected);
}

/**
 * @brief A point target produces a finite, non-zero baseband response
 */
TEST_F(SimulatorTest, PointTargetProducesResponse) {
  TargetsPtr targets = rstest::MakeTargets();
  ASSERT_NE(targets, nullptr);

  float location[3] = {100.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_EQ(Add_Point_Target(location, speed, 10.0f, 0.0f, targets.get()),
            RADARSIM_SUCCESS);

  const int bb_size = scenario_.BasebandSize();
  ASSERT_GT(bb_size, 0);
  std::vector<double> bb_real(bb_size, 0.0);
  std::vector<double> bb_imag(bb_size, 0.0);

  int ray_filter[2] = {0, 0};
  EXPECT_EQ(Run_RadarSimulator(scenario_.radar(), targets.get(), 0, 1.0f,
                               ray_filter, bb_real.data(), bb_imag.data()),
            RADARSIM_SUCCESS);

  EXPECT_TRUE(AllFinite(bb_real));
  EXPECT_TRUE(AllFinite(bb_imag));
  EXPECT_TRUE(HasNonZero(bb_real) || HasNonZero(bb_imag))
      << "A target at 100 m must leave a signature in the baseband";
}

/**
 * @brief Repeating a point-target run reproduces the same baseband
 */
TEST_F(SimulatorTest, PointTargetRunIsRepeatable) {
  TargetsPtr targets = rstest::MakeTargets();
  ASSERT_NE(targets, nullptr);

  float location[3] = {50.0f, 0.0f, 0.0f};
  float speed[3] = {-10.0f, 0.0f, 0.0f};
  ASSERT_EQ(Add_Point_Target(location, speed, 5.0f, 0.0f, targets.get()),
            RADARSIM_SUCCESS);

  const int bb_size = scenario_.BasebandSize();
  ASSERT_GT(bb_size, 0);
  std::vector<double> first_real(bb_size, 0.0), first_imag(bb_size, 0.0);
  std::vector<double> second_real(bb_size, 0.0), second_imag(bb_size, 0.0);

  int ray_filter[2] = {0, 0};
  ASSERT_EQ(Run_RadarSimulator(scenario_.radar(), targets.get(), 0, 1.0f,
                               ray_filter, first_real.data(),
                               first_imag.data()),
            RADARSIM_SUCCESS);
  ASSERT_EQ(Run_RadarSimulator(scenario_.radar(), targets.get(), 0, 1.0f,
                               ray_filter, second_real.data(),
                               second_imag.data()),
            RADARSIM_SUCCESS);

  for (int i = 0; i < bb_size; i++) {
    ASSERT_DOUBLE_EQ(first_real[i], second_real[i]) << "sample " << i;
    ASSERT_DOUBLE_EQ(first_imag[i], second_imag[i]) << "sample " << i;
  }
}

/**
 * @brief A mesh target exercises the ray-tracing path end to end
 */
TEST_F(SimulatorTest, MeshTargetRunSucceeds) {
  TargetsPtr targets = rstest::MakeTargets();
  ASSERT_NE(targets, nullptr);

  rstest::PlateMesh plate(0.5f);
  rstest::Motion motion;
  motion.location[0] = 20.0f;
  ASSERT_EQ(rstest::AddMesh(plate, motion, targets.get()), RADARSIM_SUCCESS);

  const int bb_size = scenario_.BasebandSize();
  ASSERT_GT(bb_size, 0);
  std::vector<double> bb_real(bb_size, 0.0);
  std::vector<double> bb_imag(bb_size, 0.0);

  int ray_filter[2] = {0, 10};
  EXPECT_EQ(Run_RadarSimulator(scenario_.radar(), targets.get(), 0, 0.5f,
                               ray_filter, bb_real.data(), bb_imag.data()),
            RADARSIM_SUCCESS);

  EXPECT_TRUE(AllFinite(bb_real));
  EXPECT_TRUE(AllFinite(bb_imag));
}

/**
 * @brief An empty target list is a valid, side-effect-free simulation
 */
TEST_F(SimulatorTest, EmptyTargetListLeavesBasebandZero) {
  TargetsPtr targets = rstest::MakeTargets();
  ASSERT_NE(targets, nullptr);

  const int bb_size = scenario_.BasebandSize();
  ASSERT_GT(bb_size, 0);
  std::vector<double> bb_real(bb_size, 7.0);  // poison values
  std::vector<double> bb_imag(bb_size, 7.0);

  int ray_filter[2] = {0, 0};
  EXPECT_EQ(Run_RadarSimulator(scenario_.radar(), targets.get(), 0, 1.0f,
                               ray_filter, bb_real.data(), bb_imag.data()),
            RADARSIM_SUCCESS);

  for (int i = 0; i < bb_size; i++) {
    EXPECT_DOUBLE_EQ(bb_real[i], 0.0) << "sample " << i;
    EXPECT_DOUBLE_EQ(bb_imag[i], 0.0) << "sample " << i;
  }
}

/*********************************************
 *
 *  Range gate
 *
 *********************************************/

/**
 * @brief Fixture for gated vs un-gated comparisons of the same scenario
 *
 * @details The scenario's chirp sweeps 3 MHz per microsecond, i.e. a slope of
 * 3e12 Hz/s, and the receiver samples at 1 MHz. A target at kGateRange has a
 * round-trip delay of exactly kGate, so:
 * - gated: the deramp reference is delayed by the same kGate, the residual
 *   beat is zero and every sample carries the same phase
 * - un-gated: the reference sits at zero delay and the beat is
 *   3e12 * 8e-7 = 2.4 MHz, which folds to 400 kHz at a 1 MHz sampling rate
 *   and rotates the phase from sample to sample
 */
class RangeGateTest : public ::testing::Test {
 protected:
  /** @brief Round-trip delay of a target at kGateRange (s) */
  static constexpr double kGate = 8.0e-7;

  /** @brief Range whose round trip is exactly kGate (m) */
  static constexpr float kGateRange = static_cast<float>(299792458.0 * kGate /
                                                         2.0);

  /** @brief Run one stationary point target at range through a scenario */
  static void RunPointTarget(RadarScenario& scenario, float range,
                             std::vector<double>& bb_real,
                             std::vector<double>& bb_imag) {
    TargetsPtr targets = rstest::MakeTargets();
    ASSERT_NE(targets, nullptr);

    float location[3] = {range, 0.0f, 0.0f};
    float speed[3] = {0.0f, 0.0f, 0.0f};
    ASSERT_EQ(Add_Point_Target(location, speed, 10.0f, 0.0f, targets.get()),
              RADARSIM_SUCCESS);

    const int bb_size = scenario.BasebandSize();
    ASSERT_GT(bb_size, 0);
    bb_real.assign(bb_size, 0.0);
    bb_imag.assign(bb_size, 0.0);

    int ray_filter[2] = {0, 0};
    ASSERT_EQ(Run_RadarSimulator(scenario.radar(), targets.get(), 0, 1.0f,
                                 ray_filter, bb_real.data(), bb_imag.data()),
              RADARSIM_SUCCESS);
  }

  /**
   * @brief Ratio of the coherent sum to the incoherent sum of the samples
   *
   * @details 1 when every sample shares a phase (a DC beat), and well below 1
   * once the beat rotates the phasor across the buffer.
   */
  static double PhaseCoherence(const std::vector<double>& bb_real,
                               const std::vector<double>& bb_imag) {
    double sum_real = 0.0, sum_imag = 0.0, sum_magnitude = 0.0;
    for (size_t i = 0; i < bb_real.size(); i++) {
      sum_real += bb_real[i];
      sum_imag += bb_imag[i];
      sum_magnitude += std::hypot(bb_real[i], bb_imag[i]);
    }
    if (sum_magnitude == 0.0) return 0.0;
    return std::hypot(sum_real, sum_imag) / sum_magnitude;
  }
};

/**
 * @brief A target sitting on the gate deramps to DC
 *
 * @details This is the point of the gate: without it the beat for this target
 * is far outside the receiver's Nyquist band.
 */
TEST_F(RangeGateTest, TargetOnGateBeatsAtDC) {
  RadarScenario gated;
  gated.gate_delay = kGate;
  ASSERT_TRUE(gated.Build());

  std::vector<double> bb_real, bb_imag;
  RunPointTarget(gated, kGateRange, bb_real, bb_imag);

  ASSERT_TRUE(AllFinite(bb_real));
  ASSERT_TRUE(AllFinite(bb_imag));
  ASSERT_TRUE(HasNonZero(bb_real) || HasNonZero(bb_imag));

  EXPECT_GT(PhaseCoherence(bb_real, bb_imag), 0.99)
      << "A target at the gate range must produce a constant-phase beat";
}

/**
 * @brief The same target without a gate beats above Nyquist and folds
 */
TEST_F(RangeGateTest, TargetOnGateRangeIsAliasedWithoutGate) {
  RadarScenario ungated;
  ASSERT_TRUE(ungated.Build());

  std::vector<double> bb_real, bb_imag;
  RunPointTarget(ungated, kGateRange, bb_real, bb_imag);

  ASSERT_TRUE(HasNonZero(bb_real) || HasNonZero(bb_imag));

  EXPECT_LT(PhaseCoherence(bb_real, bb_imag), 0.5)
      << "Without a gate the beat rotates across the buffer";
}

/**
 * @brief A gate of zero reproduces the un-gated baseband bit for bit
 */
TEST_F(RangeGateTest, ZeroGateMatchesUngated) {
  RadarScenario ungated;
  ASSERT_TRUE(ungated.Build());

  RadarScenario zero_gate;
  zero_gate.gate_delay = 0.0;
  ASSERT_TRUE(zero_gate.Build());

  std::vector<double> ungated_real, ungated_imag;
  std::vector<double> zero_real, zero_imag;
  RunPointTarget(ungated, 50.0f, ungated_real, ungated_imag);
  RunPointTarget(zero_gate, 50.0f, zero_real, zero_imag);

  ASSERT_EQ(ungated_real.size(), zero_real.size());
  for (size_t i = 0; i < ungated_real.size(); i++) {
    ASSERT_DOUBLE_EQ(ungated_real[i], zero_real[i]) << "sample " << i;
    ASSERT_DOUBLE_EQ(ungated_imag[i], zero_imag[i]) << "sample " << i;
  }
}

/*********************************************
 *
 *  Interference simulator
 *
 *********************************************/

/**
 * @brief A separated victim/interferer pair yields a finite, non-zero buffer
 *
 * @note The interferer must not be co-located with the victim. A zero
 * separation puts a 1/R term at R = 0 and the coupled signal comes out
 * non-finite; see InterferenceWithZeroSeparation below.
 */
TEST_F(SimulatorTest, InterferenceRunSucceeds) {
  RadarScenario interferer;
  ASSERT_TRUE(interferer.BuildTxRx());

  // 20 m down the victim's boresight, yawed 180 degrees to face back at it.
  // The 67 ns propagation delay puts the beat note near 200 kHz, inside the
  // victim's 500 kHz baseband bandwidth.
  float location[3] = {20.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {rstest::kPiF, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(interferer.AttachRadar(location, speed, rotation,
                                     rotation_rate));

  const int bb_size = scenario_.BasebandSize();
  ASSERT_GT(bb_size, 0);
  std::vector<double> interf_real(bb_size, 0.0);
  std::vector<double> interf_imag(bb_size, 0.0);

  EXPECT_EQ(Run_InterferenceSimulator(scenario_.radar(), interferer.radar(),
                                      interf_real.data(), interf_imag.data()),
            RADARSIM_SUCCESS);

  EXPECT_TRUE(AllFinite(interf_real));
  EXPECT_TRUE(AllFinite(interf_imag));
  EXPECT_TRUE(HasNonZero(interf_real) || HasNonZero(interf_imag))
      << "An interferer 50 m away must couple into the victim's baseband";
}

/**
 * @brief A co-located interferer is a degenerate but non-crashing run
 *
 * @details Zero separation is unphysical - the coupled amplitude is undefined,
 * so only the status code is asserted here. This pins down that the library
 * reports success rather than crashing or hanging on the degenerate geometry.
 */
TEST_F(SimulatorTest, InterferenceWithZeroSeparation) {
  const int bb_size = scenario_.BasebandSize();
  ASSERT_GT(bb_size, 0);
  std::vector<double> interf_real(bb_size, 0.0);
  std::vector<double> interf_imag(bb_size, 0.0);

  EXPECT_EQ(Run_InterferenceSimulator(scenario_.radar(), scenario_.radar(),
                                      interf_real.data(), interf_imag.data()),
            RADARSIM_SUCCESS);
}

/*********************************************
 *
 *  Known gaps
 *
 *********************************************/

/**
 * @brief Run_RadarSimulator should reject null handles
 *
 * @details Disabled: unlike every sibling entry point, Run_RadarSimulator
 * dereferences ptr_radar_c and ptr_targets_c without a null check, so this
 * currently crashes the runner instead of failing. Enable once the null guards
 * documented in the header ("non-zero RadarSimErrorCode on failure") exist.
 */
TEST_F(SimulatorTest, DISABLED_RadarSimulatorRejectsNullArguments) {
  const int bb_size = scenario_.BasebandSize();
  std::vector<double> bb_real(bb_size, 0.0);
  std::vector<double> bb_imag(bb_size, 0.0);
  int ray_filter[2] = {0, 0};

  TargetsPtr targets = rstest::MakeTargets();
  ASSERT_NE(targets, nullptr);

  EXPECT_EQ(Run_RadarSimulator(nullptr, targets.get(), 0, 1.0f, ray_filter,
                               bb_real.data(), bb_imag.data()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_RadarSimulator(scenario_.radar(), nullptr, 0, 1.0f, ray_filter,
                               bb_real.data(), bb_imag.data()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_RadarSimulator(scenario_.radar(), targets.get(), 0, 1.0f,
                               ray_filter, nullptr, bb_imag.data()),
            RADARSIM_ERROR_NULL_POINTER);
}

/**
 * @brief Run_InterferenceSimulator should reject null handles
 *
 * @details Disabled for the same reason as the test above - the function
 * dereferences both radar handles unconditionally.
 */
TEST_F(SimulatorTest, DISABLED_InterferenceSimulatorRejectsNullArguments) {
  const int bb_size = scenario_.BasebandSize();
  std::vector<double> interf_real(bb_size, 0.0);
  std::vector<double> interf_imag(bb_size, 0.0);

  EXPECT_EQ(Run_InterferenceSimulator(nullptr, scenario_.radar(),
                                      interf_real.data(), interf_imag.data()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_InterferenceSimulator(scenario_.radar(), nullptr,
                                      interf_real.data(), interf_imag.data()),
            RADARSIM_ERROR_NULL_POINTER);
}

/*********************************************
 *
 *  Noise simulator
 *
 *********************************************/

/**
 * @brief Fixture pre-computing the noise simulator's buffer geometry
 */
class NoiseSimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(scenario_.Build());
    sample_size_ = scenario_.SampleSize();
    channel_size_ = scenario_.VirtualChannelSize();
    pulse_size_ = scenario_.num_pulses;
    ASSERT_GT(sample_size_, 0);
    ASSERT_GT(channel_size_, 0);

    timestamps_ = scenario_.MakeTimestamps();
    out_size_ = channel_size_ * pulse_size_ * sample_size_;
  }

  /** @brief Run the noise simulator into the given buffers */
  int Run(double level, bool is_complex, std::vector<double>& real,
          std::vector<double>& imag, unsigned long long seed) {
    real.assign(out_size_, 0.0);
    imag.assign(out_size_, 0.0);
    return Run_NoiseSimulator(scenario_.radar(), level, is_complex,
                              timestamps_.data(), channel_size_, pulse_size_,
                              sample_size_, real.data(), imag.data(), seed);
  }

  RadarScenario scenario_;
  std::vector<double> timestamps_;
  int sample_size_ = 0, channel_size_ = 0, pulse_size_ = 0, out_size_ = 0;
};

TEST_F(NoiseSimulatorTest, RealNoiseRunSucceeds) {
  std::vector<double> real, imag;
  EXPECT_EQ(Run(1.0, false, real, imag, 42), RADARSIM_SUCCESS);

  EXPECT_TRUE(AllFinite(real));
  EXPECT_TRUE(HasNonZero(real));
}

TEST_F(NoiseSimulatorTest, ComplexNoisePopulatesBothComponents) {
  std::vector<double> real, imag;
  EXPECT_EQ(Run(1.0, true, real, imag, 42), RADARSIM_SUCCESS);

  EXPECT_TRUE(AllFinite(real));
  EXPECT_TRUE(AllFinite(imag));
  EXPECT_TRUE(HasNonZero(real));
  EXPECT_TRUE(HasNonZero(imag));
}

TEST_F(NoiseSimulatorTest, ZeroLevelProducesSilence) {
  std::vector<double> real, imag;
  ASSERT_EQ(Run(0.0, true, real, imag, 42), RADARSIM_SUCCESS);

  for (int i = 0; i < out_size_; i++) {
    EXPECT_DOUBLE_EQ(real[i], 0.0) << "sample " << i;
    EXPECT_DOUBLE_EQ(imag[i], 0.0) << "sample " << i;
  }
}

TEST_F(NoiseSimulatorTest, SameSeedProducesSameNoise) {
  std::vector<double> first_real, first_imag, second_real, second_imag;
  ASSERT_EQ(Run(1.0, true, first_real, first_imag, 12345), RADARSIM_SUCCESS);
  ASSERT_EQ(Run(1.0, true, second_real, second_imag, 12345), RADARSIM_SUCCESS);

  for (int i = 0; i < out_size_; i++) {
    ASSERT_DOUBLE_EQ(first_real[i], second_real[i]) << "sample " << i;
    ASSERT_DOUBLE_EQ(first_imag[i], second_imag[i]) << "sample " << i;
  }
}

TEST_F(NoiseSimulatorTest, DifferentSeedsProduceDifferentNoise) {
  std::vector<double> first_real, first_imag, second_real, second_imag;
  ASSERT_EQ(Run(1.0, true, first_real, first_imag, 1), RADARSIM_SUCCESS);
  ASSERT_EQ(Run(1.0, true, second_real, second_imag, 2), RADARSIM_SUCCESS);

  bool differs = false;
  for (int i = 0; i < out_size_ && !differs; i++) {
    differs = (first_real[i] != second_real[i]);
  }
  EXPECT_TRUE(differs) << "Distinct seeds produced identical noise";
}

TEST_F(NoiseSimulatorTest, RejectsNullRadar) {
  std::vector<double> real(out_size_, 0.0), imag(out_size_, 0.0);

  EXPECT_EQ(Run_NoiseSimulator(nullptr, 1.0, false, timestamps_.data(),
                               channel_size_, pulse_size_, sample_size_,
                               real.data(), imag.data(), 0),
            RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(NoiseSimulatorTest, RejectsNullOutputBuffers) {
  std::vector<double> buffer(out_size_, 0.0);

  EXPECT_EQ(Run_NoiseSimulator(scenario_.radar(), 1.0, false,
                               timestamps_.data(), channel_size_, pulse_size_,
                               sample_size_, nullptr, buffer.data(), 0),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_NoiseSimulator(scenario_.radar(), 1.0, false,
                               timestamps_.data(), channel_size_, pulse_size_,
                               sample_size_, buffer.data(), nullptr, 0),
            RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(NoiseSimulatorTest, RejectsNegativeNoiseLevel) {
  std::vector<double> real(out_size_, 0.0), imag(out_size_, 0.0);

  EXPECT_EQ(Run_NoiseSimulator(scenario_.radar(), -1.0, false,
                               timestamps_.data(), channel_size_, pulse_size_,
                               sample_size_, real.data(), imag.data(), 0),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

}  // namespace
