/*
 * @file test_rcs.cpp
 * @brief Unit tests for the Physical Optics RCS simulator
 *
 * @details
 * Test scenarios:
 * - Run_RcsSimulator parameter validation with exact error codes
 * - Monostatic RCS of a flat plate: finite, positive and area-monotonic
 * - Multi-direction batches fill every output slot
 * - Repeatability of a deterministic Physical Optics run
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

using rstest::PlateMesh;
using rstest::TargetsPtr;

namespace {

/**
 * @brief Broadside monostatic RCS of a square plate 10 m down the +X axis
 *
 * @param half_size Half the plate edge length (m)
 * @return double RCS in m^2, or -1.0 if the run failed
 */
double BroadsidePlateRcs(float half_size) {
  TargetsPtr targets = rstest::MakeTargets();
  if (!targets) return -1.0;

  PlateMesh plate(half_size);
  rstest::Motion motion;
  motion.location[0] = 10.0f;
  if (rstest::AddMesh(plate, motion, targets.get()) != RADARSIM_SUCCESS) {
    return -1.0;
  }

  double inc_dir[3] = {-1.0, 0.0, 0.0};
  double obs_dir[3] = {-1.0, 0.0, 0.0};
  double polar_real[3] = {0.0, 0.0, 1.0};
  double polar_imag[3] = {0.0, 0.0, 0.0};
  double rcs = 0.0;

  if (Run_RcsSimulator(targets.get(), inc_dir, obs_dir, 1, polar_real,
                       polar_imag, polar_real, polar_imag, 24e9, 1.0,
                       &rcs) != RADARSIM_SUCCESS) {
    return -1.0;
  }
  return rcs;
}

/**
 * @brief Fixture holding a monostatic PO configuration along the -X axis
 */
class RcsSimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    targets_ = rstest::MakeTargets();
    ASSERT_NE(targets_, nullptr);
  }

  /** @brief Add a square plate of the given half size, facing the sensor */
  void AddPlate(float half_size) {
    plates_.emplace_back(half_size);
    rstest::Motion motion;
    motion.location[0] = 10.0f;
    ASSERT_EQ(rstest::AddMesh(plates_.back(), motion, targets_.get()),
              RADARSIM_SUCCESS);
  }

  /** @brief Run a monostatic sweep and return the RCS values */
  int RunMonostatic(int num_directions, std::vector<double>& rcs_out,
                    double frequency = 24e9, double density = 1.0) {
    std::vector<double> inc_dir(num_directions * 3, 0.0);
    std::vector<double> obs_dir(num_directions * 3, 0.0);
    for (int i = 0; i < num_directions; i++) {
      inc_dir[i * 3] = -1.0;  // travelling toward -X, i.e. onto the plate
      obs_dir[i * 3] = -1.0;
    }

    rcs_out.assign(num_directions, 0.0);
    return Run_RcsSimulator(targets_.get(), inc_dir.data(), obs_dir.data(),
                            num_directions, inc_polar_real, inc_polar_imag,
                            obs_polar_real, obs_polar_imag, frequency, density,
                            rcs_out.data());
  }

  TargetsPtr targets_;
  std::vector<PlateMesh> plates_;

  // Vertical (z) linear polarization on both transmit and receive.
  double inc_polar_real[3] = {0.0, 0.0, 1.0};
  double inc_polar_imag[3] = {0.0, 0.0, 0.0};
  double obs_polar_real[3] = {0.0, 0.0, 1.0};
  double obs_polar_imag[3] = {0.0, 0.0, 0.0};
  double inc_dir[3] = {-1.0, 0.0, 0.0};
  double obs_dir[3] = {-1.0, 0.0, 0.0};
  double rcs_result[1] = {0.0};
};

/*********************************************
 *
 *  Validation
 *
 *********************************************/

TEST_F(RcsSimulatorTest, RejectsNullArguments) {
  AddPlate(0.5f);

  EXPECT_EQ(Run_RcsSimulator(nullptr, inc_dir, obs_dir, 1, inc_polar_real,
                             inc_polar_imag, obs_polar_real, obs_polar_imag,
                             24e9, 1.0, rcs_result),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), nullptr, obs_dir, 1,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             obs_polar_imag, 24e9, 1.0, rcs_result),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, nullptr, 1,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             obs_polar_imag, 24e9, 1.0, rcs_result),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 1, nullptr,
                             inc_polar_imag, obs_polar_real, obs_polar_imag,
                             24e9, 1.0, rcs_result),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 1,
                             inc_polar_real, nullptr, obs_polar_real,
                             obs_polar_imag, 24e9, 1.0, rcs_result),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 1,
                             inc_polar_real, inc_polar_imag, nullptr,
                             obs_polar_imag, 24e9, 1.0, rcs_result),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 1,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             nullptr, 24e9, 1.0, rcs_result),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 1,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             obs_polar_imag, 24e9, 1.0, nullptr),
            RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(RcsSimulatorTest, RejectsNonPositiveDirectionCount) {
  AddPlate(0.5f);

  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 0,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             obs_polar_imag, 24e9, 1.0, rcs_result),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, -1,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             obs_polar_imag, 24e9, 1.0, rcs_result),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

TEST_F(RcsSimulatorTest, RejectsNonPositiveFrequency) {
  AddPlate(0.5f);

  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 1,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             obs_polar_imag, 0.0, 1.0, rcs_result),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 1,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             obs_polar_imag, -24e9, 1.0, rcs_result),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

TEST_F(RcsSimulatorTest, RejectsNonPositiveDensity) {
  AddPlate(0.5f);

  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 1,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             obs_polar_imag, 24e9, 0.0, rcs_result),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Run_RcsSimulator(targets_.get(), inc_dir, obs_dir, 1,
                             inc_polar_real, inc_polar_imag, obs_polar_real,
                             obs_polar_imag, 24e9, -1.0, rcs_result),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

/**
 * @brief A target list with no mesh should be a valid, empty scattering problem
 *
 * @details Disabled: an empty target manager currently throws std::length_error
 * ("vector too long") while building the BVH, which Run_RcsSimulator reports as
 * RADARSIM_ERROR_EXCEPTION. Zero targets should instead return
 * RADARSIM_SUCCESS with an all-zero result. Enable once the empty-scene path is
 * handled.
 */
TEST_F(RcsSimulatorTest, DISABLED_EmptyTargetListRunsWithoutTargets) {
  std::vector<double> rcs;
  EXPECT_EQ(RunMonostatic(1, rcs), RADARSIM_SUCCESS);
  EXPECT_TRUE(std::isfinite(rcs[0]));
}

/*********************************************
 *
 *  Physical behaviour
 *
 *********************************************/

/**
 * @brief Broadside monostatic RCS of a plate is finite and positive
 */
TEST_F(RcsSimulatorTest, PlateHasPositiveBroadsideRcs) {
  AddPlate(0.5f);

  std::vector<double> rcs;
  ASSERT_EQ(RunMonostatic(1, rcs), RADARSIM_SUCCESS);

  EXPECT_TRUE(std::isfinite(rcs[0]));
  EXPECT_GT(rcs[0], 0.0);
}

/**
 * @brief A larger plate scatters more energy back at broadside
 *
 * @details Physical Optics gives sigma = 4*pi*A^2/lambda^2 for a plate at
 * broadside, so doubling the edge length must increase the RCS substantially.
 */
TEST(RcsPlateSizeTest, LargerPlateHasLargerRcs) {
  const double small_rcs = BroadsidePlateRcs(0.25f);
  const double large_rcs = BroadsidePlateRcs(0.5f);

  ASSERT_GT(small_rcs, 0.0) << "small plate run failed";
  ASSERT_GT(large_rcs, 0.0) << "large plate run failed";
  EXPECT_GT(large_rcs, small_rcs)
      << "small = " << small_rcs << ", large = " << large_rcs;
}

/**
 * @brief Every requested direction gets its own output slot
 */
TEST_F(RcsSimulatorTest, MultipleDirectionsFillEverySlot) {
  AddPlate(0.5f);

  constexpr int kNumDirections = 5;
  std::vector<double> rcs;
  ASSERT_EQ(RunMonostatic(kNumDirections, rcs), RADARSIM_SUCCESS);

  ASSERT_EQ(static_cast<int>(rcs.size()), kNumDirections);
  for (int i = 0; i < kNumDirections; i++) {
    EXPECT_TRUE(std::isfinite(rcs[i])) << "direction " << i;
    EXPECT_GT(rcs[i], 0.0) << "direction " << i;
    // Identical geometry for every direction, so the answers must agree.
    EXPECT_DOUBLE_EQ(rcs[i], rcs[0]) << "direction " << i;
  }
}

/**
 * @brief Repeating a Physical Optics run reproduces the same RCS
 */
TEST_F(RcsSimulatorTest, RunIsRepeatable) {
  AddPlate(0.5f);

  std::vector<double> first, second;
  ASSERT_EQ(RunMonostatic(1, first), RADARSIM_SUCCESS);
  ASSERT_EQ(RunMonostatic(1, second), RADARSIM_SUCCESS);

  EXPECT_DOUBLE_EQ(first[0], second[0]);
}

}  // namespace
