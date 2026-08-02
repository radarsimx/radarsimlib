/*
 * @file test_lidar.cpp
 * @brief Unit tests for the LiDAR point cloud simulator
 *
 * @details
 * Test scenarios:
 * - Run_LidarSimulator parameter validation with exact error codes
 * - Ray hitting a known plate: point position, distance and intensity
 * - max_points capping and actual_points reporting
 * - Rays that miss every surface produce no points
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

/** @brief Distance from the sensor to the test plate (m) */
constexpr double kPlateRange = 10.0;

/**
 * @brief Fixture with a sensor at the origin and a plate on the +X axis
 *
 * @details The plate spans y,z in [-1, 1] at x = 10 m with its normal facing
 * -X, so a ray along +X (phi = 0, theta = pi/2) hits its centre.
 */
class LidarSimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    targets_ = rstest::MakeTargets();
    ASSERT_NE(targets_, nullptr);
  }

  /** @brief Place the standard plate in front of the sensor */
  void AddPlate() {
    rstest::Motion motion;
    motion.location[0] = static_cast<float>(kPlateRange);
    ASSERT_EQ(rstest::AddMesh(plate_, motion, targets_.get()),
              RADARSIM_SUCCESS);
  }

  /**
   * @brief Shoot the given rays and collect the resulting cloud
   *
   * @param phi Azimuth angles (rad)
   * @param theta Elevation angles (rad)
   * @param max_points Output capacity to advertise
   * @return int RADARSIM_SUCCESS or a RADARSIM_ERROR_* code
   */
  int Shoot(std::vector<double> phi, std::vector<double> theta,
            int max_points) {
    phi_ = std::move(phi);
    theta_ = std::move(theta);
    points_.assign(static_cast<size_t>(max_points) * 3, 0.0);
    distances_.assign(max_points, 0.0);
    intensities_.assign(max_points, 0.0);
    actual_points_ = -1;

    return Run_LidarSimulator(targets_.get(), phi_.data(), theta_.data(),
                              static_cast<int>(phi_.size()), sensor_location_,
                              points_.data(), distances_.data(),
                              intensities_.data(), max_points,
                              &actual_points_);
  }

  TargetsPtr targets_;
  PlateMesh plate_{1.0f};

  double sensor_location_[3] = {0.0, 0.0, 0.0};
  std::vector<double> phi_, theta_;
  std::vector<double> points_, distances_, intensities_;
  int actual_points_ = -1;
};

/*********************************************
 *
 *  Validation
 *
 *********************************************/

TEST_F(LidarSimulatorTest, RejectsNullArguments) {
  AddPlate();

  double phi[1] = {0.0};
  double theta[1] = {rstest::kPi / 2.0};
  double points[3] = {0.0};
  double distances[1] = {0.0};
  double intensities[1] = {0.0};
  int actual = 0;

  EXPECT_EQ(Run_LidarSimulator(nullptr, phi, theta, 1, sensor_location_,
                               points, distances, intensities, 1, &actual),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_LidarSimulator(targets_.get(), nullptr, theta, 1,
                               sensor_location_, points, distances,
                               intensities, 1, &actual),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, nullptr, 1,
                               sensor_location_, points, distances,
                               intensities, 1, &actual),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, theta, 1, nullptr, points,
                               distances, intensities, 1, &actual),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, theta, 1,
                               sensor_location_, nullptr, distances,
                               intensities, 1, &actual),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, theta, 1,
                               sensor_location_, points, nullptr, intensities,
                               1, &actual),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, theta, 1,
                               sensor_location_, points, distances, nullptr, 1,
                               &actual),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, theta, 1,
                               sensor_location_, points, distances,
                               intensities, 1, nullptr),
            RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(LidarSimulatorTest, RejectsNonPositiveRayCount) {
  AddPlate();

  double phi[1] = {0.0};
  double theta[1] = {rstest::kPi / 2.0};
  double points[3] = {0.0};
  double distances[1] = {0.0};
  double intensities[1] = {0.0};
  int actual = 0;

  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, theta, 0,
                               sensor_location_, points, distances,
                               intensities, 1, &actual),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, theta, -1,
                               sensor_location_, points, distances,
                               intensities, 1, &actual),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

TEST_F(LidarSimulatorTest, RejectsNonPositiveMaxPoints) {
  AddPlate();

  double phi[1] = {0.0};
  double theta[1] = {rstest::kPi / 2.0};
  double points[3] = {0.0};
  double distances[1] = {0.0};
  double intensities[1] = {0.0};
  int actual = 0;

  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, theta, 1,
                               sensor_location_, points, distances,
                               intensities, 0, &actual),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Run_LidarSimulator(targets_.get(), phi, theta, 1,
                               sensor_location_, points, distances,
                               intensities, -1, &actual),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

/*********************************************
 *
 *  Ray tracing behaviour
 *
 *********************************************/

/**
 * @brief With no geometry in the scene, no points should be returned
 *
 * @details Disabled: an empty target manager currently throws std::length_error
 * ("vector too long") while building the BVH, which Run_LidarSimulator reports
 * as RADARSIM_ERROR_EXCEPTION. An empty scene should instead return
 * RADARSIM_SUCCESS with actual_points = 0. Enable once the empty-scene path is
 * handled.
 */
TEST_F(LidarSimulatorTest, DISABLED_EmptySceneProducesNoPoints) {
  ASSERT_EQ(Shoot({0.0}, {rstest::kPi / 2.0}, 16), RADARSIM_SUCCESS);
  EXPECT_EQ(actual_points_, 0);
}

/**
 * @brief A ray aimed away from the plate returns no points
 */
TEST_F(LidarSimulatorTest, MissingRayProducesNoPoints) {
  AddPlate();

  // phi = pi points along -X, directly away from the plate.
  ASSERT_EQ(Shoot({rstest::kPi}, {rstest::kPi / 2.0}, 16), RADARSIM_SUCCESS);
  EXPECT_EQ(actual_points_, 0);
}

/**
 * @brief A ray aimed at the plate centre reports the correct hit point
 */
TEST_F(LidarSimulatorTest, DirectHitReportsPositionAndRange) {
  AddPlate();

  ASSERT_EQ(Shoot({0.0}, {rstest::kPi / 2.0}, 16), RADARSIM_SUCCESS);
  ASSERT_EQ(actual_points_, 1) << "the +X ray must hit the plate";

  EXPECT_NEAR(points_[0], kPlateRange, 1e-2);
  EXPECT_NEAR(points_[1], 0.0, 1e-2);
  EXPECT_NEAR(points_[2], 0.0, 1e-2);
  EXPECT_NEAR(distances_[0], kPlateRange, 1e-2);
  EXPECT_GT(intensities_[0], 0.0);
}

/**
 * @brief The angle arrays form a phi x theta grid, not zipped ray pairs
 *
 * @details The underlying engine allocates phi.size() * theta.size() rays, so
 * `num_rays` in the C API is the length of each angle array and the scan shoots
 * num_rays^2 rays. A caller sizing its output buffers as num_rays would be
 * silently truncated, which is why this is pinned down explicitly.
 */
TEST_F(LidarSimulatorTest, AngleArraysFormAGrid) {
  AddPlate();

  const double elevation = rstest::kPi / 2.0;
  // 3 azimuths x 3 elevations, all aimed within the plate's 2 m extent.
  ASSERT_EQ(Shoot({-0.05, 0.0, 0.05},
                  {elevation - 0.05, elevation, elevation + 0.05}, 32),
            RADARSIM_SUCCESS);

  EXPECT_EQ(actual_points_, 9) << "expected a 3 x 3 ray grid";
  for (int i = 0; i < actual_points_; i++) {
    EXPECT_NEAR(points_[i * 3], kPlateRange, 1e-1) << "point " << i;
    EXPECT_GT(distances_[i], 0.0) << "point " << i;
    EXPECT_TRUE(std::isfinite(distances_[i])) << "point " << i;
  }
}

/**
 * @brief The returned cloud never exceeds the advertised capacity
 */
TEST_F(LidarSimulatorTest, RespectsMaxPointsCap) {
  AddPlate();

  const double elevation = rstest::kPi / 2.0;
  // The 3 x 3 grid would yield 9 points; only 2 slots are offered.
  ASSERT_EQ(Shoot({-0.05, 0.0, 0.05},
                  {elevation - 0.05, elevation, elevation + 0.05},
                  /*max_points=*/2),
            RADARSIM_SUCCESS);

  EXPECT_EQ(actual_points_, 2)
      << "actual_points must be clamped to max_points";
}

/**
 * @brief Repeating a run reproduces the same cloud
 */
TEST_F(LidarSimulatorTest, RunIsRepeatable) {
  AddPlate();

  ASSERT_EQ(Shoot({0.0}, {rstest::kPi / 2.0}, 16), RADARSIM_SUCCESS);
  ASSERT_EQ(actual_points_, 1);
  const double first_distance = distances_[0];

  ASSERT_EQ(Shoot({0.0}, {rstest::kPi / 2.0}, 16), RADARSIM_SUCCESS);
  ASSERT_EQ(actual_points_, 1);
  EXPECT_DOUBLE_EQ(distances_[0], first_distance);
}

}  // namespace
