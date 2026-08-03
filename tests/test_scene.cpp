/*
 * @file test_scene.cpp
 * @brief Unit tests for scene state query C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Get_Radar_State for static, moving and rotating radar platforms
 * - Get_Radar_State for time-varying (Create_Radar_Array) platforms
 * - Get_Num_Targets / Get_Target_Mesh_Size
 * - Get_Target_Mesh_State for constant-motion and time-varying targets
 * - Parameter validation
 * - Deep-copy safety (a query must not mutate live target state)
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

using rstest::kPiF;
using rstest::kTolerance;
using rstest::RadarScenario;
using rstest::TargetsPtr;
using rstest::TriangleMesh;

namespace {

/** @brief Vertex stride of Get_Target_Mesh_State output: 3 vertices * 3 axes */
constexpr int kFloatsPerTriangle = 9;

/**
 * @brief Fixture for radar platform state queries
 */
class RadarStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Small waveform - these tests only exercise geometry, not signals.
    scenario_.num_samples = 8;
    scenario_.num_pulses = 1;
  }

  RadarScenario scenario_;
};

/**
 * @brief A static radar at the origin reports channel offsets verbatim and a
 * boresight along +X.
 */
TEST_F(RadarStateTest, StaticRadarAtOrigin) {
  scenario_.tx_channel_location[0] = 1.0f;
  scenario_.rx_channel_location[0] = -1.0f;
  ASSERT_TRUE(scenario_.Build());

  double timestamps[1] = {0.0};
  float tx_out[3], rx_out[3], boresight[3];

  ASSERT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 1, tx_out, rx_out,
                            boresight),
            RADARSIM_SUCCESS);

  EXPECT_NEAR(tx_out[0], 1.0f, kTolerance);
  EXPECT_NEAR(tx_out[1], 0.0f, kTolerance);
  EXPECT_NEAR(tx_out[2], 0.0f, kTolerance);

  EXPECT_NEAR(rx_out[0], -1.0f, kTolerance);
  EXPECT_NEAR(rx_out[1], 0.0f, kTolerance);
  EXPECT_NEAR(rx_out[2], 0.0f, kTolerance);

  EXPECT_NEAR(boresight[0], 1.0f, kTolerance);
  EXPECT_NEAR(boresight[1], 0.0f, kTolerance);
  EXPECT_NEAR(boresight[2], 0.0f, kTolerance);
}

/**
 * @brief Constant-velocity platforms extrapolate analytically at query time
 */
TEST_F(RadarStateTest, ConstantVelocityExtrapolation) {
  ASSERT_TRUE(scenario_.BuildTxRx());

  float location[3] = {0.0f, 0.0f, 0.0f};
  float speed[3] = {2.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(scenario_.AttachRadar(location, speed, rotation, rotation_rate));

  double timestamps[3] = {0.0, 5.0, 10.0};
  float tx_out[9], rx_out[9], boresight[9];

  ASSERT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 3, tx_out, rx_out,
                            boresight),
            RADARSIM_SUCCESS);

  // location(t) = (0,0,0) + t * (2,0,0), channel offset is zero
  EXPECT_NEAR(tx_out[0], 0.0f, kTolerance);
  EXPECT_NEAR(tx_out[3], 10.0f, kTolerance);
  EXPECT_NEAR(tx_out[6], 20.0f, kTolerance);
  EXPECT_NEAR(rx_out[3], 10.0f, kTolerance);
}

/**
 * @brief A 90 degree yaw rotates the boresight from +X to +Y and carries the
 * channel offsets with it.
 */
TEST_F(RadarStateTest, RotatedRadarBoresight) {
  scenario_.tx_channel_location[0] = 1.0f;
  ASSERT_TRUE(scenario_.BuildTxRx());

  float location[3] = {0.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {kPiF / 2.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(scenario_.AttachRadar(location, speed, rotation, rotation_rate));

  double timestamps[1] = {0.0};
  float tx_out[3], rx_out[3], boresight[3];

  ASSERT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 1, tx_out, rx_out,
                            boresight),
            RADARSIM_SUCCESS);

  EXPECT_NEAR(boresight[0], 0.0f, kTolerance);
  EXPECT_NEAR(boresight[1], 1.0f, kTolerance);
  EXPECT_NEAR(boresight[2], 0.0f, kTolerance);

  // Local tx offset (1,0,0) yawed by 90 degrees becomes (0,1,0).
  EXPECT_NEAR(tx_out[0], 0.0f, kTolerance);
  EXPECT_NEAR(tx_out[1], 1.0f, kTolerance);
}

/**
 * @brief A constant rotation rate sweeps the boresight over time
 */
TEST_F(RadarStateTest, RotationRateSweepsBoresight) {
  ASSERT_TRUE(scenario_.BuildTxRx());

  float location[3] = {0.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {kPiF / 2.0f, 0.0f, 0.0f};  // 90 deg/s yaw
  ASSERT_TRUE(scenario_.AttachRadar(location, speed, rotation, rotation_rate));

  double timestamps[2] = {0.0, 1.0};
  float tx_out[6], rx_out[6], boresight[6];

  ASSERT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 2, tx_out, rx_out,
                            boresight),
            RADARSIM_SUCCESS);

  EXPECT_NEAR(boresight[0], 1.0f, kTolerance);  // t = 0 -> +X
  EXPECT_NEAR(boresight[4], 1.0f, kTolerance);  // t = 1 -> +Y
}

/**
 * @brief Time-varying platforms interpolate against the frame start times and
 * clamp outside the covered interval.
 */
TEST_F(RadarStateTest, TimeVaryingInterpolationAndClamping) {
  ASSERT_TRUE(scenario_.BuildTxRx());

  double frame_start_time[3] = {0.0, 1.0, 2.0};
  float location_array[9] = {0.0f,  0.0f, 0.0f,   //
                             10.0f, 0.0f, 0.0f,   //
                             20.0f, 0.0f, 0.0f};
  float rotation_array[9] = {0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};

  ASSERT_TRUE(scenario_.AttachRadarArray(frame_start_time, 3, location_array,
                                         3, speed, rotation_array, 3,
                                         rotation_rate));

  double timestamps[4] = {0.5, 1.5, -1.0, 5.0};
  float tx_out[12], rx_out[12], boresight[12];

  ASSERT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 4, tx_out, rx_out,
                            boresight),
            RADARSIM_SUCCESS);

  EXPECT_NEAR(tx_out[0], 5.0f, kTolerance);   // interpolated
  EXPECT_NEAR(tx_out[3], 15.0f, kTolerance);  // interpolated
  EXPECT_NEAR(tx_out[6], 0.0f, kTolerance);   // clamped to the first frame
  EXPECT_NEAR(tx_out[9], 20.0f, kTolerance);  // clamped to the last frame
}

/**
 * @brief A motion array that does not match the frame count is rejected,
 * because there is nothing to interpolate against.
 */
TEST_F(RadarStateTest, TimeVaryingMismatchedFrameCountIsRejected) {
  ASSERT_TRUE(scenario_.BuildTxRx());

  double frame_start_time[1] = {0.0};
  float location_array[6] = {0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f};
  float rotation_array[6] = {0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};

  ASSERT_TRUE(scenario_.AttachRadarArray(frame_start_time, 1, location_array,
                                         2, speed, rotation_array, 2,
                                         rotation_rate));

  double timestamps[1] = {0.0};
  float tx_out[3], rx_out[3], boresight[3];

  EXPECT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 1, tx_out, rx_out,
                            boresight),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

TEST_F(RadarStateTest, RejectsNullArguments) {
  ASSERT_TRUE(scenario_.Build());

  double timestamps[1] = {0.0};
  float tx_out[3], rx_out[3], boresight[3];

  EXPECT_EQ(Get_Radar_State(nullptr, timestamps, 1, tx_out, rx_out, boresight),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Radar_State(scenario_.radar(), nullptr, 1, tx_out, rx_out,
                            boresight),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 1, nullptr, rx_out,
                            boresight),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 1, tx_out, nullptr,
                            boresight),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 1, tx_out, rx_out,
                            nullptr),
            RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(RadarStateTest, RejectsNonPositiveTimestampCount) {
  ASSERT_TRUE(scenario_.Build());

  double timestamps[1] = {0.0};
  float tx_out[3], rx_out[3], boresight[3];

  EXPECT_EQ(Get_Radar_State(scenario_.radar(), timestamps, 0, tx_out, rx_out,
                            boresight),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Get_Radar_State(scenario_.radar(), timestamps, -1, tx_out, rx_out,
                            boresight),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

/*********************************************
 *
 *  Target mesh state
 *
 *********************************************/

/**
 * @brief Fixture owning a target manager and a unit triangle
 */
class TargetMeshStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    targets_ = rstest::MakeTargets();
    ASSERT_NE(targets_, nullptr);
  }

  /** @brief Add the fixture triangle with the given motion */
  int AddTriangle(const rstest::Motion& motion) {
    return Add_Mesh_Target(mesh_.points.data(), mesh_.cells.data(),
                           mesh_.cell_size, const_cast<float*>(motion.origin),
                           const_cast<float*>(motion.location),
                           const_cast<float*>(motion.speed),
                           const_cast<float*>(motion.rotation),
                           const_cast<float*>(motion.rotation_rate), 1.0f,
                           0.0f, 1.0f, 0.0f, false, 0.0f, false,
                           targets_.get());
  }

  TargetsPtr targets_;
  TriangleMesh mesh_;
};

TEST_F(TargetMeshStateTest, CountsAndSizesTrackAddedTargets) {
  EXPECT_EQ(Get_Num_Targets(targets_.get()), 0);
  EXPECT_EQ(Get_Target_Mesh_Size(targets_.get(), 0), 0);

  rstest::Motion motion;
  motion.location[0] = 10.0f;
  ASSERT_EQ(AddTriangle(motion), RADARSIM_SUCCESS);

  EXPECT_EQ(Get_Num_Targets(targets_.get()), 1);
  EXPECT_EQ(Get_Target_Mesh_Size(targets_.get(), 0), mesh_.cell_size);
  EXPECT_EQ(Get_Target_Mesh_Size(targets_.get(), 1), 0);   // past the end
  EXPECT_EQ(Get_Target_Mesh_Size(targets_.get(), -1), 0);  // negative index
}

TEST_F(TargetMeshStateTest, HelpersAreNullSafe) {
  EXPECT_EQ(Get_Num_Targets(nullptr), 0);
  EXPECT_EQ(Get_Target_Mesh_Size(nullptr, 0), 0);
}

/**
 * @brief Constant-motion vertices translate by location + speed * t
 */
TEST_F(TargetMeshStateTest, ConstantMotionTranslation) {
  rstest::Motion motion;
  motion.location[0] = 10.0f;
  motion.speed[0] = 1.0f;
  ASSERT_EQ(AddTriangle(motion), RADARSIM_SUCCESS);

  double timestamps[2] = {0.0, 5.0};
  std::vector<double> points_out(2 * mesh_.cell_size * kFloatsPerTriangle);

  ASSERT_EQ(Get_Target_Mesh_State(targets_.get(), 0, timestamps, 2, nullptr, 0,
                                  points_out.data()),
            RADARSIM_SUCCESS);

  // t = 0: vertex 0 = (0,0,0) + (10,0,0)
  EXPECT_NEAR(points_out[0], 10.0, 1e-3);
  EXPECT_NEAR(points_out[1], 0.0, 1e-3);
  EXPECT_NEAR(points_out[2], 0.0, 1e-3);
  // t = 0: vertex 1 = (1,0,0) + (10,0,0)
  EXPECT_NEAR(points_out[3], 11.0, 1e-3);

  // t = 5: vertex 0 = (10,0,0) + 5 * (1,0,0)
  const int frame = mesh_.cell_size * kFloatsPerTriangle;
  EXPECT_NEAR(points_out[frame + 0], 15.0, 1e-3);
  EXPECT_NEAR(points_out[frame + 1], 0.0, 1e-3);
  EXPECT_NEAR(points_out[frame + 2], 0.0, 1e-3);
}

/**
 * @brief Query timestamps may be supplied out of order
 */
TEST_F(TargetMeshStateTest, UnsortedTimestampsMapToTheirOwnSlots) {
  rstest::Motion motion;
  motion.location[0] = 10.0f;
  motion.speed[0] = 1.0f;
  ASSERT_EQ(AddTriangle(motion), RADARSIM_SUCCESS);

  double timestamps[3] = {5.0, 0.0, 2.0};
  std::vector<double> points_out(3 * mesh_.cell_size * kFloatsPerTriangle);

  ASSERT_EQ(Get_Target_Mesh_State(targets_.get(), 0, timestamps, 3, nullptr, 0,
                                  points_out.data()),
            RADARSIM_SUCCESS);

  const int frame = mesh_.cell_size * kFloatsPerTriangle;
  EXPECT_NEAR(points_out[0 * frame], 15.0, 1e-3);
  EXPECT_NEAR(points_out[1 * frame], 10.0, 1e-3);
  EXPECT_NEAR(points_out[2 * frame], 12.0, 1e-3);
}

/**
 * @brief Time-varying targets snap to the nearest motion sample
 */
TEST_F(TargetMeshStateTest, TimeVaryingNearestNeighbor) {
  float origin[3] = {0.0f, 0.0f, 0.0f};
  float location_array[6] = {0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f};
  float speed_array[6] = {0.0f};
  float rotation_array[6] = {0.0f};
  float rotation_rate_array[6] = {0.0f};

  ASSERT_EQ(Add_Mesh_Target_Array(mesh_.points.data(), mesh_.cells.data(),
                                  mesh_.cell_size, origin, location_array,
                                  speed_array, rotation_array,
                                  rotation_rate_array, 2, 1.0f, 0.0f, 1.0f,
                                  0.0f, false, 0.0f, false, targets_.get()),
            RADARSIM_SUCCESS);

  double sim_timestamps[2] = {0.0, 10.0};
  double timestamps[2] = {1.0, 9.0};
  std::vector<double> points_out(2 * mesh_.cell_size * kFloatsPerTriangle);

  ASSERT_EQ(Get_Target_Mesh_State(targets_.get(), 0, timestamps, 2,
                                  sim_timestamps, 2, points_out.data()),
            RADARSIM_SUCCESS);

  const int frame = mesh_.cell_size * kFloatsPerTriangle;
  EXPECT_NEAR(points_out[0], 0.0, 1e-3);           // t=1 -> sample 0
  EXPECT_NEAR(points_out[frame + 0], 20.0, 1e-3);  // t=9 -> sample 1
}

/**
 * @brief A time-varying target cannot be queried without sim_timestamps
 */
TEST_F(TargetMeshStateTest, TimeVaryingRequiresSimTimestamps) {
  float origin[3] = {0.0f, 0.0f, 0.0f};
  float location_array[6] = {0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f};
  float speed_array[6] = {0.0f};
  float rotation_array[6] = {0.0f};
  float rotation_rate_array[6] = {0.0f};

  ASSERT_EQ(Add_Mesh_Target_Array(mesh_.points.data(), mesh_.cells.data(),
                                  mesh_.cell_size, origin, location_array,
                                  speed_array, rotation_array,
                                  rotation_rate_array, 2, 1.0f, 0.0f, 1.0f,
                                  0.0f, false, 0.0f, false, targets_.get()),
            RADARSIM_SUCCESS);

  double timestamps[1] = {1.0};
  std::vector<double> points_out(mesh_.cell_size * kFloatsPerTriangle);

  EXPECT_EQ(Get_Target_Mesh_State(targets_.get(), 0, timestamps, 1, nullptr, 0,
                                  points_out.data()),
            RADARSIM_ERROR_INVALID_PARAMETER);

  // A mismatched sim_timestamps length is rejected the same way.
  double sim_timestamps[3] = {0.0, 5.0, 10.0};
  EXPECT_EQ(Get_Target_Mesh_State(targets_.get(), 0, timestamps, 1,
                                  sim_timestamps, 3, points_out.data()),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

TEST_F(TargetMeshStateTest, RejectsNullArguments) {
  rstest::Motion motion;
  motion.location[0] = 10.0f;
  ASSERT_EQ(AddTriangle(motion), RADARSIM_SUCCESS);

  double timestamps[1] = {0.0};
  std::vector<double> points_out(mesh_.cell_size * kFloatsPerTriangle);

  EXPECT_EQ(Get_Target_Mesh_State(nullptr, 0, timestamps, 1, nullptr, 0,
                                  points_out.data()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Target_Mesh_State(targets_.get(), 0, nullptr, 1, nullptr, 0,
                                  points_out.data()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Target_Mesh_State(targets_.get(), 0, timestamps, 1, nullptr, 0,
                                  nullptr),
            RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(TargetMeshStateTest, RejectsOutOfRangeArguments) {
  rstest::Motion motion;
  motion.location[0] = 10.0f;
  ASSERT_EQ(AddTriangle(motion), RADARSIM_SUCCESS);

  double timestamps[1] = {0.0};
  std::vector<double> points_out(mesh_.cell_size * kFloatsPerTriangle);

  EXPECT_EQ(Get_Target_Mesh_State(targets_.get(), 5, timestamps, 1, nullptr, 0,
                                  points_out.data()),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Get_Target_Mesh_State(targets_.get(), -1, timestamps, 1, nullptr,
                                  0, points_out.data()),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Get_Target_Mesh_State(targets_.get(), 0, timestamps, 0, nullptr, 0,
                                  points_out.data()),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Get_Target_Mesh_State(targets_.get(), 0, timestamps, -1, nullptr,
                                  0, points_out.data()),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

/**
 * @brief Querying a future timestamp must not advance the live target
 *
 * @details The query works on a deep copy. Re-querying t = 0 afterwards has to
 * reproduce the construction state; if the copy leaked, the second query would
 * see a target already displaced by speed * 100.
 */
TEST_F(TargetMeshStateTest, QueryDoesNotMutateLiveTarget) {
  rstest::Motion motion;
  motion.location[0] = 10.0f;
  motion.speed[0] = 1.0f;
  ASSERT_EQ(AddTriangle(motion), RADARSIM_SUCCESS);

  double far_future[1] = {100.0};
  std::vector<double> far_points(mesh_.cell_size * kFloatsPerTriangle);
  ASSERT_EQ(Get_Target_Mesh_State(targets_.get(), 0, far_future, 1, nullptr, 0,
                                  far_points.data()),
            RADARSIM_SUCCESS);
  ASSERT_NEAR(far_points[0], 110.0, 1e-2) << "the query itself did not move";

  double now[1] = {0.0};
  std::vector<double> now_points(mesh_.cell_size * kFloatsPerTriangle);
  ASSERT_EQ(Get_Target_Mesh_State(targets_.get(), 0, now, 1, nullptr, 0,
                                  now_points.data()),
            RADARSIM_SUCCESS);
  EXPECT_NEAR(now_points[0], 10.0, 1e-2)
      << "an earlier query mutated the live target";
}

}  // namespace
