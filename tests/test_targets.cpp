/*
 * @file test_targets.cpp
 * @brief Unit tests for Targets C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Target manager initialization and destruction
 * - Point target addition (constant and time-varying)
 * - Mesh target addition (constant and time-varying)
 * - Parameter validation with exact error codes
 * - Free-tier target and triangle limits
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

#include <vector>

#include "radarsim.h"
#include "test_helpers.hpp"

using rstest::TargetsPtr;
using rstest::TriangleMesh;

namespace {

/**
 * @brief Test fixture owning a target manager and default target parameters
 */
class TargetsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    targets = rstest::MakeTargets();
    ASSERT_NE(targets, nullptr);
  }

  /** @brief Add the fixture's default mesh triangle */
  int AddMesh(t_Targets* handle) {
    return Add_Mesh_Target(mesh.points.data(), mesh.cells.data(),
                           mesh.cell_size, motion.origin, motion.location,
                           motion.speed, motion.rotation, motion.rotation_rate,
                           1.0f, 0.0f, 1.0f, 0.0f, false, 0.0f, false, handle);
  }

  /** @brief Add the fixture's default point scatterer */
  int AddPoint(t_Targets* handle) {
    return Add_Point_Target(point_location, point_speed, point_rcs, point_phase,
                            handle);
  }

  TargetsPtr targets;
  TriangleMesh mesh;
  rstest::Motion motion;

  float point_location[3] = {10.0f, 5.0f, 0.0f};
  float point_speed[3] = {0.0f, 0.0f, 0.0f};
  float point_rcs = 10.0f;  ///< dBsm
  float point_phase = 0.0f;
};

/*********************************************
 *
 *  Initialization
 *
 *********************************************/

TEST_F(TargetsTest, InitStartsEmpty) {
  EXPECT_EQ(Get_Num_Targets(targets.get()), 0);
}

TEST_F(TargetsTest, FreeIsNullSafe) {
  Free_Targets(nullptr);  // must not crash
}

/*********************************************
 *
 *  Point targets
 *
 *********************************************/

TEST_F(TargetsTest, AddPointTargetSucceeds) {
  EXPECT_EQ(AddPoint(targets.get()), RADARSIM_SUCCESS);
}

TEST_F(TargetsTest, AddPointTargetRejectsNullArguments) {
  EXPECT_EQ(Add_Point_Target(nullptr, point_speed, point_rcs, point_phase,
                             targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Point_Target(point_location, nullptr, point_rcs, point_phase,
                             targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Point_Target(point_location, point_speed, point_rcs,
                             point_phase, nullptr),
            RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(TargetsTest, AddPointTargetArraySucceeds) {
  float location_array[6] = {10.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rcs_array[2] = {10.0f, 12.0f};
  float phase_array[2] = {0.0f, 0.5f};

  EXPECT_EQ(Add_Point_Target_Array(location_array, 2, speed, rcs_array,
                                   phase_array, 2, targets.get()),
            RADARSIM_SUCCESS);
}

TEST_F(TargetsTest, AddPointTargetArrayRejectsNullArguments) {
  float location_array[3] = {10.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rcs_array[1] = {10.0f};
  float phase_array[1] = {0.0f};

  EXPECT_EQ(Add_Point_Target_Array(nullptr, 1, speed, rcs_array, phase_array,
                                   1, targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Point_Target_Array(location_array, 1, nullptr, rcs_array,
                                   phase_array, 1, targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Point_Target_Array(location_array, 1, speed, nullptr,
                                   phase_array, 1, targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Point_Target_Array(location_array, 1, speed, rcs_array,
                                   nullptr, 1, targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Point_Target_Array(location_array, 1, speed, rcs_array,
                                   phase_array, 1, nullptr),
            RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(TargetsTest, AddPointTargetArrayRejectsNonPositiveSizes) {
  float location_array[3] = {10.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rcs_array[1] = {10.0f};
  float phase_array[1] = {0.0f};

  EXPECT_EQ(Add_Point_Target_Array(location_array, 0, speed, rcs_array,
                                   phase_array, 1, targets.get()),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Add_Point_Target_Array(location_array, -1, speed, rcs_array,
                                   phase_array, 1, targets.get()),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Add_Point_Target_Array(location_array, 1, speed, rcs_array,
                                   phase_array, 0, targets.get()),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Add_Point_Target_Array(location_array, 1, speed, rcs_array,
                                   phase_array, -1, targets.get()),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

/**
 * @brief Unlicensed builds accept a bounded number of point scatterers
 */
TEST_F(TargetsTest, FreeTierPointTargetLimit) {
  RS_SKIP_IF_LICENSED();

  for (int i = 0; i < rstest::kFreeTierMaxPointTargets; i++) {
    EXPECT_EQ(AddPoint(targets.get()), RADARSIM_SUCCESS) << "target " << i;
  }

  EXPECT_EQ(AddPoint(targets.get()), RADARSIM_ERROR_FREE_TIER_LIMIT);
}

/**
 * @brief The point limit is shared between the scalar and array entry points
 */
TEST_F(TargetsTest, FreeTierPointLimitAppliesToArrayVariant) {
  RS_SKIP_IF_LICENSED();

  for (int i = 0; i < rstest::kFreeTierMaxPointTargets; i++) {
    ASSERT_EQ(AddPoint(targets.get()), RADARSIM_SUCCESS);
  }

  float location_array[3] = {10.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rcs_array[1] = {10.0f};
  float phase_array[1] = {0.0f};

  EXPECT_EQ(Add_Point_Target_Array(location_array, 1, speed, rcs_array,
                                   phase_array, 1, targets.get()),
            RADARSIM_ERROR_FREE_TIER_LIMIT);
}

/*********************************************
 *
 *  Mesh targets
 *
 *********************************************/

TEST_F(TargetsTest, AddMeshTargetSucceeds) {
  EXPECT_EQ(AddMesh(targets.get()), RADARSIM_SUCCESS);
  EXPECT_EQ(Get_Num_Targets(targets.get()), 1);
  EXPECT_EQ(Get_Target_Mesh_Size(targets.get(), 0), mesh.cell_size);
}

TEST_F(TargetsTest, AddMeshTargetRejectsNullArguments) {
  EXPECT_EQ(Add_Mesh_Target(nullptr, mesh.cells.data(), mesh.cell_size,
                            motion.origin, motion.location, motion.speed,
                            motion.rotation, motion.rotation_rate, 1.0f, 0.0f,
                            1.0f, 0.0f, false, 0.0f, false, targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Mesh_Target(mesh.points.data(), nullptr, mesh.cell_size,
                            motion.origin, motion.location, motion.speed,
                            motion.rotation, motion.rotation_rate, 1.0f, 0.0f,
                            1.0f, 0.0f, false, 0.0f, false, targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Mesh_Target(mesh.points.data(), mesh.cells.data(),
                            mesh.cell_size, nullptr, motion.location,
                            motion.speed, motion.rotation,
                            motion.rotation_rate, 1.0f, 0.0f, 1.0f, 0.0f,
                            false, 0.0f, false, targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(AddMesh(nullptr), RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(TargetsTest, AddMeshTargetRejectsNonPositiveCellSize) {
  EXPECT_EQ(Add_Mesh_Target(mesh.points.data(), mesh.cells.data(), 0,
                            motion.origin, motion.location, motion.speed,
                            motion.rotation, motion.rotation_rate, 1.0f, 0.0f,
                            1.0f, 0.0f, false, 0.0f, false, targets.get()),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Add_Mesh_Target(mesh.points.data(), mesh.cells.data(), -1,
                            motion.origin, motion.location, motion.speed,
                            motion.rotation, motion.rotation_rate, 1.0f, 0.0f,
                            1.0f, 0.0f, false, 0.0f, false, targets.get()),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

TEST_F(TargetsTest, AddMeshTargetArraySucceeds) {
  float location_array[6] = {10.0f, 0.0f, 0.0f, 12.0f, 0.0f, 0.0f};
  float speed_array[6] = {0.0f};
  float rotation_array[6] = {0.0f};
  float rotation_rate_array[6] = {0.0f};

  EXPECT_EQ(Add_Mesh_Target_Array(mesh.points.data(), mesh.cells.data(),
                                  mesh.cell_size, motion.origin,
                                  location_array, speed_array, rotation_array,
                                  rotation_rate_array, 2, 1.0f, 0.0f, 1.0f,
                                  0.0f, false, 0.0f, false, targets.get()),
            RADARSIM_SUCCESS);
  EXPECT_EQ(Get_Num_Targets(targets.get()), 1);
}

TEST_F(TargetsTest, AddMeshTargetArrayRejectsNullArguments) {
  float motion_array[3] = {0.0f, 0.0f, 0.0f};

  EXPECT_EQ(Add_Mesh_Target_Array(nullptr, mesh.cells.data(), mesh.cell_size,
                                  motion.origin, motion_array, motion_array,
                                  motion_array, motion_array, 1, 1.0f, 0.0f,
                                  1.0f, 0.0f, false, 0.0f, false,
                                  targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Mesh_Target_Array(mesh.points.data(), mesh.cells.data(),
                                  mesh.cell_size, motion.origin, nullptr,
                                  motion_array, motion_array, motion_array, 1,
                                  1.0f, 0.0f, 1.0f, 0.0f, false, 0.0f, false,
                                  targets.get()),
            RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Add_Mesh_Target_Array(mesh.points.data(), mesh.cells.data(),
                                  mesh.cell_size, motion.origin, motion_array,
                                  motion_array, motion_array, motion_array, 1,
                                  1.0f, 0.0f, 1.0f, 0.0f, false, 0.0f, false,
                                  nullptr),
            RADARSIM_ERROR_NULL_POINTER);
}

TEST_F(TargetsTest, AddMeshTargetArrayRejectsNonPositiveSizes) {
  float motion_array[3] = {0.0f, 0.0f, 0.0f};

  EXPECT_EQ(Add_Mesh_Target_Array(mesh.points.data(), mesh.cells.data(), 0,
                                  motion.origin, motion_array, motion_array,
                                  motion_array, motion_array, 1, 1.0f, 0.0f,
                                  1.0f, 0.0f, false, 0.0f, false,
                                  targets.get()),
            RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Add_Mesh_Target_Array(mesh.points.data(), mesh.cells.data(),
                                  mesh.cell_size, motion.origin, motion_array,
                                  motion_array, motion_array, motion_array, 0,
                                  1.0f, 0.0f, 1.0f, 0.0f, false, 0.0f, false,
                                  targets.get()),
            RADARSIM_ERROR_INVALID_PARAMETER);
}

/**
 * @brief Unlicensed builds accept a bounded number of mesh targets
 */
TEST_F(TargetsTest, FreeTierMeshTargetLimit) {
  RS_SKIP_IF_LICENSED();

  for (int i = 0; i < rstest::kFreeTierMaxMeshTargets; i++) {
    EXPECT_EQ(AddMesh(targets.get()), RADARSIM_SUCCESS) << "target " << i;
  }

  EXPECT_EQ(AddMesh(targets.get()), RADARSIM_ERROR_FREE_TIER_LIMIT);
  EXPECT_EQ(Get_Num_Targets(targets.get()), rstest::kFreeTierMaxMeshTargets);
}

/**
 * @brief Unlicensed builds bound the triangle count of a single mesh
 */
TEST_F(TargetsTest, FreeTierMeshTriangleLimit) {
  RS_SKIP_IF_LICENSED();

  const int too_many = rstest::kFreeTierMaxMeshTriangles + 1;
  std::vector<float> points;
  std::vector<int> cells;
  for (int i = 0; i < too_many + 1; i++) {
    points.push_back(static_cast<float>(i));
    points.push_back(0.0f);
    points.push_back(0.0f);
  }
  for (int i = 0; i < too_many; i++) {
    cells.push_back(i);
    cells.push_back(i + 1);
    cells.push_back(0);
  }

  EXPECT_EQ(Add_Mesh_Target(points.data(), cells.data(), too_many,
                            motion.origin, motion.location, motion.speed,
                            motion.rotation, motion.rotation_rate, 1.0f, 0.0f,
                            1.0f, 0.0f, false, 0.0f, false, targets.get()),
            RADARSIM_ERROR_FREE_TIER_LIMIT);
  EXPECT_EQ(Get_Num_Targets(targets.get()), 0);

  // Exactly at the limit must still be accepted.
  EXPECT_EQ(Add_Mesh_Target(points.data(), cells.data(),
                            rstest::kFreeTierMaxMeshTriangles, motion.origin,
                            motion.location, motion.speed, motion.rotation,
                            motion.rotation_rate, 1.0f, 0.0f, 1.0f, 0.0f,
                            false, 0.0f, false, targets.get()),
            RADARSIM_SUCCESS);
}

/*********************************************
 *
 *  Mixed content
 *
 *********************************************/

/**
 * @brief Point and mesh targets are tracked in independent pools
 */
TEST_F(TargetsTest, PointAndMeshTargetsCoexist) {
  ASSERT_EQ(AddPoint(targets.get()), RADARSIM_SUCCESS);
  ASSERT_EQ(AddMesh(targets.get()), RADARSIM_SUCCESS);

  // Get_Num_Targets counts mesh targets only.
  EXPECT_EQ(Get_Num_Targets(targets.get()), 1);
}

/**
 * @brief Two managers are fully independent of one another
 */
TEST_F(TargetsTest, ManagersAreIndependent) {
  TargetsPtr other = rstest::MakeTargets();
  ASSERT_NE(other, nullptr);

  ASSERT_EQ(AddMesh(targets.get()), RADARSIM_SUCCESS);

  EXPECT_EQ(Get_Num_Targets(targets.get()), 1);
  EXPECT_EQ(Get_Num_Targets(other.get()), 0);
}

}  // namespace
