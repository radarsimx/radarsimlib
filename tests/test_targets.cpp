/*
 * @file test_targets.cpp
 * @brief Unit tests for Targets C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Target list initialization and destruction
 * - Point target addition
 * - Mesh target addition
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

#include <vector>

#include "radarsim.h"

/**
 * @brief Test fixture for Targets tests
 *
 * @details
 * This test suite demonstrates both automatic and manual memory management:
 * - Automatic: Objects are automatically cleaned up at program exit
 * - Manual: Objects can still be explicitly freed with Free_Targets()
 * - Mixed: Some objects freed manually, others automatically
 */
class TargetsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup test data
    setupTestData();
  }

  void TearDown() override {
    // With automatic memory management, manual cleanup is optional
    // Objects will be automatically cleaned up at program exit
    // But we can still manually free for testing purposes
    if (valid_targets) {
      Free_Targets(valid_targets);
      valid_targets = nullptr;
    }
  }

  void setupTestData() {
    // Setup point target parameters
    point_location[0] = 10.0f;
    point_location[1] = 5.0f;
    point_location[2] = 0.0f;
    point_speed[0] = 0.0f;
    point_speed[1] = 0.0f;
    point_speed[2] = 0.0f;
    point_rcs = 10.0f;  // 10 dBsm
    point_phase = 0.0f;

    // Setup simple mesh target (triangle)
    setupSimpleMesh();

    // Setup mesh target parameters
    mesh_origin[0] = 0.0f;
    mesh_origin[1] = 0.0f;
    mesh_origin[2] = 0.0f;
    mesh_location[0] = 20.0f;
    mesh_location[1] = 0.0f;
    mesh_location[2] = 0.0f;
    mesh_speed[0] = 0.0f;
    mesh_speed[1] = 0.0f;
    mesh_speed[2] = 0.0f;
    mesh_rotation[0] = 0.0f;
    mesh_rotation[1] = 0.0f;
    mesh_rotation[2] = 0.0f;
    mesh_rotation_rate[0] = 0.0f;
    mesh_rotation_rate[1] = 0.0f;
    mesh_rotation_rate[2] = 0.0f;

    ep_real = 1.0f;
    ep_imag = 0.0f;  // Permittivity
    mu_real = 1.0f;
    mu_imag = 0.0f;  // Permeability
    is_ground = false;
  }

  void setupSimpleMesh() {
    // Create a simple triangle mesh
    mesh_points = {
        0.0f, 0.0f, 0.0f,  // vertex 0
        1.0f, 0.0f, 0.0f,  // vertex 1
        0.5f, 1.0f, 0.0f   // vertex 2
    };

    mesh_cells = {0, 1, 2};  // One triangle
    cell_size = 1;
  }

  // Test data
  float point_location[3], point_speed[3];
  float point_rcs, point_phase;

  std::vector<float> mesh_points;
  std::vector<int> mesh_cells;
  int cell_size;
  float mesh_origin[3], mesh_location[3], mesh_speed[3];
  float mesh_rotation[3], mesh_rotation_rate[3];
  float ep_real, ep_imag, mu_real, mu_imag;
  bool is_ground;

  t_Targets* valid_targets = nullptr;
};

/**
 * @brief Test target list initialization
 */
TEST_F(TargetsTest, InitTargets) {
  valid_targets = Init_Targets();

  EXPECT_NE(valid_targets, nullptr);
  // Note: No Is_Valid_Pointer function available in the C API
}

/**
 * @brief Test adding point target
 */
TEST_F(TargetsTest, AddPointTarget) {
  valid_targets = Init_Targets();
  ASSERT_NE(valid_targets, nullptr);

  // Test adding valid point target
  int result = Add_Point_Target(point_location, point_speed, point_rcs,
                                point_phase, valid_targets);

  EXPECT_EQ(result, 0);  // 0 for success according to API
}

/**
 * @brief Test adding point target with null parameters
 */
TEST_F(TargetsTest, AddPointTargetNullParams) {
  valid_targets = Init_Targets();
  ASSERT_NE(valid_targets, nullptr);

  // Test with null location
  int result = Add_Point_Target(nullptr, point_speed, point_rcs, point_phase,
                                valid_targets);
  EXPECT_NE(result, 0);  // Non-zero for failure

  // Test with null speed
  result = Add_Point_Target(point_location, nullptr, point_rcs, point_phase,
                            valid_targets);
  EXPECT_NE(result, 0);  // Non-zero for failure

  // Test with null targets
  result = Add_Point_Target(point_location, point_speed, point_rcs, point_phase,
                            nullptr);
  EXPECT_NE(result, 0);  // Non-zero for failure
}

/**
 * @brief Test adding mesh target
 */
TEST_F(TargetsTest, AddMeshTarget) {
  valid_targets = Init_Targets();
  ASSERT_NE(valid_targets, nullptr);

  // Test adding valid mesh target
  int result = Add_Mesh_Target(
      mesh_points.data(), mesh_cells.data(), cell_size, mesh_origin,
      mesh_location, mesh_speed, mesh_rotation, mesh_rotation_rate, ep_real,
      ep_imag, mu_real, mu_imag, is_ground, valid_targets);

  EXPECT_EQ(result, 0);  // 0 for success according to API
}

/**
 * @brief Test adding mesh target with null parameters
 */
TEST_F(TargetsTest, AddMeshTargetNullParams) {
  valid_targets = Init_Targets();
  ASSERT_NE(valid_targets, nullptr);

  // Test with null points
  int result = Add_Mesh_Target(
      nullptr, mesh_cells.data(), cell_size, mesh_origin, mesh_location,
      mesh_speed, mesh_rotation, mesh_rotation_rate, ep_real, ep_imag, mu_real,
      mu_imag, is_ground, valid_targets);
  EXPECT_NE(result, 0);  // Non-zero for failure

  // Test with null cells
  result = Add_Mesh_Target(mesh_points.data(), nullptr, cell_size, mesh_origin,
                           mesh_location, mesh_speed, mesh_rotation,
                           mesh_rotation_rate, ep_real, ep_imag, mu_real,
                           mu_imag, is_ground, valid_targets);
  EXPECT_NE(result, 0);  // Non-zero for failure

  // Test with null targets
  result = Add_Mesh_Target(mesh_points.data(), mesh_cells.data(), cell_size,
                           mesh_origin, mesh_location, mesh_speed,
                           mesh_rotation, mesh_rotation_rate, ep_real, ep_imag,
                           mu_real, mu_imag, is_ground, nullptr);
  EXPECT_NE(result, 0);  // Non-zero for failure
}

/**
 * @brief Test adding mesh target with invalid parameters
 */
TEST_F(TargetsTest, AddMeshTargetInvalidParams) {
  valid_targets = Init_Targets();
  ASSERT_NE(valid_targets, nullptr);

  // Test with zero cell size
  int result = Add_Mesh_Target(
      mesh_points.data(), mesh_cells.data(), 0, mesh_origin, mesh_location,
      mesh_speed, mesh_rotation, mesh_rotation_rate, ep_real, ep_imag, mu_real,
      mu_imag, is_ground, valid_targets);
  EXPECT_NE(result, 0);  // Non-zero for failure

  // Test with negative cell size
  result = Add_Mesh_Target(mesh_points.data(), mesh_cells.data(), -1,
                           mesh_origin, mesh_location, mesh_speed,
                           mesh_rotation, mesh_rotation_rate, ep_real, ep_imag,
                           mu_real, mu_imag, is_ground, valid_targets);
  EXPECT_NE(result, 0);  // Non-zero for failure
}

/**
 * @brief Test multiple targets
 */
TEST_F(TargetsTest, MultipleTargets) {
  valid_targets = Init_Targets();
  ASSERT_NE(valid_targets, nullptr);

  // Add point target
  int result1 = Add_Point_Target(point_location, point_speed, point_rcs,
                                 point_phase, valid_targets);
  EXPECT_EQ(result1, 0);  // 0 for success according to API

  // Add mesh target
  int result2 = Add_Mesh_Target(
      mesh_points.data(), mesh_cells.data(), cell_size, mesh_origin,
      mesh_location, mesh_speed, mesh_rotation, mesh_rotation_rate, ep_real,
      ep_imag, mu_real, mu_imag, is_ground, valid_targets);
  EXPECT_EQ(result2, 0);  // 0 for success according to API
}

/**
 * @brief Test automatic memory management
 */
TEST_F(TargetsTest, AutomaticMemoryManagement) {
  // Test that we can create target lists without manual cleanup
  t_Targets* targets1 = Init_Targets();
  ASSERT_NE(targets1, nullptr);

  t_Targets* targets2 = Init_Targets();
  ASSERT_NE(targets2, nullptr);

  // Add different types of targets to both lists
  int result1 = Add_Point_Target(point_location, point_speed, point_rcs,
                                 point_phase, targets1);
  EXPECT_EQ(result1, 0);

  int result2 = Add_Mesh_Target(
      mesh_points.data(), mesh_cells.data(), cell_size, mesh_origin,
      mesh_location, mesh_speed, mesh_rotation, mesh_rotation_rate, ep_real,
      ep_imag, mu_real, mu_imag, is_ground, targets2);
  EXPECT_EQ(result2, 0);

  // Don't call Free_Targets - test automatic cleanup
  // These target lists will be automatically cleaned up at program exit
}

/**
 * @brief Test manual vs automatic cleanup
 */
TEST_F(TargetsTest, ManualVsAutomaticCleanup) {
  // Create two target lists
  t_Targets* manual_targets = Init_Targets();
  t_Targets* auto_targets = Init_Targets();

  ASSERT_NE(manual_targets, nullptr);
  ASSERT_NE(auto_targets, nullptr);

  // Add targets to both
  Add_Point_Target(point_location, point_speed, point_rcs, point_phase,
                   manual_targets);
  Add_Point_Target(point_location, point_speed, point_rcs + 5.0f, point_phase,
                   auto_targets);

  // Manually free one target list
  Free_Targets(manual_targets);

  // Leave auto_targets for automatic cleanup at program exit
  // This demonstrates both cleanup methods work
}

/**
 * @brief Test target list destruction (original test preserved)
 */
TEST_F(TargetsTest, FreeTargets) {
  // Test freeing a valid target list
  t_Targets* test_targets = Init_Targets();
  ASSERT_NE(test_targets, nullptr);

  // Add some targets
  Add_Point_Target(point_location, point_speed, point_rcs, point_phase,
                   test_targets);
  Add_Mesh_Target(mesh_points.data(), mesh_cells.data(), cell_size, mesh_origin,
                  mesh_location, mesh_speed, mesh_rotation, mesh_rotation_rate,
                  ep_real, ep_imag, mu_real, mu_imag, is_ground, test_targets);

  // Should not crash when freeing a valid target list
  Free_Targets(test_targets);

  // Test freeing a null pointer - should handle gracefully
  Free_Targets(nullptr);
}

/**
 * @brief Test automatic cleanup control
 */
TEST_F(TargetsTest, AutomaticCleanupControl) {
  // Test enabling automatic cleanup (should be enabled by default)
//   Enable_Automatic_Cleanup(true);

  // Create a target list that will be automatically cleaned up
  t_Targets* targets = Init_Targets();
  ASSERT_NE(targets, nullptr);

  // Verify it works by adding both types of targets
  int result1 = Add_Point_Target(point_location, point_speed, point_rcs,
                                 point_phase, targets);
  EXPECT_EQ(result1, 0);

  int result2 = Add_Mesh_Target(
      mesh_points.data(), mesh_cells.data(), cell_size, mesh_origin,
      mesh_location, mesh_speed, mesh_rotation, mesh_rotation_rate, ep_real,
      ep_imag, mu_real, mu_imag, is_ground, targets);
  EXPECT_EQ(result2, 0);

  // Don't free - let automatic cleanup handle it
}
