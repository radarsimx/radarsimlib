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

/**
 * @brief Test fixture for Targets tests
 */
class TargetsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup test data
        setupTestData();
    }

    void TearDown() override {
        // Cleanup targets
        if (valid_targets) {
            Free_Targets(valid_targets);
            valid_targets = nullptr;
        }
    }

    void setupTestData() {
        // Setup point target parameters
        point_location[0] = 10.0f; point_location[1] = 5.0f; point_location[2] = 0.0f;
        point_speed[0] = 0.0f; point_speed[1] = 0.0f; point_speed[2] = 0.0f;
        point_rcs = 10.0f; // 10 dBsm
        point_phase = 0.0f;

        // Setup simple mesh target (triangle)
        setupSimpleMesh();

        // Setup mesh target parameters
        mesh_origin[0] = 0.0f; mesh_origin[1] = 0.0f; mesh_origin[2] = 0.0f;
        mesh_location[0] = 20.0f; mesh_location[1] = 0.0f; mesh_location[2] = 0.0f;
        mesh_speed[0] = 0.0f; mesh_speed[1] = 0.0f; mesh_speed[2] = 0.0f;
        mesh_rotation[0] = 0.0f; mesh_rotation[1] = 0.0f; mesh_rotation[2] = 0.0f;
        mesh_rotation_rate[0] = 0.0f; mesh_rotation_rate[1] = 0.0f; mesh_rotation_rate[2] = 0.0f;
        
        ep_real = 1.0f; ep_imag = 0.0f; // Permittivity
        mu_real = 1.0f; mu_imag = 0.0f; // Permeability
        is_ground = false;
    }

    void setupSimpleMesh() {
        // Create a simple triangle mesh
        mesh_points = {
            0.0f, 0.0f, 0.0f,  // vertex 0
            1.0f, 0.0f, 0.0f,  // vertex 1
            0.5f, 1.0f, 0.0f   // vertex 2
        };

        mesh_cells = {0, 1, 2}; // One triangle
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
    EXPECT_EQ(Is_Valid_Pointer(valid_targets), 1);
}

/**
 * @brief Test adding point target
 */
TEST_F(TargetsTest, AddPointTarget) {
    valid_targets = Init_Targets();
    ASSERT_NE(valid_targets, nullptr);

    // Test adding valid point target
    int result = Add_Point_Target(
        point_location, point_speed, point_rcs, point_phase, valid_targets
    );
    
    EXPECT_EQ(result, RADARSIM_SUCCESS);
}

/**
 * @brief Test adding point target with null parameters
 */
TEST_F(TargetsTest, AddPointTargetNullParams) {
    valid_targets = Init_Targets();
    ASSERT_NE(valid_targets, nullptr);

    // Test with null location
    int result = Add_Point_Target(
        nullptr, point_speed, point_rcs, point_phase, valid_targets
    );
    EXPECT_NE(result, RADARSIM_SUCCESS);

    // Test with null speed
    result = Add_Point_Target(
        point_location, nullptr, point_rcs, point_phase, valid_targets
    );
    EXPECT_NE(result, RADARSIM_SUCCESS);

    // Test with null targets
    result = Add_Point_Target(
        point_location, point_speed, point_rcs, point_phase, nullptr
    );
    EXPECT_NE(result, RADARSIM_SUCCESS);
}

/**
 * @brief Test adding mesh target
 */
TEST_F(TargetsTest, AddMeshTarget) {
    valid_targets = Init_Targets();
    ASSERT_NE(valid_targets, nullptr);

    // Test adding valid mesh target
    int result = Add_Mesh_Target(
        mesh_points.data(), mesh_cells.data(), cell_size,
        mesh_origin, mesh_location, mesh_speed,
        mesh_rotation, mesh_rotation_rate,
        ep_real, ep_imag, mu_real, mu_imag,
        is_ground, valid_targets
    );
    
    EXPECT_EQ(result, RADARSIM_SUCCESS);
}

/**
 * @brief Test adding mesh target with null parameters
 */
TEST_F(TargetsTest, AddMeshTargetNullParams) {
    valid_targets = Init_Targets();
    ASSERT_NE(valid_targets, nullptr);

    // Test with null points
    int result = Add_Mesh_Target(
        nullptr, mesh_cells.data(), cell_size,
        mesh_origin, mesh_location, mesh_speed,
        mesh_rotation, mesh_rotation_rate,
        ep_real, ep_imag, mu_real, mu_imag,
        is_ground, valid_targets
    );
    EXPECT_NE(result, RADARSIM_SUCCESS);

    // Test with null cells
    result = Add_Mesh_Target(
        mesh_points.data(), nullptr, cell_size,
        mesh_origin, mesh_location, mesh_speed,
        mesh_rotation, mesh_rotation_rate,
        ep_real, ep_imag, mu_real, mu_imag,
        is_ground, valid_targets
    );
    EXPECT_NE(result, RADARSIM_SUCCESS);

    // Test with null targets
    result = Add_Mesh_Target(
        mesh_points.data(), mesh_cells.data(), cell_size,
        mesh_origin, mesh_location, mesh_speed,
        mesh_rotation, mesh_rotation_rate,
        ep_real, ep_imag, mu_real, mu_imag,
        is_ground, nullptr
    );
    EXPECT_NE(result, RADARSIM_SUCCESS);
}

/**
 * @brief Test adding mesh target with invalid parameters
 */
TEST_F(TargetsTest, AddMeshTargetInvalidParams) {
    valid_targets = Init_Targets();
    ASSERT_NE(valid_targets, nullptr);

    // Test with zero cell size
    int result = Add_Mesh_Target(
        mesh_points.data(), mesh_cells.data(), 0,
        mesh_origin, mesh_location, mesh_speed,
        mesh_rotation, mesh_rotation_rate,
        ep_real, ep_imag, mu_real, mu_imag,
        is_ground, valid_targets
    );
    EXPECT_NE(result, RADARSIM_SUCCESS);

    // Test with negative cell size
    result = Add_Mesh_Target(
        mesh_points.data(), mesh_cells.data(), -1,
        mesh_origin, mesh_location, mesh_speed,
        mesh_rotation, mesh_rotation_rate,
        ep_real, ep_imag, mu_real, mu_imag,
        is_ground, valid_targets
    );
    EXPECT_NE(result, RADARSIM_SUCCESS);
}

/**
 * @brief Test multiple targets
 */
TEST_F(TargetsTest, MultipleTargets) {
    valid_targets = Init_Targets();
    ASSERT_NE(valid_targets, nullptr);

    // Add point target
    int result1 = Add_Point_Target(
        point_location, point_speed, point_rcs, point_phase, valid_targets
    );
    EXPECT_EQ(result1, RADARSIM_SUCCESS);

    // Add mesh target
    int result2 = Add_Mesh_Target(
        mesh_points.data(), mesh_cells.data(), cell_size,
        mesh_origin, mesh_location, mesh_speed,
        mesh_rotation, mesh_rotation_rate,
        ep_real, ep_imag, mu_real, mu_imag,
        is_ground, valid_targets
    );
    EXPECT_EQ(result2, RADARSIM_SUCCESS);
}

/**
 * @brief Test target list destruction
 */
TEST_F(TargetsTest, FreeTargets) {
    valid_targets = Init_Targets();
    ASSERT_NE(valid_targets, nullptr);

    // Add some targets
    Add_Point_Target(point_location, point_speed, point_rcs, point_phase, valid_targets);
    Add_Mesh_Target(
        mesh_points.data(), mesh_cells.data(), cell_size,
        mesh_origin, mesh_location, mesh_speed,
        mesh_rotation, mesh_rotation_rate,
        ep_real, ep_imag, mu_real, mu_imag,
        is_ground, valid_targets
    );

    // Should not crash
    Free_Targets(valid_targets);
    valid_targets = nullptr; // Prevent double free in teardown

    // Should handle null pointer gracefully
    Free_Targets(nullptr);
}
