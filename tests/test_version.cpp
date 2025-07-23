/*
 * @file test_version.cpp
 * @brief Unit tests for RadarSim version and utility functions
 *
 * @details
 * Test scenarios:
 * - Version retrieval functions
 * - Error message functions
 * - Utility functions
 * - Basic API validation
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
#include <string>

/**
 * @brief Test fixture for version and utility tests
 */
class VersionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }

    void TearDown() override {
        // Cleanup code if needed
    }
};

/**
 * @brief Test version retrieval function
 */
TEST_F(VersionTest, GetVersion) {
    int version[2];
    Get_Version(version);
    
    // Check that version numbers are reasonable
    EXPECT_EQ(version[0], VERSION_MAJOR);
    EXPECT_EQ(version[1], VERSION_MINOR);
    EXPECT_GE(version[0], 0);
    EXPECT_GE(version[1], 0);
}

/**
 * @brief Test error message retrieval
 */
TEST_F(VersionTest, GetErrorMessage) {
    // Test known error codes
    const char* success_msg = Get_Error_Message(RADARSIM_SUCCESS);
    EXPECT_NE(success_msg, nullptr);
    EXPECT_GT(strlen(success_msg), 0);

    const char* null_ptr_msg = Get_Error_Message(RADARSIM_ERROR_NULL_POINTER);
    EXPECT_NE(null_ptr_msg, nullptr);
    EXPECT_GT(strlen(null_ptr_msg), 0);

    const char* invalid_param_msg = Get_Error_Message(RADARSIM_ERROR_INVALID_PARAMETER);
    EXPECT_NE(invalid_param_msg, nullptr);
    EXPECT_GT(strlen(invalid_param_msg), 0);

    const char* memory_msg = Get_Error_Message(RADARSIM_ERROR_MEMORY_ALLOCATION);
    EXPECT_NE(memory_msg, nullptr);
    EXPECT_GT(strlen(memory_msg), 0);

    const char* free_tier_msg = Get_Error_Message(RADARSIM_ERROR_FREE_TIER_LIMIT);
    EXPECT_NE(free_tier_msg, nullptr);
    EXPECT_GT(strlen(free_tier_msg), 0);

    const char* exception_msg = Get_Error_Message(RADARSIM_ERROR_EXCEPTION);
    EXPECT_NE(exception_msg, nullptr);
    EXPECT_GT(strlen(exception_msg), 0);

    const char* too_many_rays_msg = Get_Error_Message(RADARSIM_ERROR_TOO_MANY_RAYS_PER_GRID);
    EXPECT_NE(too_many_rays_msg, nullptr);
    EXPECT_GT(strlen(too_many_rays_msg), 0);
}

/**
 * @brief Test pointer validation utility
 */
TEST_F(VersionTest, IsValidPointer) {
    // Test with valid pointer
    int test_value = 42;
    int* valid_ptr = &test_value;
    EXPECT_EQ(Is_Valid_Pointer(valid_ptr), 1);

    // Test with null pointer
    EXPECT_EQ(Is_Valid_Pointer(nullptr), 0);
    EXPECT_EQ(Is_Valid_Pointer(NULL), 0);
}

/**
 * @brief Test CPU core count function
 */
TEST_F(VersionTest, GetCPUCoreCount) {
    int core_count = Get_CPU_Core_Count();
    
    // Core count should be positive
    EXPECT_GT(core_count, 0);
    // Should be reasonable (not more than 256 cores)
    EXPECT_LE(core_count, 256);
}
