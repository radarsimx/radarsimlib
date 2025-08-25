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
    int version[3];
    Get_Version(version);
    
    // Check that version numbers are reasonable
    EXPECT_EQ(version[0], VERSION_MAJOR);
    EXPECT_EQ(version[1], VERSION_MINOR);
    EXPECT_EQ(version[2], VERSION_PATCH);
    EXPECT_GE(version[0], 0);
    EXPECT_GE(version[1], 0);
    EXPECT_GE(version[2], 0);
}
