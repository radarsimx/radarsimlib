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

#include <cstring>
#include <string>

#include "radarsim.h"

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

/**
 * @brief Test default license state (unlicensed)
 */
TEST_F(VersionTest, DefaultLicenseState) {
  // By default, should not be licensed
  EXPECT_EQ(Is_Licensed(), 0);
}

/**
 * @brief Test Set_License activates license (open source build)
 */
TEST_F(VersionTest, SetLicense) {
  // In open source build (no RADARSIMCPP_ENABLE_LICENSE), SetLicense
  // should set is_licensed_ = true
  Set_License(nullptr, nullptr);
  EXPECT_EQ(Is_Licensed(), 1);
}

/**
 * @brief Test Get_License_Info
 */
TEST_F(VersionTest, GetLicenseInfo) {
  Set_License(nullptr, nullptr);

  char buffer[256];
  int len = Get_License_Info(buffer, sizeof(buffer));

  // Should return non-empty license info string
  EXPECT_GT(len, 0);
  EXPECT_GT(static_cast<int>(strlen(buffer)), 0);
}

/**
 * @brief Test Get_License_Info with small buffer
 */
TEST_F(VersionTest, GetLicenseInfoSmallBuffer) {
  Set_License(nullptr, nullptr);

  char small_buffer[4];
  int len = Get_License_Info(small_buffer, sizeof(small_buffer));

  // Returned length should be the full string length
  EXPECT_GT(len, 3);
  // Buffer should be null-terminated and truncated
  EXPECT_EQ(small_buffer[3], '\0');
}

/**
 * @brief Test Get_License_Info with null buffer
 */
TEST_F(VersionTest, GetLicenseInfoNullBuffer) {
  Set_License(nullptr, nullptr);

  // Should not crash with null buffer
  int len = Get_License_Info(nullptr, 0);
  EXPECT_GT(len, 0);
}
