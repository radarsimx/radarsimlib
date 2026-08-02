/*
 * @file test_version.cpp
 * @brief Unit tests for RadarSim version and license query functions
 *
 * @details
 * Test scenarios:
 * - Version retrieval matches the header constants
 * - Is_Licensed / Set_License / Set_License_Files contract
 * - Get_License_Info truncation and null-buffer handling
 *
 * @note These tests assert invariants that hold in both ENABLE_LICENSE=ON and
 * ENABLE_LICENSE=OFF builds. A build without license verification reports
 * itself as licensed with an empty info string, so nothing here may assume a
 * particular license state.
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
#include <vector>

#include "radarsim.h"
#include "test_helpers.hpp"

/*********************************************
 *
 *  Version
 *
 *********************************************/

/**
 * @brief Get_Version reports exactly the header's version constants
 */
TEST(VersionTest, GetVersionMatchesHeader) {
  int version[3] = {-1, -1, -1};
  Get_Version(version);

  EXPECT_EQ(version[0], VERSION_MAJOR);
  EXPECT_EQ(version[1], VERSION_MINOR);
  EXPECT_EQ(version[2], VERSION_PATCH);
}

/**
 * @brief Get_Version writes exactly three elements and leaves the rest alone
 */
TEST(VersionTest, GetVersionDoesNotOverrun) {
  int version[4] = {-1, -1, -1, -7};
  Get_Version(version);

  EXPECT_GE(version[0], 0);
  EXPECT_GE(version[1], 0);
  EXPECT_GE(version[2], 0);
  EXPECT_EQ(version[3], -7) << "Get_Version wrote past the 3-element contract";
}

/*********************************************
 *
 *  License state
 *
 *********************************************/

/**
 * @brief Is_Licensed returns a stable boolean-valued int
 */
TEST(LicenseTest, IsLicensedIsStableBoolean) {
  const int state = Is_Licensed();
  EXPECT_TRUE(state == 0 || state == 1);
  EXPECT_EQ(Is_Licensed(), state) << "Is_Licensed is not idempotent";
}

/**
 * @brief Set_License with default arguments agrees with Is_Licensed
 *
 * @details The license state is process-global and can only transition from
 * unlicensed to licensed, so the only order-independent assertion is that both
 * entry points report the same thing.
 */
TEST(LicenseTest, SetLicenseAgreesWithIsLicensed) {
  const int set_result = Set_License(nullptr, nullptr);
  EXPECT_TRUE(set_result == 0 || set_result == 1);
  EXPECT_EQ(set_result, Is_Licensed());
}

/**
 * @brief Set_License_Files tolerates empty, null and bogus path lists
 */
TEST(LicenseTest, SetLicenseFilesHandlesDegenerateInput) {
  const int baseline = Is_Licensed();

  EXPECT_EQ(Set_License_Files(nullptr, 0, nullptr), baseline);
  EXPECT_EQ(Set_License_Files(nullptr, 3, "radarsimlib"), baseline);

  const char* paths[2] = {"", nullptr};
  EXPECT_EQ(Set_License_Files(paths, 2, nullptr), baseline);

  const char* missing[1] = {"this_file_does_not_exist.lic"};
  EXPECT_EQ(Set_License_Files(missing, 1, "radarsimlib"), baseline)
      << "A non-existent license file must not change the license state";
}

/*********************************************
 *
 *  License info string
 *
 *********************************************/

/**
 * @brief Get_License_Info returns the full string length and null-terminates
 */
TEST(LicenseTest, GetLicenseInfoReportsFullLength) {
  char buffer[256];
  std::memset(buffer, 'x', sizeof(buffer));

  const int len = Get_License_Info(buffer, sizeof(buffer));

  ASSERT_GE(len, 0);
  ASSERT_LT(len, static_cast<int>(sizeof(buffer)))
      << "Test buffer too small for this build's license info";
  EXPECT_EQ(static_cast<int>(std::strlen(buffer)), len);
}

/**
 * @brief A short buffer is truncated but always null-terminated
 */
TEST(LicenseTest, GetLicenseInfoTruncatesSafely) {
  const int full_len = Get_License_Info(nullptr, 0);
  if (full_len < 4) {
    GTEST_SKIP() << "License info is shorter than the truncation buffer";
  }

  char small_buffer[4];
  std::memset(small_buffer, 'x', sizeof(small_buffer));

  const int len = Get_License_Info(small_buffer, sizeof(small_buffer));

  EXPECT_EQ(len, full_len) << "Truncation must not change the reported length";
  EXPECT_EQ(small_buffer[sizeof(small_buffer) - 1], '\0');
  EXPECT_EQ(std::strlen(small_buffer), sizeof(small_buffer) - 1);
}

/**
 * @brief Null buffer and non-positive sizes are handled without writing
 */
TEST(LicenseTest, GetLicenseInfoHandlesNullBuffer) {
  const int reference = Get_License_Info(nullptr, 0);
  EXPECT_GE(reference, 0);

  EXPECT_EQ(Get_License_Info(nullptr, 128), reference);

  char buffer[8];
  std::memset(buffer, 'x', sizeof(buffer));
  EXPECT_EQ(Get_License_Info(buffer, 0), reference);
  EXPECT_EQ(buffer[0], 'x') << "Zero-sized buffer must not be written";
}
