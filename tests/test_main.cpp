/*
 * @file test_main.cpp
 * @brief Google Test main entry point for RadarSim C wrapper
 *
 * @details
 * Main test runner for RadarSim C API wrapper functions.
 * This file initializes the Google Test framework and runs all
 * test cases for the C interface to RadarSimCpp.
 *
 * Features:
 * - Test framework initialization
 * - Command line argument handling
 * - Test suite execution for C wrapper
 * - Banner reporting the library version and license mode, so a run that
 *   skips free-tier tests is self-explanatory
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

#include <cstdio>

#include "test_helpers.hpp"

namespace {

/**
 * @brief Prints the environment the suite is running against
 *
 * @details Free-tier limit tests are skipped in licensed builds, so the mode
 * has to be visible in the log to make a skipped run understandable.
 */
class RadarSimEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    int version[3] = {0, 0, 0};
    Get_Version(version);
    std::printf("[  CONFIG  ] radarsimlib %d.%d.%d, license mode: %s\n",
                version[0], version[1], version[2],
                rstest::IsFreeTier() ? "free tier (limits enforced)"
                                     : "licensed (limits not enforced)");
  }
};

}  // namespace

/**
 * @brief Test framework entry point
 * @param argc Argument count
 * @param argv Argument array
 * @return Test execution status
 */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new RadarSimEnvironment());
  return RUN_ALL_TESTS();
}
