/*
 * @file test_radar.cpp
 * @brief Unit tests for Radar system C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Create_Radar / Create_Radar_Array parameter validation
 * - Get_BB_Size against the radar's own dimensions
 * - Free_Radar null-safety and tx/rx ownership semantics
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

using rstest::RadarPtr;
using rstest::RadarScenario;

namespace {

/**
 * @brief Fixture providing a built transmitter/receiver pair to attach to
 */
class RadarTest : public ::testing::Test {
 protected:
  void SetUp() override { ASSERT_TRUE(scenario_.BuildTxRx()); }

  RadarScenario scenario_;

  double frame_start_time[2] = {0.0, 1.0};
  float location[3] = {0.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};
};

/*********************************************
 *
 *  Create_Radar
 *
 *********************************************/

TEST_F(RadarTest, CreateWithValidParameters) {
  RadarPtr radar(Create_Radar(scenario_.tx(), scenario_.rx(),
                              frame_start_time, 1, location, speed, rotation,
                              rotation_rate));

  ASSERT_NE(radar, nullptr);
  EXPECT_GT(Get_BB_Size(radar.get()), 0);
}

TEST_F(RadarTest, CreateRejectsNullArguments) {
  EXPECT_EQ(Create_Radar(nullptr, scenario_.rx(), frame_start_time, 1,
                         location, speed, rotation, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar(scenario_.tx(), nullptr, frame_start_time, 1,
                         location, speed, rotation, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar(scenario_.tx(), scenario_.rx(), nullptr, 1, location,
                         speed, rotation, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar(scenario_.tx(), scenario_.rx(), frame_start_time, 1,
                         nullptr, speed, rotation, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar(scenario_.tx(), scenario_.rx(), frame_start_time, 1,
                         location, nullptr, rotation, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar(scenario_.tx(), scenario_.rx(), frame_start_time, 1,
                         location, speed, nullptr, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar(scenario_.tx(), scenario_.rx(), frame_start_time, 1,
                         location, speed, rotation, nullptr),
            nullptr);
}

TEST_F(RadarTest, CreateRejectsNonPositiveFrameCount) {
  EXPECT_EQ(Create_Radar(scenario_.tx(), scenario_.rx(), frame_start_time, 0,
                         location, speed, rotation, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar(scenario_.tx(), scenario_.rx(), frame_start_time, -1,
                         location, speed, rotation, rotation_rate),
            nullptr);
}

/*********************************************
 *
 *  Create_Radar_Array
 *
 *********************************************/

TEST_F(RadarTest, CreateArrayWithValidParameters) {
  float location_array[6] = {0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f};
  float rotation_array[6] = {0.0f};

  RadarPtr radar(Create_Radar_Array(scenario_.tx(), scenario_.rx(),
                                    frame_start_time, 2, location_array, 2,
                                    speed, rotation_array, 2, rotation_rate));

  ASSERT_NE(radar, nullptr);
  EXPECT_GT(Get_BB_Size(radar.get()), 0);
}

TEST_F(RadarTest, CreateArrayRejectsNullArguments) {
  float location_array[3] = {0.0f, 0.0f, 0.0f};
  float rotation_array[3] = {0.0f, 0.0f, 0.0f};

  EXPECT_EQ(Create_Radar_Array(nullptr, scenario_.rx(), frame_start_time, 1,
                               location_array, 1, speed, rotation_array, 1,
                               rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar_Array(scenario_.tx(), nullptr, frame_start_time, 1,
                               location_array, 1, speed, rotation_array, 1,
                               rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar_Array(scenario_.tx(), scenario_.rx(), nullptr, 1,
                               location_array, 1, speed, rotation_array, 1,
                               rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar_Array(scenario_.tx(), scenario_.rx(),
                               frame_start_time, 1, nullptr, 1, speed,
                               rotation_array, 1, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar_Array(scenario_.tx(), scenario_.rx(),
                               frame_start_time, 1, location_array, 1, speed,
                               nullptr, 1, rotation_rate),
            nullptr);
}

TEST_F(RadarTest, CreateArrayRejectsNonPositiveSizes) {
  float location_array[3] = {0.0f, 0.0f, 0.0f};
  float rotation_array[3] = {0.0f, 0.0f, 0.0f};

  EXPECT_EQ(Create_Radar_Array(scenario_.tx(), scenario_.rx(),
                               frame_start_time, 0, location_array, 1, speed,
                               rotation_array, 1, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar_Array(scenario_.tx(), scenario_.rx(),
                               frame_start_time, 1, location_array, 0, speed,
                               rotation_array, 1, rotation_rate),
            nullptr);
  EXPECT_EQ(Create_Radar_Array(scenario_.tx(), scenario_.rx(),
                               frame_start_time, 1, location_array, 1, speed,
                               rotation_array, 0, rotation_rate),
            nullptr);
}

/*********************************************
 *
 *  Get_BB_Size
 *
 *********************************************/

TEST_F(RadarTest, BasebandSizeIsNullSafe) {
  EXPECT_EQ(Get_BB_Size(nullptr), 0);
}

/**
 * @brief Baseband size equals frames * virtual channels * pulses * samples
 */
TEST_F(RadarTest, BasebandSizeMatchesDimensions) {
  const int channels = scenario_.VirtualChannelSize();
  const int samples = scenario_.SampleSize();
  ASSERT_GT(channels, 0);
  ASSERT_GT(samples, 0);

  for (int frames = 1; frames <= 2; frames++) {
    RadarPtr radar(Create_Radar(scenario_.tx(), scenario_.rx(),
                                frame_start_time, frames, location, speed,
                                rotation, rotation_rate));
    ASSERT_NE(radar, nullptr) << "frames = " << frames;
    EXPECT_EQ(Get_BB_Size(radar.get()),
              frames * channels * scenario_.num_pulses * samples)
        << "frames = " << frames;
  }
}

/**
 * @brief More pulses scale the baseband buffer proportionally
 */
TEST(RadarSizingTest, BasebandSizeScalesWithPulseCount) {
  RadarScenario single;
  single.num_pulses = 1;
  ASSERT_TRUE(single.Build());

  RadarScenario multi;
  multi.num_pulses = 4;
  ASSERT_TRUE(multi.Build());

  EXPECT_EQ(multi.BasebandSize(), 4 * single.BasebandSize());
}

/*********************************************
 *
 *  Lifetime
 *
 *********************************************/

TEST_F(RadarTest, FreeIsNullSafe) {
  Free_Radar(nullptr);  // must not crash
}

/**
 * @brief Free_Radar does not free the transmitter or receiver
 *
 * @details The header states tx/rx are managed separately; after releasing the
 * radar they must remain usable for another radar.
 */
TEST_F(RadarTest, FreeRadarLeavesTxRxUsable) {
  t_Radar* radar = Create_Radar(scenario_.tx(), scenario_.rx(),
                                frame_start_time, 1, location, speed, rotation,
                                rotation_rate);
  ASSERT_NE(radar, nullptr);
  Free_Radar(radar);

  EXPECT_EQ(Get_Num_Txchannel(scenario_.tx()), 1);
  EXPECT_EQ(Get_Num_Rxchannel(scenario_.rx()), 1);

  RadarPtr rebuilt(Create_Radar(scenario_.tx(), scenario_.rx(),
                                frame_start_time, 1, location, speed, rotation,
                                rotation_rate));
  EXPECT_NE(rebuilt, nullptr);
}

/**
 * @brief Two radars can share the same transmitter/receiver pair
 */
TEST_F(RadarTest, MultipleRadarsShareTxRx) {
  RadarPtr first(Create_Radar(scenario_.tx(), scenario_.rx(),
                              frame_start_time, 1, location, speed, rotation,
                              rotation_rate));
  RadarPtr second(Create_Radar(scenario_.tx(), scenario_.rx(),
                               frame_start_time, 1, location, speed, rotation,
                               rotation_rate));

  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(Get_BB_Size(first.get()), Get_BB_Size(second.get()));
}

}  // namespace
