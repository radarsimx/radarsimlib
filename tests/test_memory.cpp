/*
 * @file test_memory.cpp
 * @brief Unit tests for the automatic cleanup registry
 *
 * @details
 * The C API registers every object it hands out so that a client which forgets
 * to call Free_* still gets its memory back at unload. These tests exercise
 * that safety net deterministically via Force_Cleanup_All instead of relying on
 * process exit, and pin down the documented lifetime rules:
 * - Force_Cleanup_All releases objects that were never manually freed
 * - Manually freed objects are unregistered, so cleanup does not double-free
 * - Is_Cleanup_In_Progress is false outside of a cleanup pass
 *
 * @warning Force_Cleanup_All invalidates every live handle in the process, so
 * these tests must own no handles from other fixtures. They deliberately use
 * raw pointers rather than the RAII helpers.
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

namespace {

/**
 * @brief Creates unowned objects of every registered type
 *
 * @details Returned handles are intentionally not freed by the caller - they
 * exist to be reclaimed by Force_Cleanup_All.
 */
struct UnownedObjects {
  t_Transmitter* tx = nullptr;
  t_Receiver* rx = nullptr;
  t_Radar* radar = nullptr;
  t_Targets* targets = nullptr;
};

/** @brief Waveform storage kept alive for the duration of a test */
struct WaveformBuffers {
  std::vector<double> freq;
  std::vector<double> freq_time;
  std::vector<double> freq_offset;
  std::vector<double> pulse_start_time;
  std::vector<double> frame_start_time;
};

/**
 * @brief Allocate one object of every kind without taking ownership
 */
UnownedObjects CreateUnowned(WaveformBuffers& buffers) {
  constexpr int kNumSamples = 8;
  constexpr int kNumPulses = 1;

  buffers.freq.resize(kNumSamples);
  buffers.freq_time.resize(kNumSamples);
  for (int i = 0; i < kNumSamples; i++) {
    buffers.freq[i] = 24.0e9 + i * 3e6;
    buffers.freq_time[i] = i * 1e-6;
  }
  buffers.freq_offset.assign(kNumPulses, 0.0);
  buffers.pulse_start_time.assign(kNumPulses, 0.0);
  buffers.frame_start_time.assign(1, 0.0);

  UnownedObjects objects;
  objects.tx = Create_Transmitter(
      buffers.freq.data(), buffers.freq_time.data(), kNumSamples,
      buffers.freq_offset.data(), buffers.pulse_start_time.data(), kNumPulses,
      10.0f);
  objects.rx = Create_Receiver(1e6f, 30.0f, 50.0f, 20.0f, 500e3f, 0.0);
  objects.targets = Init_Targets();

  if (objects.tx != nullptr && objects.rx != nullptr) {
    float zero[3] = {0.0f, 0.0f, 0.0f};
    objects.radar =
        Create_Radar(objects.tx, objects.rx, buffers.frame_start_time.data(),
                     1, zero, zero, zero, zero);
  }
  return objects;
}

/**
 * @brief Cleanup-registry tests
 *
 * @details Each test leaves the registry empty so the next one starts clean.
 */
class CleanupRegistryTest : public ::testing::Test {
 protected:
  void TearDown() override {
    // Leave no registered objects behind for other suites.
    Force_Cleanup_All();
  }
};

TEST_F(CleanupRegistryTest, CleanupIsNotInProgressWhileIdle) {
  EXPECT_EQ(Is_Cleanup_In_Progress(), 0);
}

/**
 * @brief Objects that were never freed are reclaimed by Force_Cleanup_All
 */
TEST_F(CleanupRegistryTest, ForceCleanupReclaimsUnfreedObjects) {
  WaveformBuffers buffers;
  UnownedObjects objects = CreateUnowned(buffers);

  ASSERT_NE(objects.tx, nullptr);
  ASSERT_NE(objects.rx, nullptr);
  ASSERT_NE(objects.radar, nullptr);
  ASSERT_NE(objects.targets, nullptr);

  Force_Cleanup_All();

  EXPECT_EQ(Is_Cleanup_In_Progress(), 0)
      << "The in-progress flag must be cleared once cleanup returns";
}

/**
 * @brief Force_Cleanup_All is idempotent and safe on an empty registry
 */
TEST_F(CleanupRegistryTest, ForceCleanupIsIdempotent) {
  Force_Cleanup_All();
  Force_Cleanup_All();

  EXPECT_EQ(Is_Cleanup_In_Progress(), 0);
}

/**
 * @brief Manually freed objects are unregistered, so cleanup cannot re-free
 *
 * @details Without unregistration, the second pass would double-free and this
 * test would crash rather than fail.
 */
TEST_F(CleanupRegistryTest, ManualFreeUnregistersFromCleanup) {
  WaveformBuffers buffers;
  UnownedObjects objects = CreateUnowned(buffers);

  ASSERT_NE(objects.radar, nullptr);

  Free_Radar(objects.radar);
  Free_Receiver(objects.rx);
  Free_Transmitter(objects.tx);
  Free_Targets(objects.targets);

  Force_Cleanup_All();
  EXPECT_EQ(Is_Cleanup_In_Progress(), 0);
}

/**
 * @brief A partially freed set is handled correctly
 *
 * @details The radar is released manually while its tx/rx pair is left to the
 * registry, which is the mixed pattern the API documentation describes.
 */
TEST_F(CleanupRegistryTest, MixedManualAndAutomaticCleanup) {
  WaveformBuffers buffers;
  UnownedObjects objects = CreateUnowned(buffers);

  ASSERT_NE(objects.radar, nullptr);
  Free_Radar(objects.radar);

  // tx, rx and targets remain registered.
  Force_Cleanup_All();
  EXPECT_EQ(Is_Cleanup_In_Progress(), 0);
}

/**
 * @brief The API is usable again after a full cleanup pass
 */
TEST_F(CleanupRegistryTest, ApiRemainsUsableAfterCleanup) {
  WaveformBuffers buffers;
  UnownedObjects discarded = CreateUnowned(buffers);
  ASSERT_NE(discarded.radar, nullptr);

  Force_Cleanup_All();

  rstest::RadarScenario scenario;
  EXPECT_TRUE(scenario.Build());
  EXPECT_GT(scenario.BasebandSize(), 0);
}

}  // namespace
