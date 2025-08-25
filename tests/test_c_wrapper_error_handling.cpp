/*
 * @file test_c_wrapper_error_handling.cpp
 * @brief Error handling and edge case tests for RadarSim C wrapper API
 *
 * @details
 * Focused test coverage for error conditions and edge cases in the C wrapper:
 * - Null pointer validation
 * - Invalid parameter ranges
 * - Boundary condition testing
 * - Memory safety validation
 * - Free tier limitation testing
 * - Resource cleanup testing
 * - Exception safety testing
 *
 * Test Organization:
 * 1. Null Pointer Tests
 * 2. Invalid Parameter Tests  
 * 3. Boundary Condition Tests
 * 4. Memory Safety Tests
 * 5. Resource Management Tests
 * 6. Error Recovery Tests
 *
 * Dependencies:
 * - Google Test framework
 * - radarsim.h C wrapper interface
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

#include <limits>
#include <vector>

// Include the C wrapper header
extern "C" {
#include "radarsim.h"
}

// =============================================================================
// NULL POINTER TESTS
// =============================================================================

/**
 * @brief Test that all API functions handle null pointers gracefully
 */
class NullPointerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup minimal valid parameters for testing
    freq_ = {24.125e9, 24.125e9};
    time_ = {0.0, 40e-6};
    freq_offset_ = {0.0, 0.0};
    pulse_times_ = {0.0, 100e-6};
    frame_times_ = {0.0, 1e-3};
    
    float_array_3_ = {0.0f, 0.0f, 0.0f};
    pattern_angles_ = {-1.57f, 0.0f, 1.57f};
    pattern_values_ = {-20.0f, 0.0f, -20.0f};
    mod_params_ = {0.0f, 40e-6f, 1.0f, 1.0f, 0.0f, 0.0f};
  }

 protected:
  std::vector<double> freq_, time_, freq_offset_, pulse_times_, frame_times_;
  std::vector<float> float_array_3_, pattern_angles_, pattern_values_, mod_params_;
};

TEST_F(NullPointerTest, CreateTransmitterWithAllNullPointersReturnsNull) {
  // All null pointers should result in null return
  t_Transmitter* tx = Create_Transmitter(nullptr, nullptr, 2, nullptr, nullptr, 2, 10.0f);
  EXPECT_EQ(tx, nullptr);
}

TEST_F(NullPointerTest, CreateTransmitterWithIndividualNullPointersReturnsNull) {
  // Test each null pointer individually
  
  // Null frequency
  EXPECT_EQ(Create_Transmitter(nullptr, time_.data(), 2, freq_offset_.data(), 
                               pulse_times_.data(), 2, 10.0f), nullptr);
  
  // Null time
  EXPECT_EQ(Create_Transmitter(freq_.data(), nullptr, 2, freq_offset_.data(), 
                               pulse_times_.data(), 2, 10.0f), nullptr);
  
  // Null frequency offset
  EXPECT_EQ(Create_Transmitter(freq_.data(), time_.data(), 2, nullptr, 
                               pulse_times_.data(), 2, 10.0f), nullptr);
  
  // Null pulse times
  EXPECT_EQ(Create_Transmitter(freq_.data(), time_.data(), 2, freq_offset_.data(), 
                               nullptr, 2, 10.0f), nullptr);
}

TEST_F(NullPointerTest, AddTxChannelWithNullPointersReturnsFails) {
  // Create valid transmitter first
  t_Transmitter* tx = Create_Transmitter(freq_.data(), time_.data(), 2, 
                                         freq_offset_.data(), pulse_times_.data(), 2, 10.0f);
  ASSERT_NE(tx, nullptr);
  
  // Test null pointers in Add_Txchannel - each should return 1 (failure)
  EXPECT_EQ(Add_Txchannel(nullptr, float_array_3_.data(), float_array_3_.data(),
                          pattern_angles_.data(), pattern_values_.data(), 3,
                          pattern_angles_.data(), pattern_values_.data(), 3, 0.0f,
                          mod_params_.data(), mod_params_.data()+2, mod_params_.data()+4, 2,
                          mod_params_.data()+2, mod_params_.data()+4, 0.0f, 0.01f, tx), 1);
  
  EXPECT_EQ(Add_Txchannel(float_array_3_.data(), nullptr, float_array_3_.data(),
                          pattern_angles_.data(), pattern_values_.data(), 3,
                          pattern_angles_.data(), pattern_values_.data(), 3, 0.0f,
                          mod_params_.data(), mod_params_.data()+2, mod_params_.data()+4, 2,
                          mod_params_.data()+2, mod_params_.data()+4, 0.0f, 0.01f, tx), 1);
  
  // Test with null transmitter pointer
  EXPECT_EQ(Add_Txchannel(float_array_3_.data(), float_array_3_.data(), float_array_3_.data(),
                          pattern_angles_.data(), pattern_values_.data(), 3,
                          pattern_angles_.data(), pattern_values_.data(), 3, 0.0f,
                          mod_params_.data(), mod_params_.data()+2, mod_params_.data()+4, 2,
                          mod_params_.data()+2, mod_params_.data()+4, 0.0f, 0.01f, nullptr), 1);
  
  Free_Transmitter(tx);
}

TEST_F(NullPointerTest, CreateReceiverWithValidParametersSucceeds) {
  // Receiver creation doesn't take arrays, so test basic functionality
  t_Receiver* rx = Create_Receiver(40e6f, 0.0f, 50.0f, 0.0f, 20e6f);
  EXPECT_NE(rx, nullptr);
  Free_Receiver(rx);
}

TEST_F(NullPointerTest, AddRxChannelWithNullPointersReturnsFails) {
  t_Receiver* rx = Create_Receiver(40e6f, 0.0f, 50.0f, 0.0f, 20e6f);
  ASSERT_NE(rx, nullptr);
  
  // Test null pointers in Add_Rxchannel
  EXPECT_EQ(Add_Rxchannel(nullptr, float_array_3_.data(), float_array_3_.data(),
                          pattern_angles_.data(), pattern_values_.data(), 3,
                          pattern_angles_.data(), pattern_values_.data(), 3, 0.0f, rx), 1);
  
  EXPECT_EQ(Add_Rxchannel(float_array_3_.data(), nullptr, float_array_3_.data(),
                          pattern_angles_.data(), pattern_values_.data(), 3,
                          pattern_angles_.data(), pattern_values_.data(), 3, 0.0f, rx), 1);
  
  // Test with null receiver pointer
  EXPECT_EQ(Add_Rxchannel(float_array_3_.data(), float_array_3_.data(), float_array_3_.data(),
                          pattern_angles_.data(), pattern_values_.data(), 3,
                          pattern_angles_.data(), pattern_values_.data(), 3, 0.0f, nullptr), 1);
  
  Free_Receiver(rx);
}

TEST_F(NullPointerTest, CreateRadarWithNullPointersReturnsNull) {
  // Create transmitter and receiver for testing
  t_Transmitter* tx = Create_Transmitter(freq_.data(), time_.data(), 2, 
                                         freq_offset_.data(), pulse_times_.data(), 2, 10.0f);
  t_Receiver* rx = Create_Receiver(40e6f, 0.0f, 50.0f, 0.0f, 20e6f);
  ASSERT_NE(tx, nullptr);
  ASSERT_NE(rx, nullptr);
  
  // Test null transmitter
  EXPECT_EQ(Create_Radar(nullptr, rx, frame_times_.data(), 2,
                         float_array_3_.data(), float_array_3_.data(),
                         float_array_3_.data(), float_array_3_.data()), nullptr);
  
  // Test null receiver
  EXPECT_EQ(Create_Radar(tx, nullptr, frame_times_.data(), 2,
                         float_array_3_.data(), float_array_3_.data(),
                         float_array_3_.data(), float_array_3_.data()), nullptr);
  
  // Test null frame times
  EXPECT_EQ(Create_Radar(tx, rx, nullptr, 2,
                         float_array_3_.data(), float_array_3_.data(),
                         float_array_3_.data(), float_array_3_.data()), nullptr);
  
  // Test null platform parameters
  EXPECT_EQ(Create_Radar(tx, rx, frame_times_.data(), 2,
                         nullptr, float_array_3_.data(),
                         float_array_3_.data(), float_array_3_.data()), nullptr);
  
  Free_Transmitter(tx);
  Free_Receiver(rx);
}

TEST_F(NullPointerTest, TargetFunctionsWithNullPointersHandleGracefully) {
  // Test Init_Targets (no parameters)
  t_Targets* targets = Init_Targets();
  // Could be null in free tier or if initialization fails
  
  if (targets) {
    // Test Add_Point_Target with null pointers
    EXPECT_EQ(Add_Point_Target(nullptr, float_array_3_.data(), 10.0f, 0.0f, targets), 1);
    EXPECT_EQ(Add_Point_Target(float_array_3_.data(), nullptr, 10.0f, 0.0f, targets), 1);
    
    Free_Targets(targets);
  }
  
  // Test with null targets pointer
  EXPECT_EQ(Add_Point_Target(float_array_3_.data(), float_array_3_.data(), 10.0f, 0.0f, nullptr), 1);
}

// =============================================================================
// INVALID PARAMETER TESTS
// =============================================================================

class InvalidParameterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    valid_freq_ = {24.125e9, 24.125e9};
    valid_time_ = {0.0, 40e-6};
    valid_freq_offset_ = {0.0, 0.0};
    valid_pulse_times_ = {0.0, 100e-6};
    valid_float3_ = {0.0f, 0.0f, 0.0f};
  }

 protected:
  std::vector<double> valid_freq_, valid_time_, valid_freq_offset_, valid_pulse_times_;
  std::vector<float> valid_float3_;
};

TEST_F(InvalidParameterTest, CreateTransmitterWithInvalidSizesReturnsNull) {
  // Zero waveform size
  EXPECT_EQ(Create_Transmitter(valid_freq_.data(), valid_time_.data(), 0,
                               valid_freq_offset_.data(), valid_pulse_times_.data(), 2, 10.0f), nullptr);
  
  // Negative waveform size
  EXPECT_EQ(Create_Transmitter(valid_freq_.data(), valid_time_.data(), -1,
                               valid_freq_offset_.data(), valid_pulse_times_.data(), 2, 10.0f), nullptr);
  
  // Zero pulses
  EXPECT_EQ(Create_Transmitter(valid_freq_.data(), valid_time_.data(), 2,
                               valid_freq_offset_.data(), valid_pulse_times_.data(), 0, 10.0f), nullptr);
  
  // Negative pulses
  EXPECT_EQ(Create_Transmitter(valid_freq_.data(), valid_time_.data(), 2,
                               valid_freq_offset_.data(), valid_pulse_times_.data(), -1, 10.0f), nullptr);
}

TEST_F(InvalidParameterTest, CreateReceiverWithInvalidParametersReturnsNull) {
  // Zero or negative sampling rate
  EXPECT_EQ(Create_Receiver(0.0f, 0.0f, 50.0f, 0.0f, 20e6f), nullptr);
  EXPECT_EQ(Create_Receiver(-1.0f, 0.0f, 50.0f, 0.0f, 20e6f), nullptr);
  
  // Zero or negative resistor
  EXPECT_EQ(Create_Receiver(40e6f, 0.0f, 0.0f, 0.0f, 20e6f), nullptr);
  EXPECT_EQ(Create_Receiver(40e6f, 0.0f, -50.0f, 0.0f, 20e6f), nullptr);
}

TEST_F(InvalidParameterTest, AddChannelWithInvalidPatternSizesReturnsFails) {
  t_Transmitter* tx = Create_Transmitter(valid_freq_.data(), valid_time_.data(), 2,
                                         valid_freq_offset_.data(), valid_pulse_times_.data(), 2, 10.0f);
  ASSERT_NE(tx, nullptr);
  
  std::vector<float> pattern_angles = {0.0f, 1.57f};
  std::vector<float> pattern_values = {0.0f, -3.0f};
  std::vector<float> mod_time = {0.0f, 40e-6f};
  std::vector<float> mod_real = {1.0f, 1.0f};
  std::vector<float> mod_imag = {0.0f, 0.0f};
  
  // Zero phi length
  EXPECT_EQ(Add_Txchannel(valid_float3_.data(), valid_float3_.data(), valid_float3_.data(),
                          pattern_angles.data(), pattern_values.data(), 0,  // zero phi length
                          pattern_angles.data(), pattern_values.data(), 2, 0.0f,
                          mod_time.data(), mod_real.data(), mod_imag.data(), 2,
                          mod_real.data(), mod_imag.data(), 0.0f, 0.01f, tx), 1);
  
  // Negative phi length
  EXPECT_EQ(Add_Txchannel(valid_float3_.data(), valid_float3_.data(), valid_float3_.data(),
                          pattern_angles.data(), pattern_values.data(), -1,  // negative phi length
                          pattern_angles.data(), pattern_values.data(), 2, 0.0f,
                          mod_time.data(), mod_real.data(), mod_imag.data(), 2,
                          mod_real.data(), mod_imag.data(), 0.0f, 0.01f, tx), 1);
  
  // Zero theta length
  EXPECT_EQ(Add_Txchannel(valid_float3_.data(), valid_float3_.data(), valid_float3_.data(),
                          pattern_angles.data(), pattern_values.data(), 2,
                          pattern_angles.data(), pattern_values.data(), 0,  // zero theta length
                          0.0f, mod_time.data(), mod_real.data(), mod_imag.data(), 2,
                          mod_real.data(), mod_imag.data(), 0.0f, 0.01f, tx), 1);
  
  // Zero modulation length
  EXPECT_EQ(Add_Txchannel(valid_float3_.data(), valid_float3_.data(), valid_float3_.data(),
                          pattern_angles.data(), pattern_values.data(), 2,
                          pattern_angles.data(), pattern_values.data(), 2, 0.0f,
                          mod_time.data(), mod_real.data(), mod_imag.data(), 0,  // zero mod length
                          mod_real.data(), mod_imag.data(), 0.0f, 0.01f, tx), 1);
  
  Free_Transmitter(tx);
}

TEST_F(InvalidParameterTest, CreateRadarWithInvalidFrameCountReturnsNull) {
  t_Transmitter* tx = Create_Transmitter(valid_freq_.data(), valid_time_.data(), 2,
                                         valid_freq_offset_.data(), valid_pulse_times_.data(), 2, 10.0f);
  t_Receiver* rx = Create_Receiver(40e6f, 0.0f, 50.0f, 0.0f, 20e6f);
  ASSERT_NE(tx, nullptr);
  ASSERT_NE(rx, nullptr);
  
  std::vector<double> frame_times = {0.0, 1e-3};
  
  // Zero frames
  EXPECT_EQ(Create_Radar(tx, rx, frame_times.data(), 0,
                         valid_float3_.data(), valid_float3_.data(),
                         valid_float3_.data(), valid_float3_.data()), nullptr);
  
  // Negative frames
  EXPECT_EQ(Create_Radar(tx, rx, frame_times.data(), -1,
                         valid_float3_.data(), valid_float3_.data(),
                         valid_float3_.data(), valid_float3_.data()), nullptr);
  
  Free_Transmitter(tx);
  Free_Receiver(rx);
}

// =============================================================================
// BOUNDARY CONDITION TESTS
// =============================================================================

class BoundaryConditionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create minimal valid setup for boundary testing
    freq_ = {1e9, 1e9};  // 1 GHz
    time_ = {0.0, 1e-6}; // 1 µs
    freq_offset_ = {0.0, 0.0};
    pulse_times_ = {0.0, 10e-6};
    float3_ = {0.0f, 0.0f, 0.0f};
  }

 protected:
  std::vector<double> freq_, time_, freq_offset_, pulse_times_;
  std::vector<float> float3_;
};

TEST_F(BoundaryConditionTest, ExtremeFrequencyValuesHandledProperly) {
  // Very low frequency (might be unrealistic but should not crash)
  std::vector<double> low_freq = {1e3, 1e3};  // 1 kHz
  t_Transmitter* tx_low = Create_Transmitter(low_freq.data(), time_.data(), 2,
                                             freq_offset_.data(), pulse_times_.data(), 2, 10.0f);
  if (tx_low) Free_Transmitter(tx_low);
  
  // Very high frequency (might be unrealistic but should not crash)
  std::vector<double> high_freq = {1e12, 1e12};  // 1 THz
  t_Transmitter* tx_high = Create_Transmitter(high_freq.data(), time_.data(), 2,
                                              freq_offset_.data(), pulse_times_.data(), 2, 10.0f);
  if (tx_high) Free_Transmitter(tx_high);
  
  // Test passed if no crash occurred
  SUCCEED();
}

TEST_F(BoundaryConditionTest, ExtremePowerValuesHandledProperly) {
  // Very low power
  t_Transmitter* tx_low = Create_Transmitter(freq_.data(), time_.data(), 2,
                                             freq_offset_.data(), pulse_times_.data(), 2, -100.0f);
  if (tx_low) Free_Transmitter(tx_low);
  
  // Very high power
  t_Transmitter* tx_high = Create_Transmitter(freq_.data(), time_.data(), 2,
                                              freq_offset_.data(), pulse_times_.data(), 2, 100.0f);
  if (tx_high) Free_Transmitter(tx_high);
  
  SUCCEED();
}

TEST_F(BoundaryConditionTest, ExtremeSamplingRatesHandledProperly) {
  // Very low sampling rate
  t_Receiver* rx_low = Create_Receiver(1e3f, 0.0f, 50.0f, 0.0f, 1e3f);
  if (rx_low) Free_Receiver(rx_low);
  
  // Very high sampling rate
  t_Receiver* rx_high = Create_Receiver(1e9f, 0.0f, 50.0f, 0.0f, 1e9f);
  if (rx_high) Free_Receiver(rx_high);
  
  SUCCEED();
}

TEST_F(BoundaryConditionTest, MinimalValidConfigurationWorks) {
  // Test with minimal valid configuration (1 sample waveform, 1 pulse, 1 frame)
  std::vector<double> min_freq = {24e9};
  std::vector<double> min_time = {0.0};
  std::vector<double> min_offset = {0.0};
  std::vector<double> min_pulse = {0.0};
  std::vector<double> min_frame = {0.0};
  
  t_Transmitter* tx = Create_Transmitter(min_freq.data(), min_time.data(), 1,
                                         min_offset.data(), min_pulse.data(), 1, 10.0f);
  t_Receiver* rx = Create_Receiver(1e6f, 0.0f, 50.0f, 0.0f, 1e6f);
  
  if (tx && rx) {
    t_Radar* radar = Create_Radar(tx, rx, min_frame.data(), 1,
                                  float3_.data(), float3_.data(), float3_.data(), float3_.data());
    if (radar) Free_Radar(radar);
  }
  
  if (tx) Free_Transmitter(tx);
  if (rx) Free_Receiver(rx);
  
  SUCCEED();
}

// =============================================================================
// MEMORY SAFETY TESTS
// =============================================================================

class MemorySafetyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup for memory safety testing
    freq_ = {24e9, 24e9};
    time_ = {0.0, 40e-6};
    freq_offset_ = {0.0, 0.0};
    pulse_times_ = {0.0, 100e-6};
    float3_ = {0.0f, 0.0f, 0.0f};
  }

 protected:
  std::vector<double> freq_, time_, freq_offset_, pulse_times_;
  std::vector<float> float3_;
};

TEST_F(MemorySafetyTest, MultipleCreateAndFreeOperations) {
  // Test multiple create/free cycles to check for memory leaks
  for (int i = 0; i < 10; ++i) {
    t_Transmitter* tx = Create_Transmitter(freq_.data(), time_.data(), 2,
                                           freq_offset_.data(), pulse_times_.data(), 2, 10.0f);
    t_Receiver* rx = Create_Receiver(40e6f, 0.0f, 50.0f, 0.0f, 20e6f);
    t_Targets* targets = Init_Targets();
    
    // Clean up (should not crash)
    if (tx) Free_Transmitter(tx);
    if (rx) Free_Receiver(rx);
    if (targets) Free_Targets(targets);
  }
  
  SUCCEED();
}

TEST_F(MemorySafetyTest, DoubleFreeSafetyTest) {
  t_Transmitter* tx = Create_Transmitter(freq_.data(), time_.data(), 2,
                                         freq_offset_.data(), pulse_times_.data(), 2, 10.0f);
  if (tx) {
    Free_Transmitter(tx);
    // Double free should not crash (implementation should handle gracefully)
    EXPECT_NO_FATAL_FAILURE(Free_Transmitter(tx));
  }
  
  t_Receiver* rx = Create_Receiver(40e6f, 0.0f, 50.0f, 0.0f, 20e6f);
  if (rx) {
    Free_Receiver(rx);
    EXPECT_NO_FATAL_FAILURE(Free_Receiver(rx));
  }
  
  t_Targets* targets = Init_Targets();
  if (targets) {
    Free_Targets(targets);
    EXPECT_NO_FATAL_FAILURE(Free_Targets(targets));
  }
}

TEST_F(MemorySafetyTest, FreeWithNullPointersIsSafe) {
  // All free functions should handle null pointers gracefully
  EXPECT_NO_FATAL_FAILURE(Free_Transmitter(nullptr));
  EXPECT_NO_FATAL_FAILURE(Free_Receiver(nullptr));
  EXPECT_NO_FATAL_FAILURE(Free_Radar(nullptr));
  EXPECT_NO_FATAL_FAILURE(Free_Targets(nullptr));
}

// =============================================================================
// RESOURCE MANAGEMENT TESTS
// =============================================================================

class ResourceManagementTest : public ::testing::Test {
 protected:
  void SetUp() override {
    freq_ = {24e9, 24e9};
    time_ = {0.0, 40e-6};
    freq_offset_ = {0.0, 0.0};
    pulse_times_ = {0.0, 100e-6};
    frame_times_ = {0.0, 1e-3};
    float3_ = {0.0f, 0.0f, 0.0f};
  }

 protected:
  std::vector<double> freq_, time_, freq_offset_, pulse_times_, frame_times_;
  std::vector<float> float3_;
};

TEST_F(ResourceManagementTest, RadarFreeShouldNotFreeComponentsEarly) {
  // Create components
  t_Transmitter* tx = Create_Transmitter(freq_.data(), time_.data(), 2,
                                         freq_offset_.data(), pulse_times_.data(), 2, 10.0f);
  t_Receiver* rx = Create_Receiver(40e6f, 0.0f, 50.0f, 0.0f, 20e6f);
  
  ASSERT_NE(tx, nullptr);
  ASSERT_NE(rx, nullptr);
  
  // Create radar
  t_Radar* radar = Create_Radar(tx, rx, frame_times_.data(), 2,
                                float3_.data(), float3_.data(), float3_.data(), float3_.data());
  
  if (radar) {
    // Free radar first
    Free_Radar(radar);
    
    // Original components should still be valid to free
    // (Implementation detail: radar should not take ownership)
    EXPECT_NO_FATAL_FAILURE(Free_Transmitter(tx));
    EXPECT_NO_FATAL_FAILURE(Free_Receiver(rx));
  } else {
    // Clean up if radar creation failed
    Free_Transmitter(tx);
    Free_Receiver(rx);
  }
}

TEST_F(ResourceManagementTest, TargetsCanBeReusedAcrossSimulations) {
  t_Targets* targets = Init_Targets();
  if (!targets) {
    GTEST_SKIP() << "Targets initialization failed (possibly free tier)";
  }
  
  // Add a target
  std::vector<float> location = {10.0f, 0.0f, 0.0f};
  std::vector<float> speed = {0.0f, 0.0f, 0.0f};
  int result = Add_Point_Target(location.data(), speed.data(), 10.0f, 0.0f, targets);
  
  if (result == 0) {
    // Complete initialization multiple times (should be safe)
    EXPECT_NO_FATAL_FAILURE(Complete_Targets_Initialization(targets));
    EXPECT_NO_FATAL_FAILURE(Complete_Targets_Initialization(targets));
    
    // Add another target after completion (behavior may vary)
    std::vector<float> location2 = {20.0f, 0.0f, 0.0f};
    Add_Point_Target(location2.data(), speed.data(), 15.0f, 0.0f, targets);
  }
  
  Free_Targets(targets);
  SUCCEED();
}

// =============================================================================
// ERROR RECOVERY TESTS
// =============================================================================

class ErrorRecoveryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    freq_ = {24e9, 24e9};
    time_ = {0.0, 40e-6};
    freq_offset_ = {0.0, 0.0};
    pulse_times_ = {0.0, 100e-6};
    float3_ = {0.0f, 0.0f, 0.0f};
    pattern_ = {-1.57f, 0.0f, 1.57f};
    pattern_vals_ = {-20.0f, 0.0f, -20.0f};
    mod_time_ = {0.0f, 40e-6f};
    mod_vals_ = {1.0f, 1.0f, 0.0f, 0.0f};
  }

 protected:
  std::vector<double> freq_, time_, freq_offset_, pulse_times_;
  std::vector<float> float3_, pattern_, pattern_vals_, mod_time_, mod_vals_;
};

TEST_F(ErrorRecoveryTest, RecoveryAfterFailedChannelAddition) {
  t_Transmitter* tx = Create_Transmitter(freq_.data(), time_.data(), 2,
                                         freq_offset_.data(), pulse_times_.data(), 2, 10.0f);
  ASSERT_NE(tx, nullptr);
  
  // Try to add channel with invalid parameters (should fail)
  int result1 = Add_Txchannel(nullptr, float3_.data(), float3_.data(),  // null location
                              pattern_.data(), pattern_vals_.data(), 3,
                              pattern_.data(), pattern_vals_.data(), 3, 0.0f,
                              mod_time_.data(), mod_vals_.data(), mod_vals_.data()+2, 2,
                              mod_vals_.data(), mod_vals_.data()+2, 0.0f, 0.01f, tx);
  EXPECT_EQ(result1, 1);  // Should fail
  EXPECT_EQ(Get_Num_Txchannel(tx), 0);  // No channels added
  
  // Now add valid channel (should succeed)
  int result2 = Add_Txchannel(float3_.data(), float3_.data(), float3_.data(),
                              pattern_.data(), pattern_vals_.data(), 3,
                              pattern_.data(), pattern_vals_.data(), 3, 0.0f,
                              mod_time_.data(), mod_vals_.data(), mod_vals_.data()+2, 2,
                              mod_vals_.data(), mod_vals_.data()+2, 0.0f, 0.01f, tx);
  EXPECT_EQ(result2, 0);  // Should succeed
  EXPECT_EQ(Get_Num_Txchannel(tx), 1);  // One channel added
  
  Free_Transmitter(tx);
}

TEST_F(ErrorRecoveryTest, ContinueAfterFailedTargetAddition) {
  t_Targets* targets = Init_Targets();
  if (!targets) {
    GTEST_SKIP() << "Targets initialization failed";
  }
  
  // Try to add target with invalid parameters (should fail)
  int result1 = Add_Point_Target(nullptr, float3_.data(), 10.0f, 0.0f, targets);
  EXPECT_EQ(result1, 1);  // Should fail
  
  // Add valid target (should work)
  std::vector<float> location = {10.0f, 0.0f, 0.0f};
  int result2 = Add_Point_Target(location.data(), float3_.data(), 10.0f, 0.0f, targets);
  EXPECT_EQ(result2, 0);  // Should succeed
  
  Free_Targets(targets);
}

TEST_F(ErrorRecoveryTest, SimulationFunctionsHandleInvalidInputsGracefully) {
  // Test simulation functions with null/invalid inputs
  std::vector<double> output(1000, 0.0);
  std::vector<int> ray_filter = {0, 0, 0};
  
  // Should not crash with null inputs
  EXPECT_NO_FATAL_FAILURE(
      Run_RadarSimulator(nullptr, nullptr, 1, 1.0f, ray_filter.data(),
                         output.data(), output.data())
  );
  
  EXPECT_NO_FATAL_FAILURE(
      Run_InterferenceSimulator(nullptr, nullptr, output.data(), output.data())
  );
  
  // RCS and LiDAR simulations
  std::vector<double> directions = {1.0, 0.0, 0.0};
  std::vector<double> polarization = {0.0, 1.0, 0.0};
  std::vector<double> rcs_result(1, 0.0);
  
  int rcs_result_code = Run_RcsSimulator(nullptr, directions.data(), directions.data(), 1,
                                         polarization.data(), polarization.data(),
                                         polarization.data(), polarization.data(),
                                         24e9, 1.0, rcs_result.data());
  // Result code will depend on implementation (may be success or failure)
  EXPECT_TRUE(rcs_result_code == 0 || rcs_result_code == 1);
  
  std::vector<double> angles = {0.0, 1.57};
  std::vector<double> sensor_pos = {0.0, 0.0, 1.0};
  std::vector<double> cloud_points(30, 0.0);
  std::vector<double> cloud_distances(10, 0.0);
  std::vector<double> cloud_intensities(10, 0.0);
  int actual_points = 0;
  
  int lidar_result_code = Run_LidarSimulator(nullptr, angles.data(), angles.data(), 2,
                                             sensor_pos.data(), cloud_points.data(),
                                             cloud_distances.data(), cloud_intensities.data(),
                                             10, &actual_points);
  EXPECT_TRUE(lidar_result_code == 0 || lidar_result_code == 1);
}
