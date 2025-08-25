/*
 * @file test_c_wrapper_basic.cpp
 * @brief Basic unit tests for RadarSim C wrapper API functions
 *
 * @details
 * Basic test coverage for the C wrapper interface focusing on:
 * - Function call validation and parameter checking
 * - Null pointer handling
 * - Error condition testing
 * - Memory management validation
 * - Return value verification
 *
 * This test file focuses on testing the C wrapper functions directly
 * without complex radar simulation scenarios.
 *
 * Test Organization:
 * 1. Version API Tests
 * 2. Transmitter Basic Tests
 * 3. Receiver Basic Tests
 * 4. Radar Creation Tests
 * 5. Target Management Tests
 * 6. Error Handling Tests
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

#include <memory>
#include <vector>

// Include the C wrapper header
extern "C" {
#include "radarsim.h"
}

// =============================================================================
// TEST CONSTANTS
// =============================================================================

namespace {

// Basic test parameters
constexpr double kTestFreq = 24.125e9;  // 24.125 GHz
constexpr double kTestPulseWidth = 40e-6;  // 40 µs  
constexpr float kTestPower = 10.0f;     // 10 dBm
constexpr float kTestFs = 40e6f;        // 40 MHz
constexpr float kTestResistor = 50.0f;  // 50 Ohm

}  // namespace

// =============================================================================
// VERSION TESTS
// =============================================================================

TEST(CWrapperVersionTest, GetVersionReturnsExpectedValues) {
  int version[3] = {0, 0, 0};
  
  Get_Version(version);
  
  // Verify version matches header constants
  EXPECT_EQ(version[0], VERSION_MAJOR);
  EXPECT_EQ(version[1], VERSION_MINOR);
  EXPECT_EQ(version[2], VERSION_PATCH);
}

TEST(CWrapperVersionTest, GetVersionWithNullPointerDoesNotCrash) {
  // Should handle null gracefully
  EXPECT_NO_FATAL_FAILURE(Get_Version(nullptr));
}

// =============================================================================
// TRANSMITTER TESTS
// =============================================================================

class CWrapperTransmitterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup basic test vectors
    freq_vec_ = {kTestFreq, kTestFreq};
    time_vec_ = {0.0, kTestPulseWidth};
    freq_offset_ = {0.0, 0.0};
    pulse_times_ = {0.0, 100e-6};
    
    // Basic antenna pattern
    phi_ = {-1.57f, 0.0f, 1.57f};
    phi_pattern_ = {-20.0f, 0.0f, -20.0f};
    theta_ = {0.0f, 1.57f, 3.14f};
    theta_pattern_ = {0.0f, -3.0f, -20.0f};
    
    // Channel parameters
    location_ = {0.0f, 0.0f, 0.0f};
    polar_real_ = {1.0f, 0.0f, 0.0f};
    polar_imag_ = {0.0f, 0.0f, 0.0f};
    mod_t_ = {0.0f, kTestPulseWidth};
    mod_real_ = {1.0f, 1.0f};
    mod_imag_ = {0.0f, 0.0f};
    pulse_mod_real_ = {1.0f, 1.0f};
    pulse_mod_imag_ = {0.0f, 0.0f};
  }
  
  void TearDown() override {
    if (transmitter_) {
      Free_Transmitter(transmitter_);
    }
  }

 protected:
  t_Transmitter* transmitter_ = nullptr;
  std::vector<double> freq_vec_, time_vec_, freq_offset_, pulse_times_;
  std::vector<float> phi_, phi_pattern_, theta_, theta_pattern_;
  std::vector<float> location_, polar_real_, polar_imag_;
  std::vector<float> mod_t_, mod_real_, mod_imag_;
  std::vector<float> pulse_mod_real_, pulse_mod_imag_;
};

TEST_F(CWrapperTransmitterTest, CreateTransmitterWithValidParametersSucceeds) {
  transmitter_ = Create_Transmitter(
      freq_vec_.data(),
      time_vec_.data(),
      static_cast<int>(freq_vec_.size()),
      freq_offset_.data(),
      pulse_times_.data(),
      2,  // num_pulses
      kTestPower
  );
  
  EXPECT_NE(transmitter_, nullptr);
  EXPECT_EQ(Get_Num_Txchannel(transmitter_), 0);
}

TEST_F(CWrapperTransmitterTest, CreateTransmitterWithNullFrequencyReturnsNull) {
  transmitter_ = Create_Transmitter(
      nullptr,  // null frequency
      time_vec_.data(),
      static_cast<int>(freq_vec_.size()),
      freq_offset_.data(),
      pulse_times_.data(),
      2,
      kTestPower
  );
  
  EXPECT_EQ(transmitter_, nullptr);
}

TEST_F(CWrapperTransmitterTest, CreateTransmitterWithZeroWaveformSizeReturnsNull) {
  transmitter_ = Create_Transmitter(
      freq_vec_.data(),
      time_vec_.data(),
      0,  // invalid size
      freq_offset_.data(),
      pulse_times_.data(),
      2,
      kTestPower
  );
  
  EXPECT_EQ(transmitter_, nullptr);
}

TEST_F(CWrapperTransmitterTest, CreateTransmitterWithZeroPulsesReturnsNull) {
  transmitter_ = Create_Transmitter(
      freq_vec_.data(),
      time_vec_.data(),
      static_cast<int>(freq_vec_.size()),
      freq_offset_.data(),
      pulse_times_.data(),
      0,  // invalid pulse count
      kTestPower
  );
  
  EXPECT_EQ(transmitter_, nullptr);
}

TEST_F(CWrapperTransmitterTest, AddTxChannelWithValidParametersSucceeds) {
  transmitter_ = Create_Transmitter(
      freq_vec_.data(), time_vec_.data(), static_cast<int>(freq_vec_.size()),
      freq_offset_.data(), pulse_times_.data(), 2, kTestPower
  );
  ASSERT_NE(transmitter_, nullptr);
  
  int result = Add_Txchannel(
      location_.data(),
      polar_real_.data(),
      polar_imag_.data(),
      phi_.data(),
      phi_pattern_.data(),
      static_cast<int>(phi_.size()),
      theta_.data(),
      theta_pattern_.data(),
      static_cast<int>(theta_.size()),
      0.0f,  // antenna_gain
      mod_t_.data(),
      mod_real_.data(),
      mod_imag_.data(),
      static_cast<int>(mod_t_.size()),
      pulse_mod_real_.data(),
      pulse_mod_imag_.data(),
      0.0f,  // delay
      0.01f, // grid
      transmitter_
  );
  
  EXPECT_EQ(result, 0);  // Success
  EXPECT_EQ(Get_Num_Txchannel(transmitter_), 1);
}

TEST_F(CWrapperTransmitterTest, AddTxChannelWithNullTransmitterReturnsFails) {
  int result = Add_Txchannel(
      location_.data(), polar_real_.data(), polar_imag_.data(),
      phi_.data(), phi_pattern_.data(), static_cast<int>(phi_.size()),
      theta_.data(), theta_pattern_.data(), static_cast<int>(theta_.size()),
      0.0f, mod_t_.data(), mod_real_.data(), mod_imag_.data(),
      static_cast<int>(mod_t_.size()), pulse_mod_real_.data(),
      pulse_mod_imag_.data(), 0.0f, 0.01f,
      nullptr  // null transmitter
  );
  
  EXPECT_EQ(result, 1);  // Failure
}

TEST_F(CWrapperTransmitterTest, GetNumTxChannelWithNullTransmitterReturnsZero) {
  int count = Get_Num_Txchannel(nullptr);
  EXPECT_EQ(count, 0);
}

TEST_F(CWrapperTransmitterTest, FreeTransmitterWithNullPointerDoesNotCrash) {
  EXPECT_NO_FATAL_FAILURE(Free_Transmitter(nullptr));
}

// =============================================================================
// RECEIVER TESTS
// =============================================================================

class CWrapperReceiverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Basic antenna pattern
    phi_ = {-1.57f, 0.0f, 1.57f};
    phi_pattern_ = {-20.0f, 0.0f, -20.0f};
    theta_ = {0.0f, 1.57f, 3.14f};
    theta_pattern_ = {0.0f, -3.0f, -20.0f};
    
    // Channel parameters
    location_ = {0.0f, 0.0f, 0.0f};
    polar_real_ = {1.0f, 0.0f, 0.0f};
    polar_imag_ = {0.0f, 0.0f, 0.0f};
  }
  
  void TearDown() override {
    if (receiver_) {
      Free_Receiver(receiver_);
    }
  }

 protected:
  t_Receiver* receiver_ = nullptr;
  std::vector<float> phi_, phi_pattern_, theta_, theta_pattern_;
  std::vector<float> location_, polar_real_, polar_imag_;
};

TEST_F(CWrapperReceiverTest, CreateReceiverWithValidParametersSucceeds) {
  receiver_ = Create_Receiver(
      kTestFs,        // fs
      0.0f,           // rf_gain
      kTestResistor,  // resistor
      0.0f,           // baseband_gain
      20e6f           // baseband_bw
  );
  
  EXPECT_NE(receiver_, nullptr);
  EXPECT_EQ(Get_Num_Rxchannel(receiver_), 0);
}

TEST_F(CWrapperReceiverTest, CreateReceiverWithZeroFsReturnsNull) {
  receiver_ = Create_Receiver(
      0.0f,           // invalid fs
      0.0f,
      kTestResistor,
      0.0f,
      20e6f
  );
  
  EXPECT_EQ(receiver_, nullptr);
}

TEST_F(CWrapperReceiverTest, CreateReceiverWithZeroResistorReturnsNull) {
  receiver_ = Create_Receiver(
      kTestFs,
      0.0f,
      0.0f,           // invalid resistor
      0.0f,
      20e6f
  );
  
  EXPECT_EQ(receiver_, nullptr);
}

TEST_F(CWrapperReceiverTest, AddRxChannelWithValidParametersSucceeds) {
  receiver_ = Create_Receiver(kTestFs, 0.0f, kTestResistor, 0.0f, 20e6f);
  ASSERT_NE(receiver_, nullptr);
  
  int result = Add_Rxchannel(
      location_.data(),
      polar_real_.data(),
      polar_imag_.data(),
      phi_.data(),
      phi_pattern_.data(),
      static_cast<int>(phi_.size()),
      theta_.data(),
      theta_pattern_.data(),
      static_cast<int>(theta_.size()),
      0.0f,  // antenna_gain
      receiver_
  );
  
  EXPECT_EQ(result, 0);  // Success
  EXPECT_EQ(Get_Num_Rxchannel(receiver_), 1);
}

TEST_F(CWrapperReceiverTest, AddRxChannelWithNullReceiverFails) {
  int result = Add_Rxchannel(
      location_.data(), polar_real_.data(), polar_imag_.data(),
      phi_.data(), phi_pattern_.data(), static_cast<int>(phi_.size()),
      theta_.data(), theta_pattern_.data(), static_cast<int>(theta_.size()),
      0.0f,
      nullptr  // null receiver
  );
  
  EXPECT_EQ(result, 1);  // Failure
}

TEST_F(CWrapperReceiverTest, GetNumRxChannelWithNullReceiverReturnsZero) {
  int count = Get_Num_Rxchannel(nullptr);
  EXPECT_EQ(count, 0);
}

TEST_F(CWrapperReceiverTest, FreeReceiverWithNullPointerDoesNotCrash) {
  EXPECT_NO_FATAL_FAILURE(Free_Receiver(nullptr));
}

// =============================================================================
// RADAR SYSTEM TESTS
// =============================================================================

class CWrapperRadarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create transmitter
    std::vector<double> freq = {kTestFreq, kTestFreq};
    std::vector<double> time = {0.0, kTestPulseWidth};
    std::vector<double> freq_offset = {0.0, 0.0};
    std::vector<double> pulse_times = {0.0, 100e-6};
    
    transmitter_ = Create_Transmitter(
        freq.data(), time.data(), 2, freq_offset.data(),
        pulse_times.data(), 2, kTestPower
    );
    
    // Create receiver
    receiver_ = Create_Receiver(kTestFs, 0.0f, kTestResistor, 0.0f, 20e6f);
    
    // Platform parameters
    frame_times_ = {0.0, 1e-3};
    location_ = {0.0f, 0.0f, 0.0f};
    speed_ = {0.0f, 0.0f, 0.0f};
    rotation_ = {0.0f, 0.0f, 0.0f};
    rotation_rate_ = {0.0f, 0.0f, 0.0f};
  }
  
  void TearDown() override {
    if (radar_) Free_Radar(radar_);
    if (transmitter_) Free_Transmitter(transmitter_);
    if (receiver_) Free_Receiver(receiver_);
  }

 protected:
  t_Transmitter* transmitter_ = nullptr;
  t_Receiver* receiver_ = nullptr;
  t_Radar* radar_ = nullptr;
  std::vector<double> frame_times_;
  std::vector<float> location_, speed_, rotation_, rotation_rate_;
};

TEST_F(CWrapperRadarTest, CreateRadarWithValidParametersSucceeds) {
  ASSERT_NE(transmitter_, nullptr);
  ASSERT_NE(receiver_, nullptr);
  
  radar_ = Create_Radar(
      transmitter_,
      receiver_,
      frame_times_.data(),
      2,  // num_frames
      location_.data(),
      speed_.data(),
      rotation_.data(),
      rotation_rate_.data()
  );
  
  EXPECT_NE(radar_, nullptr);
}

TEST_F(CWrapperRadarTest, CreateRadarWithNullTransmitterReturnsNull) {
  ASSERT_NE(receiver_, nullptr);
  
  radar_ = Create_Radar(
      nullptr,  // null transmitter
      receiver_,
      frame_times_.data(),
      2,
      location_.data(),
      speed_.data(),
      rotation_.data(),
      rotation_rate_.data()
  );
  
  EXPECT_EQ(radar_, nullptr);
}

TEST_F(CWrapperRadarTest, CreateRadarWithNullReceiverReturnsNull) {
  ASSERT_NE(transmitter_, nullptr);
  
  radar_ = Create_Radar(
      transmitter_,
      nullptr,  // null receiver
      frame_times_.data(),
      2,
      location_.data(),
      speed_.data(),
      rotation_.data(),
      rotation_rate_.data()
  );
  
  EXPECT_EQ(radar_, nullptr);
}

TEST_F(CWrapperRadarTest, CreateRadarWithZeroFramesReturnsNull) {
  ASSERT_NE(transmitter_, nullptr);
  ASSERT_NE(receiver_, nullptr);
  
  radar_ = Create_Radar(
      transmitter_,
      receiver_,
      frame_times_.data(),
      0,  // invalid frame count
      location_.data(),
      speed_.data(),
      rotation_.data(),
      rotation_rate_.data()
  );
  
  EXPECT_EQ(radar_, nullptr);
}

TEST_F(CWrapperRadarTest, FreeRadarWithNullPointerDoesNotCrash) {
  EXPECT_NO_FATAL_FAILURE(Free_Radar(nullptr));
}

// =============================================================================
// TARGET MANAGEMENT TESTS
// =============================================================================

class CWrapperTargetsTest : public ::testing::Test {
 protected:
  void TearDown() override {
    if (targets_) {
      Free_Targets(targets_);
    }
  }

 protected:
  t_Targets* targets_ = nullptr;
};

TEST_F(CWrapperTargetsTest, InitTargetsSucceeds) {
  targets_ = Init_Targets();
  EXPECT_NE(targets_, nullptr);
}

TEST_F(CWrapperTargetsTest, AddPointTargetWithValidParametersSucceeds) {
  targets_ = Init_Targets();
  ASSERT_NE(targets_, nullptr);
  
  std::vector<float> location = {10.0f, 0.0f, 0.0f};
  std::vector<float> speed = {0.0f, 0.0f, 0.0f};
  
  int result = Add_Point_Target(
      location.data(),
      speed.data(),
      10.0f,  // rcs (dBsm)
      0.0f,   // phase (rad)
      targets_
  );
  
  EXPECT_EQ(result, 0);  // Success
}

TEST_F(CWrapperTargetsTest, AddPointTargetWithNullTargetsFails) {
  std::vector<float> location = {10.0f, 0.0f, 0.0f};
  std::vector<float> speed = {0.0f, 0.0f, 0.0f};
  
  int result = Add_Point_Target(
      location.data(),
      speed.data(),
      10.0f,
      0.0f,
      nullptr  // null targets
  );
  
  EXPECT_EQ(result, 1);  // Failure
}

TEST_F(CWrapperTargetsTest, AddPointTargetWithNullLocationFails) {
  targets_ = Init_Targets();
  ASSERT_NE(targets_, nullptr);
  
  std::vector<float> speed = {0.0f, 0.0f, 0.0f};
  
  int result = Add_Point_Target(
      nullptr,  // null location
      speed.data(),
      10.0f,
      0.0f,
      targets_
  );
  
  EXPECT_EQ(result, 1);  // Failure
}

TEST_F(CWrapperTargetsTest, AddMeshTargetWithValidParametersSucceeds) {
  targets_ = Init_Targets();
  ASSERT_NE(targets_, nullptr);
  
  // Simple triangle mesh
  std::vector<float> points = {
      0.0f, 0.0f, 0.0f,    // vertex 0
      1.0f, 0.0f, 0.0f,    // vertex 1
      0.5f, 1.0f, 0.0f     // vertex 2
  };
  std::vector<int> cells = {0, 1, 2};  // single triangle
  std::vector<float> origin = {0.0f, 0.0f, 0.0f};
  std::vector<float> location = {10.0f, 0.0f, 0.0f};
  std::vector<float> speed = {0.0f, 0.0f, 0.0f};
  std::vector<float> rotation = {0.0f, 0.0f, 0.0f};
  std::vector<float> rotation_rate = {0.0f, 0.0f, 0.0f};
  
  int result = Add_Mesh_Target(
      points.data(),
      cells.data(),
      1,  // one triangle
      origin.data(),
      location.data(),
      speed.data(),
      rotation.data(),
      rotation_rate.data(),
      1.0f,  // ep_real
      0.0f,  // ep_imag
      1.0f,  // mu_real
      0.0f,  // mu_imag
      false, // is_ground
      targets_
  );
  
  EXPECT_EQ(result, 0);  // Success
}

TEST_F(CWrapperTargetsTest, AddMeshTargetWithNullTargetsFails) {
  std::vector<float> points = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.5f, 1.0f, 0.0f};
  std::vector<int> cells = {0, 1, 2};
  std::vector<float> origin = {0.0f, 0.0f, 0.0f};
  std::vector<float> location = {10.0f, 0.0f, 0.0f};
  std::vector<float> speed = {0.0f, 0.0f, 0.0f};
  std::vector<float> rotation = {0.0f, 0.0f, 0.0f};
  std::vector<float> rotation_rate = {0.0f, 0.0f, 0.0f};
  
  int result = Add_Mesh_Target(
      points.data(), cells.data(), 1, origin.data(),
      location.data(), speed.data(), rotation.data(),
      rotation_rate.data(), 1.0f, 0.0f, 1.0f, 0.0f, false,
      nullptr  // null targets
  );
  
  EXPECT_EQ(result, 1);  // Failure
}

TEST_F(CWrapperTargetsTest, CompleteTargetsInitializationDoesNotCrash) {
  targets_ = Init_Targets();
  ASSERT_NE(targets_, nullptr);
  
  // Add a target first
  std::vector<float> location = {10.0f, 0.0f, 0.0f};
  std::vector<float> speed = {0.0f, 0.0f, 0.0f};
  Add_Point_Target(location.data(), speed.data(), 10.0f, 0.0f, targets_);
  
  EXPECT_NO_FATAL_FAILURE(Complete_Targets_Initialization(targets_));
}

TEST_F(CWrapperTargetsTest, CompleteTargetsInitializationWithNullDoesNotCrash) {
  EXPECT_NO_FATAL_FAILURE(Complete_Targets_Initialization(nullptr));
}

TEST_F(CWrapperTargetsTest, FreeTargetsWithNullPointerDoesNotCrash) {
  EXPECT_NO_FATAL_FAILURE(Free_Targets(nullptr));
}

// =============================================================================
// SIMULATION EXECUTION TESTS
// =============================================================================

class CWrapperSimulationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create minimal radar system for simulation tests
    std::vector<double> freq = {kTestFreq, kTestFreq};
    std::vector<double> time = {0.0, kTestPulseWidth};
    std::vector<double> freq_offset = {0.0, 0.0};
    std::vector<double> pulse_times = {0.0, 100e-6};
    
    transmitter_ = Create_Transmitter(
        freq.data(), time.data(), 2, freq_offset.data(),
        pulse_times.data(), 2, kTestPower
    );
    
    receiver_ = Create_Receiver(kTestFs, 0.0f, kTestResistor, 0.0f, 20e6f);
    
    std::vector<double> frame_times = {0.0, 1e-3};
    std::vector<float> location = {0.0f, 0.0f, 0.0f};
    std::vector<float> speed = {0.0f, 0.0f, 0.0f};
    std::vector<float> rotation = {0.0f, 0.0f, 0.0f};
    std::vector<float> rotation_rate = {0.0f, 0.0f, 0.0f};
    
    if (transmitter_ && receiver_) {
      radar_ = Create_Radar(
          transmitter_, receiver_, frame_times.data(), 2,
          location.data(), speed.data(), rotation.data(), rotation_rate.data()
      );
    }
    
    targets_ = Init_Targets();
    if (targets_) {
      std::vector<float> target_location = {10.0f, 0.0f, 0.0f};
      std::vector<float> target_speed = {0.0f, 0.0f, 0.0f};
      Add_Point_Target(target_location.data(), target_speed.data(), 10.0f, 0.0f, targets_);
      Complete_Targets_Initialization(targets_);
    }
    
    // Prepare output buffers
    int num_samples = static_cast<int>(kTestFs * kTestPulseWidth);
    output_size_ = 2 * 2 * num_samples;  // frames * pulses * samples
    bb_real_.resize(output_size_, 0.0);
    bb_imag_.resize(output_size_, 0.0);
  }
  
  void TearDown() override {
    if (targets_) Free_Targets(targets_);
    if (radar_) Free_Radar(radar_);
    if (transmitter_) Free_Transmitter(transmitter_);
    if (receiver_) Free_Receiver(receiver_);
  }

 protected:
  t_Transmitter* transmitter_ = nullptr;
  t_Receiver* receiver_ = nullptr;
  t_Radar* radar_ = nullptr;
  t_Targets* targets_ = nullptr;
  std::vector<double> bb_real_, bb_imag_;
  int output_size_ = 0;
};

TEST_F(CWrapperSimulationTest, RunRadarSimulatorWithValidParametersDoesNotCrash) {
  if (!radar_ || !targets_) {
    GTEST_SKIP() << "Radar system or targets not created properly";
  }
  
  int level = 1;
  float density = 1.0f;
  std::vector<int> ray_filter = {0, 0, 0};
  
  EXPECT_NO_FATAL_FAILURE(
      Run_RadarSimulator(
          radar_,
          targets_,
          level,
          density,
          ray_filter.data(),
          bb_real_.data(),
          bb_imag_.data()
      )
  );
}

TEST_F(CWrapperSimulationTest, RunRadarSimulatorWithNullRadarDoesNotCrash) {
  if (!targets_) {
    GTEST_SKIP() << "Targets not created properly";
  }
  
  int level = 1;
  float density = 1.0f;
  std::vector<int> ray_filter = {0, 0, 0};
  
  EXPECT_NO_FATAL_FAILURE(
      Run_RadarSimulator(
          nullptr,  // null radar
          targets_,
          level,
          density,
          ray_filter.data(),
          bb_real_.data(),
          bb_imag_.data()
      )
  );
}

TEST_F(CWrapperSimulationTest, RunRadarSimulatorWithNullTargetsDoesNotCrash) {
  if (!radar_) {
    GTEST_SKIP() << "Radar system not created properly";
  }
  
  int level = 1;
  float density = 1.0f;
  std::vector<int> ray_filter = {0, 0, 0};
  
  EXPECT_NO_FATAL_FAILURE(
      Run_RadarSimulator(
          radar_,
          nullptr,  // null targets
          level,
          density,
          ray_filter.data(),
          bb_real_.data(),
          bb_imag_.data()
      )
  );
}

TEST_F(CWrapperSimulationTest, RunInterferenceSimulatorDoesNotCrash) {
  if (!radar_) {
    GTEST_SKIP() << "Radar system not created properly";
  }
  
  // Use same radar as interference source for simplicity
  std::vector<double> interf_real(output_size_, 0.0);
  std::vector<double> interf_imag(output_size_, 0.0);
  
  EXPECT_NO_FATAL_FAILURE(
      Run_InterferenceSimulator(
          radar_,
          radar_,  // interference radar
          interf_real.data(),
          interf_imag.data()
      )
  );
}

TEST_F(CWrapperSimulationTest, RunRcsSimulatorReturnsValidStatus) {
  if (!targets_) {
    GTEST_SKIP() << "Targets not created properly";
  }
  
  std::vector<double> inc_dirs = {1.0, 0.0, 0.0};
  std::vector<double> obs_dirs = {-1.0, 0.0, 0.0};
  std::vector<double> inc_polar_real = {0.0, 1.0, 0.0};
  std::vector<double> inc_polar_imag = {0.0, 0.0, 0.0};
  std::vector<double> obs_polar_real = {0.0, 1.0, 0.0};
  std::vector<double> obs_polar_imag = {0.0, 0.0, 0.0};
  std::vector<double> rcs_result(1, 0.0);
  
  int result = Run_RcsSimulator(
      targets_,
      inc_dirs.data(),
      obs_dirs.data(),
      1,  // one direction pair
      inc_polar_real.data(),
      inc_polar_imag.data(),
      obs_polar_real.data(),
      obs_polar_imag.data(),
      kTestFreq,
      1.0,  // density
      rcs_result.data()
  );
  
  EXPECT_EQ(result, 0);  // Success expected
}

TEST_F(CWrapperSimulationTest, RunLidarSimulatorReturnsValidStatus) {
  if (!targets_) {
    GTEST_SKIP() << "Targets not created properly";
  }
  
  std::vector<double> phi_array = {0.0, 0.1};
  std::vector<double> theta_array = {1.57, 1.57};
  std::vector<double> sensor_location = {0.0, 0.0, 1.0};
  
  int max_points = 10;
  std::vector<double> cloud_points(max_points * 3, 0.0);
  std::vector<double> cloud_distances(max_points, 0.0);
  std::vector<double> cloud_intensities(max_points, 0.0);
  int actual_points = 0;
  
  int result = Run_LidarSimulator(
      targets_,
      phi_array.data(),
      theta_array.data(),
      static_cast<int>(phi_array.size()),
      sensor_location.data(),
      cloud_points.data(),
      cloud_distances.data(),
      cloud_intensities.data(),
      max_points,
      &actual_points
  );
  
  EXPECT_EQ(result, 0);  // Success expected
  EXPECT_GE(actual_points, 0);
  EXPECT_LE(actual_points, max_points);
}
