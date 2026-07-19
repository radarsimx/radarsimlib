/*
 * @file test_scene.cpp
 * @brief Unit tests for Scene State Query C wrapper functions
 *
 * @details
 * Test scenarios:
 * - Get_Scene_State for static and moving radar platforms
 * - Get_Scene_State for time-varying (Create_Radar_Array) platforms
 * - Get_Num_Targets / Get_Target_Mesh_Size
 * - Get_Target_Mesh_State for constant-motion and time-varying targets
 * - Parameter validation
 * - Deep-copy safety (query does not mutate live target state)
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

#include <cmath>
#include <vector>

#include "radarsim.h"

#define kPI 3.141592653589793
#define kTolerance 1e-3f

/**
 * @brief Helper to create a minimal configured radar system for scene tests
 */
struct SceneRadarSetup {
  t_Transmitter* tx = nullptr;
  t_Receiver* rx = nullptr;
  t_Radar* radar = nullptr;

  std::vector<double> freq;
  std::vector<double> freq_time;
  std::vector<double> freq_offset;
  std::vector<double> pulse_start_time;
  std::vector<float> pulse_mod_real;
  std::vector<float> pulse_mod_imag;

  bool CreateTxRx(float tx_location[3], float rx_location[3]) {
    int num_samples = 8;
    int num_pulses = 1;

    freq.resize(num_samples);
    freq_time.resize(num_samples);
    for (int i = 0; i < num_samples; i++) {
      freq[i] = 24.0e9 + i * 3e6;
      freq_time[i] = i * 1e-6;
    }

    freq_offset.resize(num_pulses, 0.0);
    pulse_start_time.resize(num_pulses, 0.0);

    tx = Create_Transmitter(freq.data(), freq_time.data(), num_samples,
                            freq_offset.data(), pulse_start_time.data(),
                            num_pulses, 10.0f);
    if (!tx) return false;

    float polar_real[3] = {1.0f, 0.0f, 0.0f};
    float polar_imag[3] = {0.0f, 0.0f, 0.0f};
    float phi[2] = {-static_cast<float>(kPI) / 2.0f,
                    static_cast<float>(kPI) / 2.0f};
    float phi_ptn[2] = {0.0f, 0.0f};
    float theta[2] = {0.0f, static_cast<float>(kPI)};
    float theta_ptn[2] = {0.0f, 0.0f};

    pulse_mod_real.resize(num_pulses, 1.0f);
    pulse_mod_imag.resize(num_pulses, 0.0f);

    int result = Add_Txchannel(tx_location, polar_real, polar_imag, phi,
                               phi_ptn, 2, theta, theta_ptn, 2, 20.0f,
                               nullptr, nullptr, nullptr, 0,
                               pulse_mod_real.data(), pulse_mod_imag.data(),
                               0.0f, 0.1f, tx);
    if (result != 0) return false;

    rx = Create_Receiver(1e6f, 30.0f, 50.0f, 20.0f, 500e3f);
    if (!rx) return false;

    result = Add_Rxchannel(rx_location, polar_real, polar_imag, phi, phi_ptn,
                           2, theta, theta_ptn, 2, 20.0f, rx);
    if (result != 0) return false;

    return true;
  }

  bool CreateStaticRadar(float location[3], float speed[3], float rotation[3],
                         float rotation_rate[3]) {
    double frame_start_time[1] = {0.0};
    radar = Create_Radar(tx, rx, frame_start_time, 1, location, speed,
                         rotation, rotation_rate);
    return radar != nullptr;
  }

  bool CreateArrayRadar(double* frame_start_time, int num_frames,
                        float* location_array, int num_locations,
                        float* speed, float* rotation_array,
                        int num_rotations, float* rotation_rate) {
    radar = Create_Radar_Array(tx, rx, frame_start_time, num_frames,
                               location_array, num_locations, speed,
                               rotation_array, num_rotations, rotation_rate);
    return radar != nullptr;
  }

  void Destroy() {
    if (radar) {
      Free_Radar(radar);
      radar = nullptr;
    }
    if (rx) {
      Free_Receiver(rx);
      rx = nullptr;
    }
    if (tx) {
      Free_Transmitter(tx);
      tx = nullptr;
    }
  }
};

class SceneStateTest : public ::testing::Test {
 protected:
  void TearDown() override { setup_.Destroy(); }

  SceneRadarSetup setup_;
};

/**
 * @brief Static radar at the origin, zero rotation - channel globals should
 * equal the local channel offsets, boresight should be +X.
 */
TEST_F(SceneStateTest, StaticRadarOrigin) {
  float tx_loc[3] = {1.0f, 0.0f, 0.0f};
  float rx_loc[3] = {-1.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateTxRx(tx_loc, rx_loc));

  float loc[3] = {0.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateStaticRadar(loc, speed, rotation, rotation_rate));

  double timestamps[1] = {0.0};
  float tx_out[3], rx_out[3], bore_out[3];

  int result = Get_Scene_State(setup_.radar, timestamps, 1, tx_out, rx_out,
                               bore_out);
  EXPECT_EQ(result, RADARSIM_SUCCESS);

  EXPECT_NEAR(tx_out[0], 1.0f, kTolerance);
  EXPECT_NEAR(tx_out[1], 0.0f, kTolerance);
  EXPECT_NEAR(tx_out[2], 0.0f, kTolerance);

  EXPECT_NEAR(rx_out[0], -1.0f, kTolerance);
  EXPECT_NEAR(rx_out[1], 0.0f, kTolerance);
  EXPECT_NEAR(rx_out[2], 0.0f, kTolerance);

  EXPECT_NEAR(bore_out[0], 1.0f, kTolerance);
  EXPECT_NEAR(bore_out[1], 0.0f, kTolerance);
  EXPECT_NEAR(bore_out[2], 0.0f, kTolerance);
}

/**
 * @brief Constant-velocity radar - platform pose should extrapolate
 * analytically at query time.
 */
TEST_F(SceneStateTest, ConstantVelocityExtrapolation) {
  float tx_loc[3] = {0.0f, 0.0f, 0.0f};
  float rx_loc[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateTxRx(tx_loc, rx_loc));

  float loc[3] = {0.0f, 0.0f, 0.0f};
  float speed[3] = {2.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateStaticRadar(loc, speed, rotation, rotation_rate));

  double timestamps[1] = {5.0};
  float tx_out[3], rx_out[3], bore_out[3];

  int result = Get_Scene_State(setup_.radar, timestamps, 1, tx_out, rx_out,
                               bore_out);
  EXPECT_EQ(result, RADARSIM_SUCCESS);

  // location(5) = (0,0,0) + 5*(2,0,0) = (10,0,0); channel offset is zero
  EXPECT_NEAR(tx_out[0], 10.0f, kTolerance);
  EXPECT_NEAR(rx_out[0], 10.0f, kTolerance);
}

/**
 * @brief Rotated static radar - boresight should rotate from +X toward +Y
 * for a 90 degree yaw.
 */
TEST_F(SceneStateTest, RotatedRadarBoresight) {
  float tx_loc[3] = {1.0f, 0.0f, 0.0f};
  float rx_loc[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateTxRx(tx_loc, rx_loc));

  float loc[3] = {0.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {static_cast<float>(kPI) / 2.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateStaticRadar(loc, speed, rotation, rotation_rate));

  double timestamps[1] = {0.0};
  float tx_out[3], rx_out[3], bore_out[3];

  int result = Get_Scene_State(setup_.radar, timestamps, 1, tx_out, rx_out,
                               bore_out);
  EXPECT_EQ(result, RADARSIM_SUCCESS);

  EXPECT_NEAR(bore_out[0], 0.0f, kTolerance);
  EXPECT_NEAR(bore_out[1], 1.0f, kTolerance);
  EXPECT_NEAR(bore_out[2], 0.0f, kTolerance);

  // tx channel (1,0,0) local, rotated 90deg yaw -> (0,1,0) global
  EXPECT_NEAR(tx_out[0], 0.0f, kTolerance);
  EXPECT_NEAR(tx_out[1], 1.0f, kTolerance);
}

/**
 * @brief Time-varying radar with location/rotation array matching frame
 * count - should linearly interpolate against frame start times.
 */
TEST_F(SceneStateTest, TimeVaryingInterpolation) {
  float tx_loc[3] = {0.0f, 0.0f, 0.0f};
  float rx_loc[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateTxRx(tx_loc, rx_loc));

  double frame_start_time[3] = {0.0, 1.0, 2.0};
  float location_array[9] = {0.0f, 0.0f, 0.0f, 10.0f, 0.0f,
                             0.0f, 20.0f, 0.0f, 0.0f};
  float rotation_array[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};

  ASSERT_TRUE(setup_.CreateArrayRadar(frame_start_time, 3, location_array, 3,
                                      speed, rotation_array, 3,
                                      rotation_rate));

  double timestamps[4] = {0.5, 1.5, -1.0, 5.0};
  float tx_out[4 * 3], rx_out[4 * 3], bore_out[4 * 3];

  int result = Get_Scene_State(setup_.radar, timestamps, 4, tx_out, rx_out,
                               bore_out);
  EXPECT_EQ(result, RADARSIM_SUCCESS);

  EXPECT_NEAR(tx_out[0], 5.0f, kTolerance);    // t=0.5 -> interpolated
  EXPECT_NEAR(tx_out[3], 15.0f, kTolerance);   // t=1.5 -> interpolated
  EXPECT_NEAR(tx_out[6], 0.0f, kTolerance);    // t=-1.0 -> clamped to first
  EXPECT_NEAR(tx_out[9], 20.0f, kTolerance);   // t=5.0 -> clamped to last
}

/**
 * @brief Time-varying radar with a location array length that does not
 * match the frame count should be rejected.
 */
TEST_F(SceneStateTest, TimeVaryingMismatchedFrameCount) {
  float tx_loc[3] = {0.0f, 0.0f, 0.0f};
  float rx_loc[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateTxRx(tx_loc, rx_loc));

  double frame_start_time[1] = {0.0};
  float location_array[6] = {0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f};
  float rotation_array[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};

  ASSERT_TRUE(setup_.CreateArrayRadar(frame_start_time, 1, location_array, 2,
                                      speed, rotation_array, 2,
                                      rotation_rate));

  double timestamps[1] = {0.0};
  float tx_out[3], rx_out[3], bore_out[3];

  int result = Get_Scene_State(setup_.radar, timestamps, 1, tx_out, rx_out,
                               bore_out);
  EXPECT_EQ(result, RADARSIM_ERROR_INVALID_PARAMETER);
}

TEST_F(SceneStateTest, NullPointerAndInvalidParams) {
  float tx_loc[3] = {0.0f, 0.0f, 0.0f};
  float rx_loc[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateTxRx(tx_loc, rx_loc));

  float loc[3] = {0.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};
  ASSERT_TRUE(setup_.CreateStaticRadar(loc, speed, rotation, rotation_rate));

  double timestamps[1] = {0.0};
  float tx_out[3], rx_out[3], bore_out[3];

  EXPECT_EQ(Get_Scene_State(nullptr, timestamps, 1, tx_out, rx_out, bore_out),
           RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Scene_State(setup_.radar, nullptr, 1, tx_out, rx_out,
                            bore_out),
           RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Scene_State(setup_.radar, timestamps, 1, nullptr, rx_out,
                            bore_out),
           RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Scene_State(setup_.radar, timestamps, 0, tx_out, rx_out,
                            bore_out),
           RADARSIM_ERROR_INVALID_PARAMETER);
}

/**
 * @brief Test fixture for target-mesh scene-state tests
 */
class TargetMeshStateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mesh_points = {
        0.0f, 0.0f, 0.0f,  // vertex 0
        1.0f, 0.0f, 0.0f,  // vertex 1
        0.5f, 1.0f, 0.0f   // vertex 2
    };
    mesh_cells = {0, 1, 2};
    cell_size = 1;
  }

  void TearDown() override {
    if (targets) {
      Free_Targets(targets);
      targets = nullptr;
    }
  }

  std::vector<float> mesh_points;
  std::vector<int> mesh_cells;
  int cell_size;
  t_Targets* targets = nullptr;
};

TEST_F(TargetMeshStateTest, NumTargetsAndMeshSize) {
  targets = Init_Targets();
  ASSERT_NE(targets, nullptr);

  EXPECT_EQ(Get_Num_Targets(targets), 0);
  EXPECT_EQ(Get_Target_Mesh_Size(targets, 0), 0);

  float origin[3] = {0.0f, 0.0f, 0.0f};
  float location[3] = {10.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};

  int result = Add_Mesh_Target(mesh_points.data(), mesh_cells.data(),
                               cell_size, origin, location, speed, rotation,
                               rotation_rate, 1.0f, 0.0f, 1.0f, 0.0f, false,
                               0.0f, false, targets);
  ASSERT_EQ(result, 0);

  EXPECT_EQ(Get_Num_Targets(targets), 1);
  EXPECT_EQ(Get_Target_Mesh_Size(targets, 0), cell_size);
  EXPECT_EQ(Get_Target_Mesh_Size(targets, 1), 0);   // out of range
  EXPECT_EQ(Get_Target_Mesh_Size(targets, -1), 0);  // out of range
}

TEST_F(TargetMeshStateTest, NullPointerHelpers) {
  EXPECT_EQ(Get_Num_Targets(nullptr), 0);
  EXPECT_EQ(Get_Target_Mesh_Size(nullptr, 0), 0);
}

/**
 * @brief Constant-motion mesh target - vertices should translate exactly by
 * speed*time at each query timestamp.
 */
TEST_F(TargetMeshStateTest, ConstantMotionTranslation) {
  targets = Init_Targets();
  ASSERT_NE(targets, nullptr);

  float origin[3] = {0.0f, 0.0f, 0.0f};
  float location[3] = {10.0f, 0.0f, 0.0f};
  float speed[3] = {1.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};

  int result = Add_Mesh_Target(mesh_points.data(), mesh_cells.data(),
                               cell_size, origin, location, speed, rotation,
                               rotation_rate, 1.0f, 0.0f, 1.0f, 0.0f, false,
                               0.0f, false, targets);
  ASSERT_EQ(result, 0);

  double timestamps[2] = {0.0, 5.0};
  std::vector<double> points_out(2 * cell_size * 9);

  int gres = Get_Target_Mesh_State(targets, 0, timestamps, 2, nullptr, 0,
                                   points_out.data());
  EXPECT_EQ(gres, RADARSIM_SUCCESS);

  // t=0: vertex0 = mesh_points[0] + location = (0,0,0) + (10,0,0)
  EXPECT_NEAR(points_out[0], 10.0, 1e-3);
  EXPECT_NEAR(points_out[1], 0.0, 1e-3);
  EXPECT_NEAR(points_out[2], 0.0, 1e-3);
  // vertex1 = (1,0,0) + (10,0,0)
  EXPECT_NEAR(points_out[3], 11.0, 1e-3);

  // t=5: vertex0 = (0,0,0) + (10,0,0) + 5*(1,0,0) = (15,0,0)
  int base_t1 = cell_size * 9;
  EXPECT_NEAR(points_out[base_t1 + 0], 15.0, 1e-3);
  EXPECT_NEAR(points_out[base_t1 + 1], 0.0, 1e-3);
  EXPECT_NEAR(points_out[base_t1 + 2], 0.0, 1e-3);
}

/**
 * @brief Time-varying mesh target - query should nearest-neighbor match
 * against sim_timestamps and jump to the corresponding motion sample.
 */
TEST_F(TargetMeshStateTest, TimeVaryingNearestNeighbor) {
  targets = Init_Targets();
  ASSERT_NE(targets, nullptr);

  float origin[3] = {0.0f, 0.0f, 0.0f};
  float location_array[6] = {0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f};
  float speed_array[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float rotation_array[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float rotation_rate_array[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  int result = Add_Mesh_Target_Array(
      mesh_points.data(), mesh_cells.data(), cell_size, origin,
      location_array, speed_array, rotation_array, rotation_rate_array, 2,
      1.0f, 0.0f, 1.0f, 0.0f, false, 0.0f, false, targets);
  ASSERT_EQ(result, 0);

  double sim_timestamps[2] = {0.0, 10.0};
  double timestamps[2] = {1.0, 9.0};  // nearest to sample 0 and sample 1
  std::vector<double> points_out(2 * cell_size * 9);

  int gres = Get_Target_Mesh_State(targets, 0, timestamps, 2, sim_timestamps,
                                   2, points_out.data());
  EXPECT_EQ(gres, RADARSIM_SUCCESS);

  // query t=1.0 -> nearest sim sample is index 0 (location (0,0,0))
  EXPECT_NEAR(points_out[0], 0.0, 1e-3);
  // query t=9.0 -> nearest sim sample is index 1 (location (20,0,0))
  int base_t1 = cell_size * 9;
  EXPECT_NEAR(points_out[base_t1 + 0], 20.0, 1e-3);
}

TEST_F(TargetMeshStateTest, TimeVaryingRequiresSimTimestamps) {
  targets = Init_Targets();
  ASSERT_NE(targets, nullptr);

  float origin[3] = {0.0f, 0.0f, 0.0f};
  float location_array[6] = {0.0f, 0.0f, 0.0f, 20.0f, 0.0f, 0.0f};
  float speed_array[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float rotation_array[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float rotation_rate_array[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  int result = Add_Mesh_Target_Array(
      mesh_points.data(), mesh_cells.data(), cell_size, origin,
      location_array, speed_array, rotation_array, rotation_rate_array, 2,
      1.0f, 0.0f, 1.0f, 0.0f, false, 0.0f, false, targets);
  ASSERT_EQ(result, 0);

  double timestamps[1] = {1.0};
  std::vector<double> points_out(1 * cell_size * 9);

  // sim_timestamps omitted for a time-varying target -> rejected
  int gres = Get_Target_Mesh_State(targets, 0, timestamps, 1, nullptr, 0,
                                   points_out.data());
  EXPECT_EQ(gres, RADARSIM_ERROR_INVALID_PARAMETER);
}

TEST_F(TargetMeshStateTest, NullPointerAndInvalidParams) {
  targets = Init_Targets();
  ASSERT_NE(targets, nullptr);

  float origin[3] = {0.0f, 0.0f, 0.0f};
  float location[3] = {10.0f, 0.0f, 0.0f};
  float speed[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};

  int result = Add_Mesh_Target(mesh_points.data(), mesh_cells.data(),
                               cell_size, origin, location, speed, rotation,
                               rotation_rate, 1.0f, 0.0f, 1.0f, 0.0f, false,
                               0.0f, false, targets);
  ASSERT_EQ(result, 0);

  double timestamps[1] = {0.0};
  std::vector<double> points_out(cell_size * 9);

  EXPECT_EQ(Get_Target_Mesh_State(nullptr, 0, timestamps, 1, nullptr, 0,
                                  points_out.data()),
           RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Target_Mesh_State(targets, 0, nullptr, 1, nullptr, 0,
                                  points_out.data()),
           RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Target_Mesh_State(targets, 0, timestamps, 1, nullptr, 0,
                                  nullptr),
           RADARSIM_ERROR_NULL_POINTER);
  EXPECT_EQ(Get_Target_Mesh_State(targets, 5, timestamps, 1, nullptr, 0,
                                  points_out.data()),
           RADARSIM_ERROR_INVALID_PARAMETER);
  EXPECT_EQ(Get_Target_Mesh_State(targets, 0, timestamps, 0, nullptr, 0,
                                  points_out.data()),
           RADARSIM_ERROR_INVALID_PARAMETER);
}

/**
 * @brief Querying mesh state must not perturb the live target used by a
 * subsequent RCS simulation run (deep-copy safety).
 */
TEST_F(TargetMeshStateTest, QueryDoesNotMutateLiveTarget) {
  targets = Init_Targets();
  ASSERT_NE(targets, nullptr);

  float origin[3] = {0.0f, 0.0f, 0.0f};
  float location[3] = {10.0f, 0.0f, 0.0f};
  float speed[3] = {1.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float rotation_rate[3] = {0.0f, 0.0f, 0.0f};

  int result = Add_Mesh_Target(mesh_points.data(), mesh_cells.data(),
                               cell_size, origin, location, speed, rotation,
                               rotation_rate, -1.0f, 0.0f, 1.0f, 0.0f, false,
                               0.0f, false, targets);
  ASSERT_EQ(result, 0);

  // Query far in the future - if this mutated the live target, the RCS
  // simulation below would be evaluated against a target displaced by
  // speed*100, not the original one at t=0.
  double timestamps[1] = {100.0};
  std::vector<double> points_out(cell_size * 9);
  int gres = Get_Target_Mesh_State(targets, 0, timestamps, 1, nullptr, 0,
                                   points_out.data());
  ASSERT_EQ(gres, RADARSIM_SUCCESS);
  EXPECT_NEAR(points_out[0], 110.0, 1e-2);  // sanity: query result did move

  double inc_dir[3] = {-1.0, 0.0, 0.0};
  double obs_dir[3] = {-1.0, 0.0, 0.0};
  double inc_polar_real[3] = {0.0, 0.0, 1.0};
  double inc_polar_imag[3] = {0.0, 0.0, 0.0};
  double obs_polar_real[3] = {0.0, 0.0, 1.0};
  double obs_polar_imag[3] = {0.0, 0.0, 0.0};
  double rcs_result[1] = {0.0};

  int rcs_res = Run_RcsSimulator(targets, inc_dir, obs_dir, 1, inc_polar_real,
                                 inc_polar_imag, obs_polar_real,
                                 obs_polar_imag, 24e9, 1.0, rcs_result);
  // The RCS run should still complete against the untouched live target -
  // if the query had mutated location_ to (110,0,0), this would still
  // "succeed" numerically, so the real assertion is on Get_Target_Mesh_Size/
  // repeatability below.
  EXPECT_EQ(rcs_res, RADARSIM_SUCCESS);

  // Re-querying at t=0 should still reproduce the original construction
  // state - proving the earlier t=100 query left the live target untouched.
  double timestamps_zero[1] = {0.0};
  std::vector<double> points_out_zero(cell_size * 9);
  int gres2 = Get_Target_Mesh_State(targets, 0, timestamps_zero, 1, nullptr,
                                    0, points_out_zero.data());
  ASSERT_EQ(gres2, RADARSIM_SUCCESS);
  EXPECT_NEAR(points_out_zero[0], 10.0, 1e-2);
}
