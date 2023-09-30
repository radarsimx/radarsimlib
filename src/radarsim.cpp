/*
 *
 *    C wrapper of RadarSimCpp
 *
 *    ----------
 *
 *    Copyright (C) 2023 - PRESENT  Zhengyu Peng
 *    E-mail: zpeng.me@gmail.com
 *    Website: https://zpeng.me
 *
 *    `                      `
 *    -:.                  -#:
 *    -//:.              -###:
 *    -////:.          -#####:
 *    -/:.://:.      -###++##:
 *    ..   `://:-  -###+. :##:
 *           `:/+####+.   :##:
 *    .::::::::/+###.     :##:
 *    .////-----+##:    `:###:
 *     `-//:.   :##:  `:###/.
 *       `-//:. :##:`:###/.
 *         `-//:+######/.
 *           `-/+####/.
 *             `+##+.
 *              :##:
 *              :##:
 *              :##:
 *              :##:
 *              :##:
 *               .+:
 *
 */

#include "radarsim.h"

#include <iostream>
#include <vector>

#include "point.hpp"
#include "radar.hpp"
#include "receiver.hpp"
#include "simulator.hpp"
#include "transmitter.hpp"

/*********************************************
 *
 *  Version
 *
 *********************************************/
/**
 * @brief Get the version of RadarSimLib
 *
 * @param version Version numbers {major, minor}
 */
void Get_Version(int version[2]) {
  version[0] = VERSION_MAJOR;
  version[1] = VERSION_MINOR;
}

/*********************************************
 *
 *  Transmitter
 *
 *********************************************/
struct s_Transmitter {
  Transmitter<float> *_ptr_transmitter;
};

/**
 * @brief Create a Transmitter, return the pointer of the Transmitter
 *
 * @param freq Frequency vector (Hz)
 * @param freq_time Timestamp vector for the frequency vector (s)
 * @param waveform_size Length of the frequency and timestamp vector
 * @param freq_offset Frequency offset per pulse (Hz)
 * length should equal to the number of pulses
 * @param pulse_start_time Pulse start time vector (s)
 * length should equal to the number of pulses
 * @param num_pulses Number of pulses
 * @param frame_start_time Frame start time vector (s)
 * length should equal to the number of frames
 * @param num_frames Number of frames
 * @param tx_power Transmitter power (dBm)
 * @return t_Transmitter* Ponter to the Transmitter
 */
t_Transmitter *Create_Transmitter(double *freq, double *freq_time,
                                  int waveform_size, double *freq_offset,
                                  double *pulse_start_time, int num_pulses,
                                  double *frame_start_time, int num_frames,
                                  float tx_power) {
  t_Transmitter *ptr_tx_c;

  ptr_tx_c = (t_Transmitter *)malloc(sizeof(t_Transmitter));

  std::vector<double> freq_vt;
  std::vector<double> freq_time_vt;
  freq_vt.reserve(waveform_size);
  freq_time_vt.reserve(waveform_size);
  for (int idx = 0; idx < waveform_size; idx++) {
    freq_vt.push_back(freq[idx]);
    freq_time_vt.push_back(freq_time[idx]);
  }

  std::vector<double> freq_offset_vt;
  std::vector<double> pulse_start_time_vt;
  freq_offset_vt.reserve(num_pulses);
  pulse_start_time_vt.reserve(num_pulses);
  for (int idx = 0; idx < num_pulses; idx++) {
    freq_offset_vt.push_back(freq_offset[idx]);
    pulse_start_time_vt.push_back(pulse_start_time[idx]);
  }

  std::vector<double> frame_start_time_vt;
  frame_start_time_vt.reserve(num_frames);
  for (int idx = 0; idx < num_frames; idx++) {
    frame_start_time_vt.push_back(frame_start_time[idx]);
  }

  ptr_tx_c->_ptr_transmitter =
      new Transmitter<float>(tx_power, freq_vt, freq_time_vt, freq_offset_vt,
                             pulse_start_time_vt, frame_start_time_vt);

  return ptr_tx_c;
}

/**
 * @brief Add a transmitter channel to Transmitter
 *
 * @param location Location of the channel [x, y, z] (m)
 * @param polar Polarization of the channel [x, y, z]
 * @param phi Phi angles of the channel's radiation pattern (rad)
 * angles must be equal-spaced incremental array
 * @param phi_ptn Normalized radiation pattern along phi (dB)
 * @param phi_length Length of phi and phi_ptn
 * @param theta Theta angles of the channel's radiation pattern (rad)
 * angles must be equal-spaced incremental array
 * @param theta_ptn Normalized radiation pattern along theta (dB)
 * @param theta_length Length of theta and theta_ptn
 * @param antenna_gain Antenna gain (dB)
 * @param mod_t Timestamp of the modulation data (s)
 * mod_t must be equal-spaced incremental array
 * @param mod_var_real Real part of modulation value vector
 * @param mod_var_imag Imag part of modulation value vector
 * @param mod_length Length of mod_t, mod_var_real and mod_var_imag
 * @param pulse_mod_real Real part of pulse modulation vector
 * the length should be the same
 * as the number of pulses defined in Transmitter
 * @param pulse_mod_imag Imag part of pulse modulation vector
 * the length should be the same
 * as the number of pulses defined in Transmitter
 * @param delay Transmitting delay (s)
 * @param grid Ray occupancy checking grid (rad)
 * @param ptr_tx_c Pointer to the Transmitter
 */
void Add_Txchannel(float *location, float *polar, float *phi, float *phi_ptn,
                   int phi_length, float *theta, float *theta_ptn,
                   int theta_length, float antenna_gain, float *mod_t,
                   float *mod_var_real, float *mod_var_imag, int mod_length,
                   float *pulse_mod_real, float *pulse_mod_imag, float delay,
                   float grid, t_Transmitter *ptr_tx_c) {
  std::vector<float> phi_vt, phi_ptn_vt;
  phi_vt.reserve(phi_length);
  phi_ptn_vt.reserve(phi_length);
  for (int idx = 0; idx < phi_length; idx++) {
    phi_vt.push_back(phi[idx]);
    phi_ptn_vt.push_back(phi_ptn[idx]);
  }

  std::vector<float> theta_vt, theta_ptn_vt;
  theta_vt.reserve(theta_length);
  theta_ptn_vt.reserve(theta_length);
  for (int idx = 0; idx < theta_length; idx++) {
    theta_vt.push_back(theta[idx]);
    theta_ptn_vt.push_back(theta_ptn[idx]);
  }

  std::vector<float> mod_t_vt;
  std::vector<std::complex<float>> mod_var_vt;
  mod_t_vt.reserve(mod_length);
  mod_var_vt.reserve(mod_length);
  for (int idx = 0; idx < mod_length; idx++) {
    mod_t_vt.push_back(mod_t[idx]);
    mod_var_vt.push_back(
        std::complex<float>(mod_var_real[idx], mod_var_imag[idx]));
  }

  std::vector<std::complex<float>> pulse_mod_vt;
  pulse_mod_vt.reserve(ptr_tx_c->_ptr_transmitter->pulse_size_);
  for (int idx = 0; idx < ptr_tx_c->_ptr_transmitter->pulse_size_; idx++) {
    pulse_mod_vt.push_back(
        std::complex<float>(pulse_mod_real[idx], pulse_mod_imag[idx]));
  }

  ptr_tx_c->_ptr_transmitter->AddChannel(
      TxChannel<float>(zpv::Vec3<float>(location[0], location[1], location[2]),
                       zpv::Vec3<float>(polar[0], polar[1], polar[2]), phi_vt,
                       phi_ptn_vt, theta_vt, theta_ptn_vt, antenna_gain,
                       mod_t_vt, mod_var_vt, pulse_mod_vt, delay, grid));
}

/**
 * @brief Get the number of Tx channels
 * 
 * @param ptr_tx_c Pointer to the Transmitter
 * @return int Number of Tx channels
 */
int Get_Num_Txchannel(t_Transmitter *ptr_tx_c){
  return ptr_tx_c->_ptr_transmitter->channel_size_;
}

/**
 * @brief Free the memory of Transmitter
 *
 * @param ptr_tx_c Pointer to the Transmitter
 */
void Free_Transmitter(t_Transmitter *ptr_tx_c) {
  if (ptr_tx_c == NULL) {
    return;
  }
  delete static_cast<Transmitter<float> *>(ptr_tx_c->_ptr_transmitter);
  free(ptr_tx_c);
}

/*********************************************
 *
 *  Receiver
 *
 *********************************************/
struct s_Receiver {
  Receiver<float> *_ptr_receiver;
};

/**
 * @brief Create a Receiver, return the pointer of the Receiver
 *
 * @param fs Sampling rate (Hz)
 * @param rf_gain RF gain (dB)
 * @param resistor Load resistor (Ohm)
 * @param baseband_gain Baseband gain (dB)
 * @return t_Receiver* Pointer to Receiver
 */
t_Receiver *Create_Receiver(float fs, float rf_gain, float resistor,
                            float baseband_gain) {
  t_Receiver *ptr_rx_c;
  ptr_rx_c = (t_Receiver *)malloc(sizeof(t_Receiver));

  ptr_rx_c->_ptr_receiver =
      new Receiver<float>(fs, rf_gain, resistor, baseband_gain);

  return ptr_rx_c;
}

/**
 * @brief Add a receiver channel to Receiver
 *
 * @param location Location of the channel [x, y, z] (m)
 * @param polar Polarization of the channel [x, y, z]
 * @param phi Phi angles of the channel's radiation pattern (rad)
 * angles must be equal-spaced incremental array
 * @param phi_ptn Normalized radiation pattern along phi (dB)
 * @param phi_length Length of phi and phi_ptn
 * @param theta Theta angles of the channel's radiation pattern (rad)
 * angles must be equal-spaced incremental array
 * @param theta_ptn Normalized radiation pattern along theta (dB)
 * @param theta_length Length of theta and theta_ptn
 * @param antenna_gain Antenna gain (dB)
 * @param ptr_rx_c Pointer to Receiver
 */
void Add_Rxchannel(float *location, float *polar, float *phi, float *phi_ptn,
                   int phi_length, float *theta, float *theta_ptn,
                   int theta_length, float antenna_gain, t_Receiver *ptr_rx_c) {
  std::vector<float> phi_vt, phi_ptn_vt;
  phi_vt.reserve(phi_length);
  phi_ptn_vt.reserve(phi_length);
  for (int idx = 0; idx < phi_length; idx++) {
    phi_vt.push_back(phi[idx]);
    phi_ptn_vt.push_back(phi_ptn[idx]);
  }

  std::vector<float> theta_vt, theta_ptn_vt;
  theta_vt.reserve(theta_length);
  theta_ptn_vt.reserve(theta_length);
  for (int idx = 0; idx < theta_length; idx++) {
    theta_vt.push_back(theta[idx]);
    theta_ptn_vt.push_back(theta_ptn[idx]);
  }

  ptr_rx_c->_ptr_receiver->AddChannel(
      RxChannel<float>(zpv::Vec3<float>(location[0], location[1], location[2]),
                       zpv::Vec3<float>(polar[0], polar[1], polar[2]), phi_vt,
                       phi_ptn_vt, theta_vt, theta_ptn_vt, antenna_gain));
}

/**
 * @brief Get the number of Rx channels
 * 
 * @param ptr_rx_c Pointer to the Receiver
 * @return int Number of Rx channels
 */
int Get_Num_Rxchannel(t_Receiver *ptr_rx_c){
  return ptr_rx_c->_ptr_receiver->channel_size_;
}

/**
 * @brief Free the memory of Receiver
 *
 * @param ptr_rx_c Pointer to the Receiver
 */
void Free_Receiver(t_Receiver *ptr_rx_c) {
  if (ptr_rx_c == NULL) {
    return;
  }
  delete static_cast<Receiver<float> *>(ptr_rx_c->_ptr_receiver);
  free(ptr_rx_c);
}

/*********************************************
 *
 *  Radar
 *
 *********************************************/
struct s_Radar {
  t_Transmitter *_ptr_tx;
  t_Receiver *_ptr_rx;
  Radar<float> *_ptr_radar;
};

/**
 * @brief Create a Radar, return the pointer of the Radar
 *
 * @param ptr_tx_c Pointer to the Transmitter
 * @param ptr_rx_c Pointer to the Receiver
 * @return t_Radar* Pointer to the Radar
 */
t_Radar *Create_Radar(t_Transmitter *ptr_tx_c, t_Receiver *ptr_rx_c) {
  t_Radar *ptr_radar_c;
  ptr_radar_c = (t_Radar *)malloc(sizeof(t_Radar));
  ptr_radar_c->_ptr_tx = ptr_tx_c;
  ptr_radar_c->_ptr_rx = ptr_rx_c;

  ptr_radar_c->_ptr_radar =
      new Radar<float>(*ptr_tx_c->_ptr_transmitter, *ptr_rx_c->_ptr_receiver);

  return ptr_radar_c;
}

/**
 * @brief Set radar's location and motion
 *
 * @param location Radar's location {x, y, z} (m)
 * @param speed Radar's speed {x, y, z} (m/s)
 * @param rotation Radar's rotation {x, y, z} (rad)
 * @param rotation_rate Radar's rotation rate {x, y, z} (rad/s)
 * @param ptr_radar_c Pointer to the Radar
 */
void Set_Radar_Motion(float *location, float *speed, float *rotation,
                      float *rotation_rate, t_Radar *ptr_radar_c) {
  std::vector<zpv::Vec3<float>> loc_vt, spd_vt, rot_vt, rrt_vt;
  loc_vt.push_back(zpv::Vec3<float>(location[0], location[1], location[2]));
  spd_vt.push_back(zpv::Vec3<float>(speed[0], speed[1], speed[2]));
  rot_vt.push_back(zpv::Vec3<float>(rotation[0], rotation[1], rotation[2]));
  rrt_vt.push_back(
      zpv::Vec3<float>(rotation_rate[0], rotation_rate[1], rotation_rate[2]));
  ptr_radar_c->_ptr_radar->SetMotion(loc_vt, spd_vt, rot_vt, rrt_vt);
}

/**
 * @brief Free the memory of Radar
 *
 * @param ptr_radar_c Pointer to the Radar
 */
void Free_Radar(t_Radar *ptr_radar_c) {
  Free_Transmitter(ptr_radar_c->_ptr_tx);
  Free_Receiver(ptr_radar_c->_ptr_rx);
  if (ptr_radar_c == NULL) {
    return;
  }
  delete static_cast<Radar<float> *>(ptr_radar_c->_ptr_radar);
  free(ptr_radar_c);
}

/*********************************************
 *
 *  Targets
 *
 *********************************************/
struct s_Targets {
  PointList<float> *_ptr_points;
};

/**
 * @brief Initialize the target list
 *
 * @return t_Targets* Pointer to the target list
 */
t_Targets *Init_Targets() {
  t_Targets *ptr_targets_c;
  ptr_targets_c = (t_Targets *)malloc(sizeof(t_Targets));

  ptr_targets_c->_ptr_points = new PointList<float>();
  return ptr_targets_c;
}

/**
 * @brief Add an ideal point target to the target list
 *
 * @param loc Target's location {x, y, z} (m)
 * @param speed Target's speed {x, y, z} (m/s)
 * @param rcs Target's RCS (dBsm)
 * @param phs Target's phase (rad)
 * @param ptr_targets_c Pointer to the target list
 */
void Add_Target(float *loc, float *speed, float rcs, float phs,
                t_Targets *ptr_targets_c) {
  ptr_targets_c->_ptr_points->Add_Point(
      Point<float>(zpv::Vec3<float>(loc[0], loc[1], loc[2]),
                   zpv::Vec3<float>(speed[0], speed[1], speed[2]), rcs, phs));
}

/**
 * @brief Free the memory of target list
 *
 * @param ptr_targets_c Pointer to the target list
 */
void Free_Targets(t_Targets *ptr_targets_c) {
  if (ptr_targets_c == NULL) {
    return;
  }
  delete static_cast<PointList<float> *>(ptr_targets_c->_ptr_points);
  free(ptr_targets_c);
}

/*********************************************
 *
 *  Simulator
 *
 *********************************************/
/**
 * @brief Run simulator for the ideal targets
 *
 * @param ptr_radar_c Pointer to the radar
 * @param ptr_targets_c Pointer to the target list
 * @param ptr_bb_real Real part of baseband samples
 * @param ptr_bb_imag Imag part of baseband samples
 */
void Run_Simulator(t_Radar *ptr_radar_c, t_Targets *ptr_targets_c,
                   double *ptr_bb_real, double *ptr_bb_imag) {
  Simulator<float> simc = Simulator<float>();

  simc.Run(*ptr_radar_c->_ptr_radar, ptr_targets_c->_ptr_points->ptr_points_,
           ptr_bb_real, ptr_bb_imag);
}

/*********************************************
 *
 *  main
 *
 *********************************************/
// int main() {
//   double f[] = {24.075e9, 24.175e9};
//   double t[] = {0, 80e-6};

//   double freq_offset[256];
//   for (int idx = 0; idx < 256; idx++) {
//     freq_offset[idx] = 0.0;
//   }

//   double pulse_start_time[256];
//   for (int idx = 0; idx < 256; idx++) {
//     pulse_start_time[idx] = 100e-6 * idx;
//   }

//   double frame_start_time[] = {0};

//   t_Transmitter *tx_ptr = Create_Transmitter(
//       f, t, 2, freq_offset, pulse_start_time, 256, frame_start_time, 1, 10.0);

//   float location[] = {0, 0, 0};
//   float polar[] = {0, 0, 1};
//   float phi[] = {-90, 90};
//   float phi_ptn[] = {0, 0};
//   float theta[] = {0, 180};
//   float theta_ptn[] = {0, 0};
//   float antenna_gain = 0;
//   float mod_t[1];
//   float mod_var_real[1];
//   float mod_var_imag[1];

//   float pulse_mod_real[256];
//   float pulse_mod_imag[256];
//   for (int idx = 0; idx < 256; idx++) {
//     pulse_mod_real[idx] = 1.0;
//     pulse_mod_imag[idx] = 0.0;
//   }

//   Add_Txchannel(location, polar, phi, phi_ptn, 2, theta, theta_ptn, 2,
//                 antenna_gain, mod_t, mod_var_real, mod_var_imag, 0,
//                 pulse_mod_real, pulse_mod_imag, 0, 1, tx_ptr);

//   float fs = 2e6;
//   float rf_gain = 20;
//   float resistor = 500;
//   float baseband_gain = 30;
//   t_Receiver *rx_ptr = Create_Receiver(fs, rf_gain, resistor, baseband_gain);

//   Add_Rxchannel(location, polar, phi, phi_ptn, 2, theta, theta_ptn, 2,
//                 antenna_gain, rx_ptr);

//   t_Radar *radar_ptr = Create_Radar(tx_ptr, rx_ptr);

//   float radar_loc_ptr[] = {0, 0, 0};
//   float radar_spd_ptr[] = {0, 0, 0};
//   float radar_rot_ptr[] = {0, 0, 0};
//   float radar_rrt_ptr[] = {0, 0, 0};
//   Set_Radar_Motion(radar_loc_ptr, radar_spd_ptr, radar_rot_ptr, radar_rrt_ptr,
//                radar_ptr);

//   t_Targets *targets_ptr = Init_Targets();

//   float tg_loc[] = {200, 0, 0};
//   float tg_speed[] = {-5, 0, 0};
//   Add_Target(tg_loc, tg_speed, 20, 0, targets_ptr);

//   double bb_real[256][160];
//   double bb_imag[256][160];
//   for (int idx_a = 0; idx_a < 160; idx_a++) {
//     for (int idx_b = 0; idx_b < 256; idx_b++) {
//       bb_real[idx_b][idx_a] = 0.0;
//       bb_imag[idx_b][idx_a] = 0.0;
//     }
//   }

//   Run_Simulator(radar_ptr, targets_ptr, &bb_real[0][0], &bb_imag[0][0]);

//   return 0;
// }
