#include "radarsim.h"

#include <iostream>
#include <vector>

#include "point.hpp"
#include "radar.hpp"
#include "receiver.hpp"
#include "transmitter.hpp"

/*********************************************
 *
 *  Transmitter
 *
 *********************************************/

struct s_Transmitter {
  Transmitter<float> *_ptr_transmitter;
};

/**
 * @brief
 *
 * @param freq
 * @param freq_time
 * @param waveform_size
 * @param freq_offset
 * @param pulse_start_time
 * @param num_pulses
 * @param frame_start_time
 * @param num_frames
 * @param tx_power
 * @return t_Transmitter*
 */
t_Transmitter *Create_Transmitter(double *freq, double *freq_time,
                                  int waveform_size, double *freq_offset,
                                  double *pulse_start_time, int num_pulses,
                                  double *frame_start_time, int num_frames,
                                  float tx_power) {
  t_Transmitter *ptr_tx_c;

  ptr_tx_c = (t_Transmitter *)malloc(sizeof(t_Transmitter *));

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
 * @brief
 *
 * @param location
 * @param polar
 * @param phi
 * @param phi_ptn
 * @param phi_length
 * @param theta
 * @param theta_ptn
 * @param theta_length
 * @param antenna_gain
 * @param mod_t
 * @param mod_var_real
 * @param mod_var_imag
 * @param mod_length
 * @param pulse_mod_real
 * @param pulse_mod_imag
 * @param delay
 * @param grid
 * @param ptr_tx_c
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
 * @brief
 *
 * @param ptr_tx_c
 */
void Free_Transmitter(t_Transmitter *ptr_tx_c) {
  if (ptr_tx_c == NULL) return;
  delete static_cast<Transmitter<float> *>(ptr_tx_c->_ptr_transmitter);
  free(ptr_tx_c);
}

void Dump_Transmitter(t_Transmitter *ptr_tx_c) {
  ptr_tx_c->_ptr_transmitter->Dump("");
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
 * @brief
 *
 * @param fs
 * @param rf_gain
 * @param resistor
 * @param baseband_gain
 * @param samples
 * @return t_Receiver*
 */
t_Receiver *Create_Receiver(float fs, float rf_gain, float resistor,
                            float baseband_gain, int samples) {
  t_Receiver *ptr_rx_c;
  ptr_rx_c = (t_Receiver *)malloc(sizeof(t_Receiver *));

  ptr_rx_c->_ptr_receiver =
      new Receiver<float>(fs, rf_gain, resistor, baseband_gain, samples);

  return ptr_rx_c;
}

/**
 * @brief
 *
 * @param location
 * @param polar
 * @param phi
 * @param phi_ptn
 * @param phi_length
 * @param theta
 * @param theta_ptn
 * @param theta_length
 * @param antenna_gain
 * @param ptr_rx_c
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
 * @brief
 *
 * @param ptr_rx_c
 */
void Free_Receiver(t_Receiver *ptr_rx_c) {
  if (ptr_rx_c == NULL) return;
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
 * @brief
 *
 * @param ptr_tx_c
 * @param ptr_rx_c
 * @return t_Radar*
 */
t_Radar *Create_Radar(t_Transmitter *ptr_tx_c, t_Receiver *ptr_rx_c) {
  t_Radar *ptr_radar_c;
  ptr_radar_c = (t_Radar *)malloc(sizeof(t_Radar *));
  ptr_radar_c->_ptr_tx = ptr_tx_c;
  ptr_radar_c->_ptr_rx = ptr_rx_c;

  ptr_radar_c->_ptr_radar =
      new Radar<float>(*ptr_tx_c->_ptr_transmitter, *ptr_rx_c->_ptr_receiver);

  return ptr_radar_c;
}

/**
 * @brief
 *
 * @param ptr_radar_c
 */
void Free_Radar(t_Radar *ptr_radar_c) {
  if (ptr_radar_c == NULL) return;

  Free_Transmitter(ptr_radar_c->_ptr_tx);
  Free_Receiver(ptr_radar_c->_ptr_rx);
  delete static_cast<Radar<float> *>(ptr_radar_c->_ptr_radar);
  free(ptr_radar_c);
}

/*********************************************
 *
 *  Targets
 *
 *********************************************/
struct s_Targets {
  std::vector<Point<float>> *_points;
};

t_Targets *Init_Targets() {
  t_Targets *ptr_targets_c;
  ptr_targets_c = (t_Targets *)malloc(sizeof(t_Targets *));

  return ptr_targets_c;
}

void Add_Target(float *loc, float *speed, float rcs, float phs,
                t_Targets *ptr_targets_c) {
  // ptr_targets_c.push_back(Point<float>());
}
