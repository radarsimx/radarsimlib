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

#ifndef RADARSIM_HPP
#define RADARSIM_HPP

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************
 *
 *  Transmitter
 *
 *********************************************/
struct s_Transmitter;
typedef struct s_Transmitter t_Transmitter;

__declspec(dllexport) t_Transmitter *Create_Transmitter(
    double *freq, double *freq_time, int waveform_size, double *freq_offset,
    double *pulse_start_time, int num_pulses, double *frame_start_time,
    int num_frames, float tx_power);

__declspec(dllexport) void Add_Txchannel(
    float *location, float *polar, float *phi, float *phi_ptn, int phi_length,
    float *theta, float *theta_ptn, int theta_length, float antenna_gain,
    float *mod_t, float *mod_var_real, float *mod_var_imag, int mod_length,
    float *pulse_mod_real, float *pulse_mod_imag, float delay, float grid,
    t_Transmitter *ptr_tx_c);

__declspec(dllexport) void Free_Transmitter(t_Transmitter *ptr_tx_c);

/*********************************************
 *
 *  Receiver
 *
 *********************************************/
struct s_Receiver;
typedef struct s_Receiver t_Receiver;

__declspec(dllexport) t_Receiver *Create_Receiver(float fs, float rf_gain,
                                                  float resistor,
                                                  float baseband_gain);

__declspec(dllexport) void Add_Rxchannel(float *location, float *polar,
                                         float *phi, float *phi_ptn,
                                         int phi_length, float *theta,
                                         float *theta_ptn, int theta_length,
                                         float antenna_gain,
                                         t_Receiver *ptr_rx_c);

__declspec(dllexport) void Free_Receiver(t_Receiver *ptr_rx_c);

// __declspec(dllexport) void Dump_Receiver(t_Receiver *ptr_rx_c);

/*********************************************
 *
 *  Radar
 *
 *********************************************/
struct s_Radar;
typedef struct s_Radar t_Radar;

__declspec(dllexport) t_Radar *Create_Radar(t_Transmitter *ptr_tx_c,
                                            t_Receiver *ptr_rx_c);

__declspec(dllexport) void Radar_Motion(float *location, float *speed,
                                        float *rotation, float *rotation_rate,
                                        t_Radar *ptr_radar_c);

__declspec(dllexport) void Free_Radar(t_Radar *ptr_radar_c);

/*********************************************
 *
 *  Targets
 *
 *********************************************/
struct s_Targets;
typedef struct s_Targets t_Targets;

__declspec(dllexport) t_Targets *Init_Targets();

__declspec(dllexport) void Add_Target(float *loc, float *speed, float rcs,
                                      float phs, t_Targets *ptr_targets_c);

/*********************************************
 *
 *  Simulator
 *
 *********************************************/
__declspec(dllexport) void Run_Simulator(t_Radar *ptr_radar_c,
                                         t_Targets *ptr_targets_c,
                                         double *ptr_bb_real,
                                         double *ptr_bb_imag);

#ifdef __cplusplus
}
#endif

#endif  // RADARSIM_HPP
