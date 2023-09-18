/*
 *
 *    radarsim
 *
 *    ----------
 *    RadarSimC - A Radar Simulation Library Built with C++
 *    Copyright (C) 2018 - PRESENT  Zhengyu Peng
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

__declspec(dllexport) void Dump_Transmitter(t_Transmitter *ptr_tx_c);

/*********************************************
 *
 *  Receiver
 *
 *********************************************/
struct s_Receiver;
typedef struct s_Receiver t_Receiver;



#ifdef __cplusplus
}
#endif

#endif  // RADARSIM_HPP
