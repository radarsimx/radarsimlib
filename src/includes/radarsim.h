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

struct s_Transmitter;
typedef struct s_Transmitter t_Transmitter;

__declspec(dllexport) t_Transmitter *Create_Transmitter(
    double *freq, double *freq_time, int waveform_size, double *freq_offset,
    double *pulse_start_time, int num_pulses, double *frame_start_time,
    int num_frames, float tx_power);

__declspec(dllexport) void Free_Transmitter(t_Transmitter *ptr_tx_c);

#ifdef __cplusplus
}
#endif

#endif  // RADARSIM_HPP
