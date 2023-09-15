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

struct c_Transmitter;

c_Transmitter *Create_Transmitter(double *freq, double *freq_time,
                                  int waveform_size, double *freq_offset,
                                  double *pulse_start_time, int num_pulses,
                                  double *frame_start_time, int num_frames,
                                  float tx_power);

#ifdef __cplusplus
}
#endif

#endif  // RADARSIM_HPP
