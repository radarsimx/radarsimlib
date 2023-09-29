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

#define VERSION_MAJOR 1
#define VERSION_MINOR 0

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
__declspec(dllexport) void Get_Version(int version[2]);

/*********************************************
 *
 *  Transmitter
 *
 *********************************************/
struct s_Transmitter;
typedef struct s_Transmitter t_Transmitter;

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
__declspec(dllexport) t_Transmitter *Create_Transmitter(
    double *freq, double *freq_time, int waveform_size, double *freq_offset,
    double *pulse_start_time, int num_pulses, double *frame_start_time,
    int num_frames, float tx_power);

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
__declspec(dllexport) void Add_Txchannel(
    float *location, float *polar, float *phi, float *phi_ptn, int phi_length,
    float *theta, float *theta_ptn, int theta_length, float antenna_gain,
    float *mod_t, float *mod_var_real, float *mod_var_imag, int mod_length,
    float *pulse_mod_real, float *pulse_mod_imag, float delay, float grid,
    t_Transmitter *ptr_tx_c);

/**
 * @brief Free the memory of Transmitter
 *
 * @param ptr_tx_c Pointer to the Transmitter
 */
__declspec(dllexport) void Free_Transmitter(t_Transmitter *ptr_tx_c);

/*********************************************
 *
 *  Receiver
 *
 *********************************************/
struct s_Receiver;
typedef struct s_Receiver t_Receiver;

/**
 * @brief Create a Receiver, return the pointer of the Receiver
 *
 * @param fs Sampling rate (Hz)
 * @param rf_gain RF gain (dB)
 * @param resistor Load resistor (Ohm)
 * @param baseband_gain Baseband gain (dB)
 * @return t_Receiver* Pointer to Receiver
 */
__declspec(dllexport) t_Receiver *Create_Receiver(float fs, float rf_gain,
                                                  float resistor,
                                                  float baseband_gain);

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
__declspec(dllexport) void Add_Rxchannel(float *location, float *polar,
                                         float *phi, float *phi_ptn,
                                         int phi_length, float *theta,
                                         float *theta_ptn, int theta_length,
                                         float antenna_gain,
                                         t_Receiver *ptr_rx_c);

/**
 * @brief Free the memory of Receiver
 *
 * @param ptr_rx_c Pointer to the Receiver
 */
__declspec(dllexport) void Free_Receiver(t_Receiver *ptr_rx_c);

/*********************************************
 *
 *  Radar
 *
 *********************************************/
struct s_Radar;
typedef struct s_Radar t_Radar;

/**
 * @brief Create a Radar, return the pointer of the Radar
 *
 * @param ptr_tx_c Pointer to the Transmitter
 * @param ptr_rx_c Pointer to the Receiver
 * @return t_Radar* Pointer to the Radar
 */
__declspec(dllexport) t_Radar *Create_Radar(t_Transmitter *ptr_tx_c,
                                            t_Receiver *ptr_rx_c);

/**
 * @brief Set radar's location and motion
 *
 * @param location Radar's location {x, y, z} (m)
 * @param speed Radar's speed {x, y, z} (m/s)
 * @param rotation Radar's rotation {x, y, z} (rad)
 * @param rotation_rate Radar's rotation rate {x, y, z} (rad/s)
 * @param ptr_radar_c Pointer to the Radar
 */
__declspec(dllexport) void Set_Radar_Motion(float *location, float *speed,
                                            float *rotation,
                                            float *rotation_rate,
                                            t_Radar *ptr_radar_c);

/**
 * @brief Free the memory of Radar
 *
 * @param ptr_radar_c Pointer to the Radar
 */
__declspec(dllexport) void Free_Radar(t_Radar *ptr_radar_c);

/*********************************************
 *
 *  Targets
 *
 *********************************************/
struct s_Targets;
typedef struct s_Targets t_Targets;

/**
 * @brief Initialize the target list
 *
 * @return t_Targets* Pointer to the target list
 */
__declspec(dllexport) t_Targets *Init_Targets();

/**
 * @brief Add an ideal point target to the target list
 *
 * @param loc Target's location {x, y, z} (m)
 * @param speed Target's speed {x, y, z} (m/s)
 * @param rcs Target's RCS (dBsm)
 * @param phs Target's phase (rad)
 * @param ptr_targets_c Pointer to the target list
 */
__declspec(dllexport) void Add_Target(float *loc, float *speed, float rcs,
                                      float phs, t_Targets *ptr_targets_c);

/**
 * @brief Free the memory of target list
 *
 * @param ptr_targets_c Pointer to the target list
 */
__declspec(dllexport) void Free_Targets(t_Targets *ptr_targets_c);

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
__declspec(dllexport) void Run_Simulator(t_Radar *ptr_radar_c,
                                         t_Targets *ptr_targets_c,
                                         double *ptr_bb_real,
                                         double *ptr_bb_imag);

#ifdef __cplusplus
}
#endif

#endif  // RADARSIM_HPP
