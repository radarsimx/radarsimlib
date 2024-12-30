/*
 *
 *    C wrapper of RadarSimCpp
 *
 *    ----------
 *
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

#ifndef RADARSIM_HPP
#define RADARSIM_HPP

#ifdef __cplusplus
extern "C" {
#endif

#if defined _WIN64 || defined _WIN32
#define EXPORTED __declspec(dllexport)
#else
#define EXPORTED
#endif

#define VERSION_MAJOR 3
#define VERSION_MINOR 2

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
EXPORTED void Get_Version(int version[2]);

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
 * @param freq_offset Frequency offset per pulse (Hz), length should equal to the number of pulses
 * @param pulse_start_time Pulse start time vector (s), length should equal to the number of pulses
 * @param num_pulses Number of pulses
 * @param frame_start_time Frame start time vector (s), length should equal to the number of frames
 * @param num_frames Number of frames
 * @param tx_power Transmitter power (dBm)
 * @return t_Transmitter* Pointer to the Transmitter
 */
EXPORTED t_Transmitter *Create_Transmitter(
    double *freq, double *freq_time, int waveform_size, double *freq_offset,
    double *pulse_start_time, int num_pulses, double *frame_start_time,
    int num_frames, float tx_power);

/**
 * @brief Add a transmitter channel to Transmitter
 *
 * @param location Location of the channel [x, y, z] (m)
 * @param polar_real Real part of the polarization vector [x, y, z]
 * @param polar_imag Imaginary part of the polarization vector [x, y, z]
 * @param phi Phi angles of the channel's radiation pattern (rad), angles must be equal-spaced incremental array
 * @param phi_ptn Normalized radiation pattern along phi (dB)
 * @param phi_length Length of phi and phi_ptn
 * @param theta Theta angles of the channel's radiation pattern (rad), angles must be equal-spaced incremental array
 * @param theta_ptn Normalized radiation pattern along theta (dB)
 * @param theta_length Length of theta and theta_ptn
 * @param antenna_gain Antenna gain (dB)
 * @param mod_t Timestamp of the modulation data (s), mod_t must be equal-spaced incremental array
 * @param mod_var_real Real part of modulation value vector
 * @param mod_var_imag Imaginary part of modulation value vector
 * @param mod_length Length of mod_t, mod_var_real and mod_var_imag
 * @param pulse_mod_real Real part of pulse modulation vector,the length should be the same as the number of pulses defined in Transmitter
 * @param pulse_mod_imag Imaginary part of pulse modulation vector, the length should be the same as the number of pulses defined in Transmitter
 * @param delay Transmitting delay (s)
 * @param grid Ray occupancy checking grid (rad)
 * @param ptr_tx_c Pointer to the Transmitter
 * @return int Status code (0 for success, 1 for failure)
 */
EXPORTED int Add_Txchannel(float *location, float *polar_real,
                           float *polar_imag, float *phi, float *phi_ptn,
                           int phi_length, float *theta, float *theta_ptn,
                           int theta_length, float antenna_gain, float *mod_t,
                           float *mod_var_real, float *mod_var_imag,
                           int mod_length, float *pulse_mod_real,
                           float *pulse_mod_imag, float delay, float grid,
                           t_Transmitter *ptr_tx_c);

/**
 * @brief Get the number of Tx channels
 *
 * @param ptr_tx_c Pointer to the Transmitter
 * @return int Number of Tx channels
 */
EXPORTED int Get_Num_Txchannel(t_Transmitter *ptr_tx_c);

/**
 * @brief Free the memory of Transmitter
 *
 * @param ptr_tx_c Pointer to the Transmitter
 */
EXPORTED void Free_Transmitter(t_Transmitter *ptr_tx_c);

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
 * @param baseband_bw Baseband bandwidth (Hz)
 * @return t_Receiver* Pointer to the Receiver
 */
EXPORTED t_Receiver *Create_Receiver(float fs, float rf_gain, float resistor,
                                     float baseband_gain, float baseband_bw);

/**
 * @brief Add a receiver channel to Receiver
 *
 * @param location Location of the channel [x, y, z] (m)
 * @param polar_real Real part of the polarization vector [x, y, z]
 * @param polar_imag Imaginary part of the polarization vector [x, y, z]
 * @param phi Phi angles of the channel's radiation pattern (rad), angles must be equal-spaced incremental array
 * @param phi_ptn Normalized radiation pattern along phi (dB)
 * @param phi_length Length of phi and phi_ptn
 * @param theta Theta angles of the channel's radiation pattern (rad), angles must be equal-spaced incremental array
 * @param theta_ptn Normalized radiation pattern along theta (dB)
 * @param theta_length Length of theta and theta_ptn
 * @param antenna_gain Antenna gain (dB)
 * @param ptr_rx_c Pointer to the Receiver
 * @return int Status code (0 for success, 1 for failure)
 */
EXPORTED int Add_Rxchannel(float *location, float *polar_real,
                           float *polar_imag, float *phi, float *phi_ptn,
                           int phi_length, float *theta, float *theta_ptn,
                           int theta_length, float antenna_gain,
                           t_Receiver *ptr_rx_c);

/**
 * @brief Get the number of Rx channels
 *
 * @param ptr_rx_c Pointer to the Receiver
 * @return int Number of Rx channels
 */
EXPORTED int Get_Num_Rxchannel(t_Receiver *ptr_rx_c);

/**
 * @brief Free the memory of Receiver
 *
 * @param ptr_rx_c Pointer to the Receiver
 */
EXPORTED void Free_Receiver(t_Receiver *ptr_rx_c);

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
 * @param location Radar's location {x, y, z} (m)
 * @param speed Radar's speed {x, y, z} (m/s)
 * @param rotation Radar's rotation {x, y, z} (rad)
 * @param rotation_rate Radar's rotation rate {x, y, z} (rad/s)
 * @return t_Radar* Pointer to the Radar
 */
EXPORTED t_Radar *Create_Radar(t_Transmitter *ptr_tx_c, t_Receiver *ptr_rx_c,
                               float *location, float *speed, float *rotation,
                               float *rotation_rate);

/**
 * @brief Free the memory of Radar
 *
 * @param ptr_radar_c Pointer to the Radar
 */
EXPORTED void Free_Radar(t_Radar *ptr_radar_c);

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
EXPORTED t_Targets *Init_Targets();

/**
 * @brief Add an ideal point target to the target list
 *
 * @param location Target's location {x, y, z} (m)
 * @param speed Target's speed {x, y, z} (m/s)
 * @param rcs Target's RCS (dBsm)
 * @param phs Target's phase (rad)
 * @param ptr_targets_c Pointer to the target list
 * @return int Status code (0 for success, 1 for failure)
 */
EXPORTED int Add_Point_Target(float *location, float *speed, float rcs,
                              float phs, t_Targets *ptr_targets_c);

/**
 * @brief Add a 3D mesh target to the target list
 *
 * @param points Mesh coordinates
 * @param cells Mesh connections
 * @param cell_size Number of meshes
 * @param origin Target origin (m)
 * @param location Target location (m)
 * @param speed Target speed (m/s)
 * @param rotation Target rotation (rad)
 * @param rotation_rate Target rotation rate (rad/s)
 * @param ep_real Real part of Permittivity
 * @param ep_imag Imaginary part of Permittivity
 * @param mu_real Real part of Permeability
 * @param mu_imag Imaginary part of Permeability
 * @param is_ground Flag to identify if the target is ground
 * @param ptr_targets_c Pointer to the target list
 * @return int Status code (0 for success, 1 for failure)
 */
EXPORTED int Add_Mesh_Target(float *points, int *cells, int cell_size,
                             float *origin, float *location, float *speed,
                             float *rotation, float *rotation_rate,
                             float ep_real, float ep_imag, float mu_real,
                             float mu_imag, bool is_ground,
                             t_Targets *ptr_targets_c);

/**
 * @brief Free the memory of target list
 *
 * @param ptr_targets_c Pointer to the target list
 */
EXPORTED void Free_Targets(t_Targets *ptr_targets_c);

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
 * @param level Fidelity level of ray tracing
 * @param density Ray density
 * @param ray_filter Ray filter parameters
 * @param ptr_bb_real Real part of baseband samples
 * @param ptr_bb_imag Imaginary part of baseband samples
 */
EXPORTED void Run_Simulator(t_Radar *ptr_radar_c, t_Targets *ptr_targets_c,
                            int level, float density, int *ray_filter,
                            double *ptr_bb_real, double *ptr_bb_imag);

/**
 * @brief Run interference simulation
 *
 * @param ptr_radar_c Pointer to the victim radar
 * @param ptr_interf_radar_c Pointer to the interference radar
 * @param ptr_interf_real Real part of the interference baseband
 * @param ptr_interf_imag Imaginary part of the interference baseband
 */
EXPORTED void Run_Interference(t_Radar *ptr_radar_c,
                               t_Radar *ptr_interf_radar_c,
                               double *ptr_interf_real,
                               double *ptr_interf_imag);

#ifdef __cplusplus
}
#endif

#endif  // RADARSIM_HPP
